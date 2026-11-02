# T32 Event 后端重写设计

## 目标与验收顺序

1. **50k 正确性**：小模型全回归、LargeBOOM 三次 50k 全寄存器哈希与 dense 一致，且无重复运行偏差。
2. **500k 对齐 p2sim**：同一 T32、同一 `coremark.bin`、同一批 CPU，五轮交错测速，目标不超过 p2sim 的 1.20×。
3. **超越 p2sim**：在正确性和 500k 对齐通过后，用 block 合并、quiet-range 比较和分区权重优化取得超过 p2sim 的实测收益。

所有测量只比较 T32，不引用 k4/k2 作为基准。每轮都先验证 cycle 数和架构状态，再测 wall time。

## 根因

当前 event 后端同时存在以下非等价开销：

- OwnerCurrent/OwnerNext 双文件及每周期 commit 扫描，既重复写状态又引入大量不可预测分支。
- MemoryStage/pending 的跨核共置约束，导致少数 worker 承担绝大部分工作，其他 worker 在屏障处自旋。
- 激活、提交和 mailbox 的粒度比 p2sim 更细，块数、CHG 检查和调度分支显著增加。
- 头文件和发射器中的全量扫描造成生成时间失控，与运行时问题相互放大。

这些不是调整 mask、编译选项或 QCDP 权重能够修复的结构问题，必须改执行模型。

## p2sim 的实际执行模型

逐源码对照后，p2sim 的关键点不是“普通数组跨核直写”，而是：

1. **每 partition 本地状态**：组合块按拓扑序执行，寄存器/next 只作为块内临时值，不存在独立的每周期 commit block 阶段。
2. **WritePre/WritePost**：写前保存 `OLD`，写后得到 `CHG`；小 fanout 用无分支 `act |= CHG << bit`，大 fanout 用条件分支。
3. **本地 plain act mask**：单写者线程内普通 OR，跨核通信由相位屏障而非 atomic RMW 保证。
4. **两阶段周期**：`B1 -> pull -> step -> B2 -> push`。跨核 shared state 由 writer 在 push 阶段发布，reader 下一周期 pull 使用，普通 memory 不做跨核当周期 write-first。
5. **single-writer shared state**：pure comb producer 通过复制闭合消除跨核组合依赖；shared state 每个写者分区唯一。

## 新执行模型

### 1. 状态与提交

- 每个 worker 只保留一份当前状态。OwnerNext 不再作为独立状态文件存在，next 仅是 NextState producer 的块内临时值。
- NextState 块放在该 worker 扫描末尾，直接写当前状态，并执行 WritePre/WritePost CHG。NextState 的尾提交不再另设 block 或 act bit。
- 所有 NextState 提交必须晚于本 worker 的其他计算和 side effect，保证其他读者仍读到提交前状态；禁止把提交提前到 producer 中间。
- 仅 queue pointer/occupancy 等确实需要 dense 提前语义的寄存器保留 early publish，位置在扫描前和扫描后各一次。early 集合必须有明确布局证据，禁止扩大。
- host-visible output 仍取同周期 producing worker 的值，不读下一周期 mailbox。

### 2. CHG 与激活

- 每个被写值只在有消费者或需要跨核发布时做 WritePre/WritePost；不做全量 OwnerNext 比较。
- fanout < 5 采用无分支激活，较大 fanout 用条件分支，避免目前每个值一个不可预测分支。
- 本地 act mask 为 plain `uint32_t`。跨核写只允许在 B2 前进入 remote atomic mask，B2 后合并到本地 mask；B2 后没有跨核写路径。
- commit 不再拥有独立激活几何。quiet commit range 只在确实有 next 变化时执行，否则整块跳过。

### 3. MemoryStage/pending

- 每个 writer port 保留唯一 fixed staging instance，address/data/valid 同组发布，禁止拆组。
- 同一 RAM base 只有一个 commitOwner，preCommit 严格按 commitOrdinal 顺序提交，且不受 reader 是否 active 影响。
- **canonical memory 的唯一写点是 preCommit（B2 前）**。B2 后所有 worker 对 canonical memory 只读，删除跨核 reader-entry 和 writer-tail 的二次 apply。
- remote pending slot 只能读本周 pull 后的 commit-owner mirror；普通 remote reader 看见上一拍提交。
- NODE_READWRITER 只使用同端口 pending forwarding 实现 write-first，不借助 shared canonical write。
- sparse reset 只写 worker-private staging，preCommit 消费并发布，禁止 reset 直接写 canonical 的旁路。

### 4. Side effect 与 reset

- printf/assert 继续 fixed、exactly-once，使用 serialOrder 和 quiescent deferred merge 排序；合并块只减少扫描开销，不改变输出顺序。
- reset publish 必须在扫描前执行；reset staging 在 preCommit 消费。保留现有通过回归的 reset/early-commit 顺序。
- 初始化保持原顺序：全部 Init、全部 worker Seed、最后 canonical cycle-0 seed，避免 reset 和 mailbox 初值竞态。

### 5. 发射与头文件

- 保留 `format()` 的 `thread_local` 缓冲区，保证并行发射安全。
- 主声明头只含 forward declaration 和共享声明；每 worker 状态头单独生成，按函数实际引用包含，避免所有 TU 解析 40MB 单头。
- producer/name/field/array 查询全部建立索引，禁止 per-instance/per-probe 全量扫描。

## 超越 p2sim 的机会

1. **NextState 尾提交融合**：保持与 p2sim 相同的无独立 commit 阶段，同时去掉本后端现有 commit block 扫描，减少每周期固定工作。
2. **Side effect 合并块**：将 64 个实例合并为一个 act block，保持 serialOrder，减少每周期 mask/bit 测试和分支。
3. **Quiet-range commit**：对连续 quiet state 只做一次 range memcmp，避免逐字段比较；发生实际变化再回退到逐寄存器路径。
4. **依赖驱动的负载权重**：用真实活动量和跨核流量而不是静态 eta 分配工作，降低屏障自旋，目标是优于 p2sim 的 KaHyPar 静态划分。
5. **生成/编译并行化**：并行 worker 发射和依赖头拆分缩短迭代时间，支持更多性能实验。

这些优化只有在实测证明不劣于 p2sim 时才晋升，不能用生成时间或小模型结果代替运行时收益。

## 实施步骤

1. 在 `experiment/event-rewrite` 干净分支实现状态/提交路径，先不使用旧 event 文件作为增量补丁底稿。
2. 先跑小模型和 BOOM 特征夹具，重点覆盖 reset、queue read-after-write、remote pending、multiwriter 同地址顺序。
3. 做 LargeBOOM 50k 三次全寄存器哈希对照，之后才开始性能测量。
4. 500k 五轮交错对齐 p2sim；未对齐则按阶段画像定位，不继续无证据微调。
5. 对齐后逐项做超越实验，逐项保留或回退。

## 测量协议

- 模型：LargeBOOM，同一 FIR、同一 `coremark.bin`。
- 线程：T32，CPU `48-79`，`GSIM_EVENT_BATCH=MAX_CYCLES`。
- 正确性：dense 全寄存器哈希，50k 连续三次；500k 再做一次终态/CRC/PC 对照。
- 性能：同会话交错运行 p2sim 与候选，五轮，记录每轮 wall time、median/min/max；有系统活动时整组作废。
- 里程碑分别记录生成时间、构建时间、二进制 provenance、实际 cycle 数和日志路径。

## 独立工程补丁

以下工程收益与重写分离整理，不作为重写设计的隐含依赖：

- `format()` 静态缓冲区改 `thread_local`。
- 主声明头/每 worker 状态头及依赖包含。
- producer/name/field/array 索引化，消除生成期二次复杂度。
- reset publish、commit/push 一对多映射和相关正确性修复。

该补丁单独可合入主线，重写分支不反向包含历史性能实验改动。

## 当前实施状态（2026-11-02）

### 实现路径调整

没有复活旧 event 文件，而是在干净基线的 dense 原生 MTask/状态提交之上增加
`GSIM_MT_EXECUTOR=event`。dense 默认路径和 packed register commit 保持原样，event
复用同一份 canonical state、pending memory commit 和异步 reset 顺序。

已实现：

- compute lval 的 WritePre/WritePost CHG；大对象不做栈上旧值副本，退化为直接唤醒。
- packed register 在 memcpy 前逐字段比较 source/destination，memory/sparse writer
  按 valid 唤醒，sync reset 同时唤醒 reader 和 next-state producer。
- 顶层输入按周期入口 CHG 唤醒，不再把引用 reset/clock 的任务静态设为 always。
- `NODE_SPECIAL` 使用 hot/cold 门控：`gprintf` 触发置 hot；hot 任务继续每周期执行，
  冷任务仅在依赖变化时执行，保持重复打印/断言语义。
- 本地 plain activation mask 与跨核 atomic remote mask；dense token DAG 继续负责
  数据就绪顺序。LargeBOOM 使用 lookahead=0，事件稀疏下 lookahead=64 反而增加开销。
- 可选 `GSIM_MT_EVENT_STATS=<csv>` 记录真实任务执行次数，诊断默认关闭。

### 可复现结果

模型：`/tmp/eventRewriteBest`，生成约 28s、101MB、12 个模型 TU；脚本：
`/tmp/measure-event-rewrite.py`，完整结果：`/tmp/eventRewriteBest/final-results.json`。

- 50k：dense/event 各三轮，32 个 canonical register block 的总哈希均为
  `3957d36ff148eced`。event 外部 wall 中位数 1.570s，dense 2.020s。
- 500k：dense/event 哈希均为 `0906f44ae933cb55`；随后 p2sim/event 五轮交错，
  p2sim 外部 wall 中位数 3.173s，event 15.73s，尚未达到对齐里程碑。
- LA0 bubble 的 zero-wait compute 为 0.306s/50k，而实测 1.611s/50k；
  spin-ready 约 70%。因此后续问题已收敛到跨核组合关键路径和调度，而不是任务体计算。

### 已定量拒绝并回退

- producer-task 共享 token：50k 1.819s，比最佳 1.493s 慢，cache-line 竞争增加。
- 原子依赖计数 + MPSC ready queue：13.234s。
- 44 个静态 cross-edge phase + 屏障：3.585s。
- `rheft maxops=8, fanout=2`：1.507s，无收益；R64 与 R16F1 在 dense 也超时，
  不进入候选。
- worker 内 remote-fanout 拓扑重排：热点 task2220 从 57/64 变为 59/64，无效果。

### 下一步

实现 p2sim 等价的 pure-comb 复制分区，消除跨核同周期组合依赖；不能再通过
token/queue/phase 微调替代该工作。复制后仍按 50k 哈希、500k 五轮交错顺序验收。

独立工程补丁已生成：`/home/zhangyangjie/test/engineering-fixes.patch`（880334 字节），
包含旧工作树中的 thread_local、头文件拆分、索引化和 reset/commit 修复，与当前干净
重写分支分离。

### 香山 KMHV3 CoreMark 50k（2026-11-02）

使用冻结 RTL `rtl-kunminghu-v3-frozen/SimTop.fir`，与 dense 冠军相同的 HEFT、
LA64、supernode30、T32 和 O3 编译参数；CPU 48-79，同一
`coremark-2-iteration.bin`。没有为追求正收益调整参数。模型/日志位于
`/tmp/kmh-event-rewrite`，测量脚本 `/tmp/measure-kmh-event-rewrite.py`，完整结果为
`results.json`。pure-comb 分区工作在本测量前后串行排队，没有并发重编译。

构建事实：

- 生成 960s，250 个模型 TU，模型源码 2,448,388,398 字节。
- difftest-on 完整编译约 305s，difftest-off（`NO_DIFF=1`）302s；on 使用
  `--diff riscv64-nemu-interpreter-so`，off 不加载 reference。
- on 的 NEMU 端点每次均为 67,944 instructions / 50,000 cycles。

正确性：on/off 各自 dense/event 同轮三次、32 个 canonical register block 全哈希对照。
on 三轮双方均为 `c2513ddc1ca2a78d`，off 三轮双方均为 `ee7c29c70bb6befe`。
两种口径哈希不同是 difftest 状态不同所致，各自内部完全一致。

性能（同会话交错五轮，外部 wall，秒）：

| 口径 | dense median/min/max | event median/min/max | event/dense |
|---|---|---|---|
| difftest-on | 5.830 / 5.781 / 5.880 | 7.334 / 7.285 / 7.385 | 1.258× |
| difftest-off | 5.780 / 5.728 / 5.829 | 7.333 / 7.283 / 7.334 | 1.269× |

on/off 的 wall 差异很小；收益为负，event 比同模型 dense 慢约 26%。已知 dense
冠军口径 4.43s 未被当前干净 dense 模型复现（本次约 5.78s），因此这里只作同模型
对照，不把 event 与 4.43s 直接拼成晋升结论。

稀疏度来自独立的 `GSIM_MT_EVENT_STATS` 诊断运行，不混入上述测速：on 执行
53,265,078 个 task-body，off 执行 53,260,673 个；按 2242 个 MTask、50k cycles
计算，实际任务稀疏度分别为 **47.52% / 47.51%**，高于预期约 25%。这与本次负收益
一致，未通过关闭统计或选择其他 workload 美化结果。

### 后续路线决策（2026-11-02）

当前干净 dense 未复现 4.43s 冠军，所有 event/dense 相对数字只用于同模型对照，
不用于宣称超过冠军。后续验收必须同时保留绝对 wall 和同模型对照，避免弱基线
造成假收益。

先在 BOOM 上尝试 sink 超图式闭包复制。该原型能消除全部跨核 task edge，但在当前
MTask 粒度只有 228/2330 个任务可复制，非纯 monitor/state 任务及其下游被迫合并成
3 个分量，静态 makespan 从 169,961 增至 2,061,080（约 12×），因此未编译验收，
实现已删除。结论是：不能直接在当前粗粒度 MTask 上套用 sink 闭包；继续追加约束
修复不会解决 monitor 与纯组合共块的问题。

下一步改为每任务单向自适应 hot/cold：首 1024 cycles 统计真实执行率，达到 95%
的任务永久走 dense 路径，其余任务保留 event 门控；仅当至少一个消费者仍为稀疏
任务时保留 WritePre/WritePost 和 activation。这样不改变状态/reset/pending 语义，
也不需要 profile 文件或参数扫描。BOOM 50k 首轮为 1.515s，与原最佳 1.493s 基本
持平，不作为收益证据。香山实际活跃度 47.5%，更有可能减少热任务 overhead，因此
下一步只在 difftest-off 口径检验该路线；若不改善则停止，不同时开展 LA0 或其他
重编译实验。

自适应香山 off 实测已完成：生成 959s、编译 843s、模型 2,500,019,050B。三轮
 dense/event 哈希仍均为 `ee7c29c70bb6befe`，但五轮 wall 中位数 dense 7.384s、
event 9.589s，反而比原实现的 5.780s/7.333s 更差。额外模式读取和更复杂的生成
代码形态同时拖慢 dense 与 event，因此该路线已回退。下一步仅做 KMH LA0 对照，
不改变任务体或激活逻辑。

KMH LA0 也已完成：生成 995s、编译 253s，三轮哈希均为 `ee7c29c70bb6befe`。
五轮 wall 中位数 dense 6.427s（min 6.426 / max 6.528）、event 7.629s
（min 7.579 / max 7.630），均差于 LA64 的 5.780s/7.333s；EVENT_STATS 稀疏度仍为
47.51%。因此 LA0 同样回退，不再继续 lookahead 或 hot/cold 参数试验。

目前保留的最佳实现仍是 LA64 + 原始 hot/cold。唯一有结构依据的后续方向是先把
monitor/side-effect 与纯组合计算拆成更细任务，再重新评估 sink 闭包；在粗粒度任务上
继续复制、调度或自适应都已用数据否定。

### 生成期 dense/event 分离与最终验收（2026-11-02）

细粒度 sink 复核进一步表明：9280 个任务中 2270 个纯任务，但零跨边约束经
unique-pred×succ 连通把 99.4% 成本吸收到单一 worker，estimated-makespan 5,546,806，
且没有实际复制收益。该约束模型在当前任务图上不成立，实现和 CLI 已完整移除，不再
作为后续方向。

弱基线的一个实际来源是 dense 模型也被迫携带 event guard/watch/CHG。新增生成期
`GSIM_MT_EVENT_GENERATE=1`：默认生成完全 dense-only 模型，只有显式开启才生成
event 代码；event 模型仍保留运行时 dense/event 切换用于正确性对照。小模型 5/5
回归通过，dense-only 与 event 模型分开验证。

香山 dense-only 源码 2,097,562,159B（event 2,448,388,398B，减少 14.3%），off 编译
117s（event 302s）。与既有 event 独立二进制做三轮哈希和五轮 off 交错：三轮哈希
均为 `ee7c29c70bb6befe`；dense-only wall 中位数/min/max 为
**5.130 / 5.128 / 5.180s**，event 为 **7.284 / 7.235 / 7.335s**，真实 event/dense
为 **1.420×**。相比旧同模型 5.780s 基线，dense-only 确实更快，但仍未复现 4.43s
冠军，不能宣称赶超。结果：`/tmp/kmh-dense-only/results.json`。

BOOM 双模型最终结果（`/tmp/eventRewriteEventGen/dual-results.json`）：

- 50k 三轮 dense/event 哈希均为 `3957d36ff148eced`；500k 均为
  `0906f44ae933cb55`。
- 500k 五轮 wall 中位数/min/max：dense-only **19.263 / 19.066 / 19.467s**，
  event **15.507 / 15.457 / 15.654s**，p2sim **3.173 / 3.123 / 3.174s**。
- event 仍约 4.89× p2sim，未对齐；event 快于 dense-only 不构成超过 p2sim 的证据。

最终决策：保留 LA64 + 原始 hot/cold + 生成期分离，不晋升任何 sink、自适应或 LA0
实验。后续若重启闭包复制，必须先拆分 monitor/side-effect 与纯组合任务并放宽
零跨边约束，不能复用当前已证伪的粗粒度方案。
