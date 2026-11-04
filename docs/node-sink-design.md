# 节点级 Sink 闭包执行器设计

## 目标与验收

目标是让 event T32 先达到 p2sim 同一量级，再争取超过，而不是继续在 HEFT/token
上做局部微调。

里程碑顺序固定：

1. 小模型 dense/event 全差分：reset、queue read-after-write、memory pending、
   multiwriter 同地址顺序、重复 printf。
2. BOOM 50k 三次全寄存器哈希与 dense 一致。
3. BOOM 500k 正确性后，与 p2sim 同会话五轮交错测速；先在 1.20× 内，再争取
   1.05× 以上优势。
4. KMH difftest on/off 各三轮哈希、五轮测速，并记录 EVENT_STATS 实际稀疏度。

任何性能结论必须来自同一 T32、CPU 48-79、同一镜像和同会话配对；dense-only
使用独立生成二进制，不能把携带 event 代码的弱 dense 当作基线。

## 实施结果（2026-11-02）

当前稳定实现为 KaHyPar sink 分区 + 预算复制 + 连续区间 DP + cap2 静态相位链。
小模型 7 项节点测试与 5 项原有 MT 测试均通过。

BOOM LargeBOOM 的最终可复现结果（`/tmp/eventNodeCap2/final-results.json`）：

- 50k 三轮 dense/event 全寄存器哈希均为 `4b78f9a2a4d11493`。
- 500k dense/event 哈希均为 `b7f8056fad9d544a`。
- 500k 五轮 wall 中位数/min/max：event **11.582 / 11.383 / 11.585s**，
  p2sim **3.221 / 3.220 / 3.271s**，event 仍约 3.60× p2sim，**未达到对齐目标**。

随后没有运行时收益的实现均已默认关闭或删除：全托管/块级 rebalance、活动画像成本、
CCD 映射（跨 CCD 权重反而上升）、cap3（正确性失败）。cap2 是当前最佳节点候选，
50k 约 1.1s，500k 约 11.6s。

KMH 尝试在 cap2 下生成 `SimTop.fir`：生成耗时 2473s，产生 **797 TU、
12,928,618,117B 源码、265,723 MTask**。该规模不是可接受的可编译候选，因此未进行
KMH 编译和性能验收，不能宣称节点 sink 路线在 KMH 成立。

后续若继续该路线，必须先解决副作用微任务数量与闭包串行成本，而不是增加复制、
调整 cap 或继续用静态成本模型掩盖运行时差距。

补充：随后尝试按寄存器目的地批处理 REG_DST，静态任务数与 makespan 虽下降，但整组
克隆留下未声明的原始内部节点引用，BOOM 编译失败。该版本已完整回退到 cap2 稳定
实现并重新通过全部小模型回归，不能把其静态下降作为有效结果。

## 已证伪的前置方案

- MTask/细 MTask 的零跨边闭包会把绝大多数成本吸入单分量，不能实现负载均衡。
- 有界 MTask 闭包在 BOOM 上仍产生 1,078,125 的静态 makespan，远高于 HEFT
  169,961，不能进入编译验收。
- per-source plain activation bank 在 50k 后存在非确定性竞态，禁止使用。
- ready-CHG generation 可以消除该类竞态，但实测没有性能收益，只作为后续残余
  跨核边的可选安全机制，不作为主线优化。

因此新工作必须直接下到节点级任务形成，不再给 MTask 分区器增加模式。

## 任务形成与纯度判定

### 钉死的副作用微任务

以下节点各自形成独立、唯一、不可复制的微任务；除状态提交外，不与其他计算
合并：

- `NODE_SPECIAL`：printf/assert，保留原 serialOrder 和 hot/cold 语义。
- `NODE_WRITER` / `NODE_READWRITER`：memory pending 写与 readwriter forwarding。
- `NODE_REG_RESET` 与 async/sync reset 发布。
- `NODE_EXT` / `NODE_EXT_IN` / `NODE_EXT_OUT` 与 host-visible output。
- 状态提交：REG_SRC/REG_DST 的 canonical current/next 更新、memory preCommit、
  sparse register commit。它们只能有唯一 commitOwner。

这些节点是 sink。它们可以读取纯组合副本，但其副作用和写状态顺序必须与原图一致。

### 可复制纯组合节点

同时满足以下条件才允许复制：

- 类型为 `NODE_OTHERS` 或 `NODE_INFER`，无 printf/assert/ext/memory side effect。
- 不写 REG_DST、memory、reset、host output。
- 标量或总元素数不超过 32 的数组；更大数组钉住，避免复制成本和 cache 放大。
- 输入只能是顶层输入、cycle-start 稳定状态、同 worker 纯组合值或同 worker
  唯一微任务输出。跨 worker 的非常规输入必须使用残余跨核边协议。

`NODE_READER` 的存储器读取本身是否可复制由该读是否引用 canonical memory 决定：
readwriter 同端口 write-first 一律不可复制；普通 reader 只在读取 cycle-start 稳定
快照且提交顺序已证安全时允许作为边界输入，不复制存储器本体。

## Sink 闭包与节点图

1. 使用前端原始 Node 图，不使用 `MtTask` 作为纯度或复制单位。
2. 保留全局拓扑序。组合数据依赖是本周期边；REG_SRC 当前态和 canonical memory
   是 cycle-start 边界，不能误当成需要复制的组合生产者。
3. sink 包括所有副作用微任务以及没有本周期后继的节点。
4. 逆拓扑计算每个纯节点的可达 sink 集。纯节点可以复制到每个需要它的 worker，
   但复制不是强制闭包：必须同时通过成本与预算检查。
5. 每个 worker 的链是全局拓扑序过滤后的子序列。链内只使用本地激活掩码，
   不发 ready token。

## 复制代价模型

对候选纯节点 `n` 和目标 worker `w`：

```text
cloneCost(n,w) = lowered_instruction_cost(n)
               + clone_watch_cost(n)
               + per_worker_activation_cost(n,w)
               + estimated_cache_cost(n,w)

saved(n,w) = removed_ready_tokens(n,w) * (tokenLatency + remoteCheck + remotePublish)
           + removed_remote_activation_RMW(n,w)
           + removed_spin_estimate(n,w)
```

只有满足以下条件才复制：

- `saved(n,w) > cloneCost(n,w)`，且确实删除至少一条跨核边；
- 目标 worker 总成本不超过 `cap × ceil(total_original_cost / workerCount)`；
- 首个 cap 固定为 1.25，预算以复制前总成本计算，禁止用新增复制成本继续抬高预算；
- 不复制 always/side-effect 微任务、大数组、多写者状态和 canonical commit；
- 合并或复制不得改变全局拓扑序、reset 顺序和 side-effect serialOrder。

接受残余跨核边。零跨边不是目标，也不允许为了达到零跨边而牺牲负载均衡。

## Worker 链与激活

- 每个 worker 使用 plain 本地 activation mask；首周期全置位，reset 按现有协议重放。
- 纯组合写使用 WritePre/WritePost CHG：小 fanout 无分支传播，大 fanout 条件传播。
- 本地消费者按拓扑序在后续链位置执行，不需要 ready token。
- 副作用微任务保持 hot/cold：触发 printf/assert 时置 hot，hot 期间每周期执行，
  冷任务只在输入变化时执行。
- canonical state 提交保持 dense 的 current/next、pending 和 reset 语义；不得把
  canonical write 挪到普通 worker compute 阶段。
- host-visible output 从本周期 producing worker 读取，不读下一周期 mailbox。

## 残余跨核边

优先使用每条 producer→consumer 边的 ready-CHG generation：

- producer 完成时写 ready parity；只有实际 CHG 时才写 generation。
- consumer 先等 ready，再比较 generation，保证数据就绪和变化语义分离。
- generation 单调递增，避免 parity 重用造成假激活。
- 没有对应 ready slot 的边回退到原子 remote activation mask，不允许 plain bank。

当后续某区域真正达到零跨边时，可改为 p2sim 的双屏障 pull/step/push，但这不是
首个里程碑的前置条件。

## 与 dense 的隔离

- 基线为 `experiment/node-sink`，从 `ca1f141` 的干净工作树开始。
- dense/vcontract 路径保持零行为改动；默认 `GSIM_MT_EVENT_GENERATE=0`。
- 新任务形成使用独立 `mtNodeSink` 数据结构和发射器，不修改 `MtTaskPlan`、
  HEFT、dense scheduler 或 dense state update。
- event 仅由 `GSIM_MT_EVENT_GENERATE=1` 生成；节点 sink 再由独立
  `GSIM_MT_NODE_SINK=1` 选择，便于同图对照。
- 重活一次一件；builder 实现、reviewer 审查；被拒绝探针完整删除，不保留死代码。

## 实施顺序

1. 建立节点纯度、微任务和全局拓扑报告，先验证不生成模型时的分类与成本。
2. 实现单 worker 节点链，再做受预算约束的复制和残余跨核协议。
3. 小模型差分通过后，才允许 BOOM 生成和 50k 哈希。
4. BOOM 500k 达到 1.20× p2sim 后才进入 KMH；未达标继续按阶段画像定位，不能
  用 KMH 或较弱 dense 掩盖 BOOM 差距。

## 测量驱动的架构探索（2026-11-04 用户路线约束）

不做 P2SIM 的等价复刻。等价协议只用于理解瓶颈，最终协议必须在结构上可区分。
P2SIM 仅承担两个角色：性能对照标尺，以及 sink 闭包复制、WritePre/WritePost CHG
等思想参考。每一轮先取得 profiling、阶段画像或定量归因，再只针对当前最大瓶颈
设计和验证优化；禁止因为某个旋钮与 P2SIM 相似就默认它有效。

优先探索自己的机制：依赖驱动权重分区、quiet-range 提交、面向实测热点的大块融合、
以及能降低关键路径而非只降低静态 makespan 的调度。复制、合并和提交必须先保持
reader/memory/reset provenance 与顺序，再讨论性能；跨 worker 的语义闭包和运行时
负载均衡是两个独立约束，不能互相冒充。

### 增量维护清单

有效，继续维护：

- 组掩码：用少量组级测试跳过绝大多数空任务 prologue。
- 读口闭包复制：保留并扩展该方向，但每个副本必须保留 original reader/memory
  provenance、memory 唤醒和 memory-order 边；不能把曾经出现断言的实现直接视为
  已验收版本。

无效，不重复投入：

- 并行 join、共享 ready token、原子计数 MPSC、静态相位、hot-cold 自适应和 LA0。
- 只为对齐 P2SIM 而做的逐点等价协议、无定量依据的 cap/clone-budget/O3/LTO 扫参。

每轮记录：当前瓶颈、证据、假设、改动、正确性门、同场性能、是否保留。被拒绝探针
完整回退；有效增量也必须在后续协议中重新证明语义，不因进入清单而免除差分。
