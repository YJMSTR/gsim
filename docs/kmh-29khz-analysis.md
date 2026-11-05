# 闭源竞品 KMH V3 CoreMark 29kHz 理论逆向分析 + 快速探针

## 96 线程口径修正（2026-11-05 用户补充，优先于下文 T32 结论）

友商 29kHz 的实际配置是 **96 线程**，不是 32 线程。必须并列保留两套预算：

| 口径 | 每周期 wall | 总核时/周期 | 每信号预算（5,903,097 个命名信号） | 2.4GHz CPU 周期/信号 |
|---|---:|---:|---:|---:|
| T32 | 34.48us | 1.103 core-ms | 0.187ns | 0.45 |
| **T96** | 34.48us | **3.310 core-ms** | **0.561ns** | **1.35** |

精确计算为 `96 / 29000 × 1000 = 3.310 core-ms`；与约 3.0 core-ms 的近似值同量级。
T96 使预算恰好放大 3 倍。原先“稠密执行被数学排除”的结论**不再成立**：

- 我方 bodies+state 实测 2.525–3.414 core-ms，相对 T96 预算 3.310 core-ms 的比值为
  **0.76–1.03×**，已落在稠密执行的可达边缘，而不是原先 T32 下的 2.3–3.1× 缺口。
- 4.43s 冠军配方按满占估算的 bodies+state 上限约 2.842 core-ms，也在预算内。
- 因此竞品在 96 线程达到 29kHz **不再要求接近零成本的信号级门控**。H1 稠密执行、
  H3/H4/H5 的代码与调度优化都可能解释该性能；H2 稀疏执行从“必要机制”降为“可选
  加速机制”，其实际收益仍须由 KMH 信号级 census 定量。
- 新关键是**跨 CCD/socket 的可扩展同步**：已有 token 延迟从约 24.5ns 到 322ns 的数据，
  不能按单 socket 成本外推。友商若在 T96 仍达到 29kHz，说明其通信/提交结构在
  多 socket 下也能扩展，这比绝对频率更值得研究。

修订后的 H1–H5 优先级：

1. **先做 dense-T96 KMH CoreMark 实测**，建立同线程数直接标尺；当前 T32 二进制不能
  通过修改 `GSIM_THREADS` 变为 T96，必须确认并生成对应模型。
2. 优先分析 H1（优化稠密执行）与 H4（多 socket 同步/批处理）的组合；H2 稀疏门控
  只作为附加候选，不再用来解释竞品必然实现方式。
3. 若 dense-T96 仍明显低于 29kHz，再分别量化跨 socket 等待、访存和计算三者的占比，
  决定是优化同步结构、位打包还是活动门控，而不是预设结论。

下文保留 T32 分析作为预算对照；其中“稠密排除定理”及 H2 必要性仅适用于 T32。

- 日期：2026-11-05。作者：Competitor29kAnalysis（与 event/AWP 主线并行；未修改 wt-native-p2sim；未跑 KMH 三门；无新编译）。
- 竞品口径（用户给定，作为外部分别记录）：**29kHz**（KMH V3 + CoreMark，闭源竞品，线程/机器/计时口径未知）。
- 我方 dense 参考口径（用户给定）：**4.43s/50k**。本文证明它是"特定冠军配方 + difftest 全开"的口径，非当前任一在盘二进制可复现（见 §1、§4）。
- 一句话结论：**在 32 线程、2.4GHz 级 EPYC 上，"每周期稠密求值全部信号"在数学上被 29kHz 预算排除；竞品若数字同口径，必然依赖（按可能性排序）口径放大（H1，1.2–1.8×）+ 细粒度活跃门控（H2，剩余 2–3×）；单靠调度/同步/代码级优化（H3/H4/H5）三者全叠加也到不了 29kHz。**

---

## 1. 口径盘点：29kHz 到底比我们快多少（三个我方口径必须分开记）

我方 KMH V3 CoreMark 50k（`coremark-2-iteration.bin -C 50000 -I 67944`，T32=mask 48-79，端点一律 67,944 instr / 50,000 cycles，IPC 1.3589）：

| 口径 | 二进制/配方 | wall 中位 (min–max) | us/cycle | 换算频率 | vs 29kHz |
|---|---|---:|---:|---:|---:|
| **A. 4.43s 参考**（用户口径） | Oct-24 冠军配方：生成器 `41c7fe8`（cptopdown+dirty-chunk），交付分支 `db0b1ba` 独立复现 4.43s；r9-report 5 轮中位 **4.44s**（4.39–4.48），**difftest 全开**，2400 MTask | 4.43–4.44 | 88.6–88.8 | 11.26–11.29 kHz | **2.57×** |
| B. 当前在盘干净 dense（Nov-2） | `/tmp/kmh-event-rewrite/{on,off}/gsim-compile/emu`，2242 MTask | **5.829**（5.780–5.881）on / 5.779 off | 116.6 / 115.6 | 8.58 / 8.65 kHz | 3.38× |
| C. Sep-10 冠军参考二进制（本文指定） | `gsim-task-verilator-dual-default4488/champions/newrtl-t32-v4stack-maxmt3200/emu`，12,488 MTask | **10.590**（10.539–10.641）on / 10.592 off | 211.8 | 4.72 kHz | 6.14× |
| （对照）官方 Verilator 5.052 T32 | competitor-bench-table §6 | 17.677（17.104–18.013） | 353.5 | 2.83 kHz | 10.25× |
| （对照）我方 T64 | Oct 冠军配方 | 3.52（3.40–3.62） | 70.4 | 14.2 kHz | 2.04× |

**口径结论：**
1. 4.43s 与 29kHz **不是同一二进制、也大概率不是同一计时口径**；4.43s 属"difftest 全开的 wall、50k 全窗、T32"的冠军配方口径，当前干净 dense（B）比它慢 1.31×，文档已记录"4.43s 未被当前干净 dense 模型复现"（event-rewrite-design.md §157-185）。
2. 我方最好记录（4.43s=11.29kHz）距 29kHz **2.57×**；当前在盘最好（5.78s=8.65kHz）距 **3.35×**；T64 也差 2.04×。
3. 竞品 29kHz 相对官方 Verilator 是 **10.25×**——比 P2SIM 在 large-BOOM 上对官方 Verilator 的量级（待核值）还高，说明该数字若同口径，其实现形态必然显著不同于"稠密并行 Verilator 类"。

---

## 2. 数学：29kHz 的核时预算（用户三参数 + 实测信号数）

**参数（用户给定）**：~百万信号、2400 MTask、32 线程、每周期 34.5us、核时预算 1.08ms。

**精确化：**
- 29,000 cycles/s ⇒ 每周期 wall 预算 = 1/29,000 = **34.48us**。
- 32 线程全占满 ⇒ 核时 = 34.48us × 32 = **1.1034 core-ms/周期**。用户 1.08ms 对应 33.75us/cycle = **29.63kHz**（即 97.9% 线程占满的 29kHz）；两者相差 2.1%，下文用 1.103ms 为准。
- KMH V3 冻结 FIR（`rtl-kunminghu-v3-frozen/SimTop.fir`，1.4GB）实际规模（本文实测，方法见 §7）：
  - 模块体内声明：node 4,802,072 / wire 228,023 / reg 21,414；module 8,170，inst 8,876；
  - **按实例加权展平：node 5,652,755 + wire 229,928 + reg 20,414 = 5,903,097 个命名信号**。
  - 用户"百万信号"是正确量级的保守值；实际命名声明 ~5.9M（编译期常量折叠后有效求值点会少于命名数，取 1M–5.9M 为区间）。

**预算表（每周期、每单位）：**

| 单位 | 预算 @29kHz | @2.4GHz（EPYC 9654 基频，tsc 实测 2,400,001,452Hz） | @3.7GHz 睿频 |
|---|---:|---:|---:|
| 每 MTask（2400） | 459.8 ns | 1,103 TSC-cycles | 1,701 cycles |
| 每 MTask（2242，Nov-2 实际） | 492.1 ns | 1,181 cycles | 1,821 |
| 每信号（按用户 1M） | 1.103 ns | **2.65 cycles** | 4.08 |
| 每信号（按实测 5.9M） | 0.187 ns | **0.45 cycles** | 0.69 |

即：29kHz 下**每个信号的全程摊销成本（求值+调度+同步+内存）必须 ≤ 0.45–2.65 个 CPU 周期**。

**我方实测对照（每周期核时与 ns/信号，信号数按 5.9M）：**

| 配置 | wall/cyc | core-ms/cyc（含自旋） | ns/信号 | cycles/信号@2.4G |
|---|---:|---:|---:|---:|
| Sep 冠军（探针A实测） | 211.8us | 7.030 | 1.191 | 2.86 |
| Sep 冠军 bubble 分解：task_bodies | — | 2.589 | 0.439 | 1.05 |
| Sep 冠军 bubble 分解：state_update | — | 0.825 | 0.140 | 0.34 |
| Sep 冠军 bubble 分解：spin（ready+barrier+idle） | — | 3.222（45.9%） | 0.546 | 1.31 |
| Nov-2 dense（探针A实测） | 116.6us | 3.731 | 0.632 | 1.52 |
| Nov-2 bubble：task_bodies+state_update | — | 2.525 | 0.428 | 1.03 |
| Oct 冠军（4.43s 参考，无 bubble 数据=缺口） | 88.8us | ≤2.842（按满占） | ≤0.481 | ≤1.15 |
| **29kHz 目标** | **34.48us** | **1.103** | **0.187** | **0.45** |

**核心推论（稠密排除定理）：**
1. 我方三个独立画像一致表明：**纯计算（bodies+state）就要 2.5–3.4 core-ms/周期**（Sep 3.414、Nov-2 2.525、Oct 冠军按满占上限 2.84）——全部超过 29kHz 的总预算 1.103 core-ms。**即使调度/同步开销归零，当前代码生成质量下的稠密全量求值也进不了 29kHz**（差 2.3–3.1×）。
2. Sep 冠军的调度理论上限（throughput bound，完美均衡）= 106.7us/cyc（9.4kHz）；Nov-2 = 78.9us/cyc（12.7kHz）；零等待 floor 分别 162.1/100.3us。**H4 类（调度）天花板 ~1.19–1.36×，远小于 2.57× 缺口。**
3. 因此竞品（若同口径）必须把每周期实际求值的信号/工作量压到稠密的 **≤1/3**：按我方 bodies 成本 0.44ns/信号折算，29kHz 预算只够稠密求值 ~40% 的信号且零调度开销——**细粒度活跃门控（H2）是唯一具有 ≥2.5× 数学余量的机制**；或者其口径被放大（H1）。
4. 频率敏感性：全部 cycles/信号按 2.4GHz；即使 3.7GHz 全核睿频（+54%），稠密排除定理仍成立（0.69 vs 我方 bodies 1.05+ 同比折算）。

---

## 3. H1–H5 假设矩阵

| # | 机制（竞品靠什么到 29kHz） | 与现有代码距离 | 预估加速（口径放大倍数） | 优先级 | 判定证据 |
|---|---|---|---|---|---|
| **H1 口径/测量差异**：内部计时器 vs wall、短窗稳态、difftest 关、不同机器/频宽 | 零（纯分析+已有探针） | **1.2–1.8×**（有据上限：P2SIM 内部 263.3 vs wall 145.4 kcycles/s=1.81×；BOOM last-window 207 vs 均值 168=1.23×；difftest 税实测 ≤0.9%） | **1（先校准目标）** | 探针A：difftest 税 Sep 冠军 −0.02%（区间全重叠）、Nov-2 +0.86% ⇒ **difftest 不是缺口来源**；剩余可疑项=内部计时/短窗/机器 |
| **H2 细粒度活跃门控/事件驱动**（只求值活跃锥；信号级 ~99% 每周期不变） | **大**：现粒度直接实验已负收益（event 1.26× 慢于 dense）；KMH 节点级生成在我方链条爆炸（265,723 MTask/12.9GB 源码，未编译）；激活扇出跨区（Aug 普查：中位 word 由 9–10 lane 写，仅 0.37% 站点 plain-eligible）⇒ 需执行器所有权重设计 | 理论 **2–3×**（若门控开销 ~0）；现状实测 **0.79×**（回退） | **2（唯一余量足够的机制，研究级）** | 探针B：KMH 任务级稀疏度 47.5%（2242 MTask）⇒ 当前粒度门控上限仅 2.11× 且 54.8% 执行集中在永热 26% 任务；信号级无 KMH 直接普查（证据缺口）；BOOM 侧 p2sim 6.4–7.7× 证明该机制上限真实存在 |
| **H3 值不变缓存/跨周期脏传播**（#106：task 输出不变率 74.5% calls / 67.7% body） | 中大：联合可预测性塌缩、全周期 checkpoint 1.24× 成本已被定量拒绝（当前放置） | ≤1.3–1.5× | 4 | candidates.jsonl #106（2026-10-26 REJECT current placement）；仅作 repartition 后研究候选 |
| **H4 调度/同步/负载再压缩**（自旋 22–46%、avg_par 15.5–21.1/32） | 中：wave pipelining、tree helper、speculation、rebalance、join 重定价等轴已全部关闭；残余=结构性协议 | **1.19–1.36×**（bubble 实测 headroom_pure：Sep 1.36×、Nov-2 1.19×） | 3 | 两份 bubble 画像（§2 表）；Oct 冠军无画像=缺口 |
| **H5 稠密核代码级**（布局/PGO/内联/字级批处理） | 小但轴已关闭：PGO +7.5% 慢、表序 +29.9% 慢、inode 布局 ±50% 运气、fused-ring +0.51% PARKED | ≤1.2× | 5 | benchmark.csv 各 REJECTED/PARKED 行 |

**叠加判定**：H3+H4+H5 全赢 ≈ 1.19×1.35×1.2 ≈ 1.9× < 2.57×（对 4.43s 口径），< 3.35×（对当前在盘口径）。**唯一够得着 29kHz 的组合是 H1(1.2–1.8×) × H2(2–3×)**；若 H1=1（完全同口径 wall），则需要 H2 实际兑现 ~2.6–3.4×，即竞品必须拥有"开销≈0 的细粒度门控"，这在我们的三次实现尝试中从未接近（最好 0.79×）。

---

## 4. 探针 A：同一 dense emu+image，difftest on/off 的 50k wall（原始结果）

- 目的：(a) 实测 difftest 税，检验"竞品 29kHz 是关 difftest 跑出来的"；(b) 确认 4.43s 参考值与当前实际值是否同口径。
- 方法：同会话交错 4 格 ×（1 次热身丢弃 + 5 正式轮），持锁 `/tmp/gsim-vcontract-ablation/measurement.lock`，mask 48-79，`GSIM_THREADS=32 GSIM_MT_EXECUTOR=dense GSIM_MT_CPU_AFFINITY=auto NEMU_HOME=RTR`，全窗 50k（`-C 50000 -I 67944 -b 0 -e 0`），端点每轮校验。短窗（如 -C 2000）不作为真值依据；首轮冷启动按协议丢弃。
- 负载条件：跑前 load 4.13（clang++ 兄弟构建在 CPU 141/227/252，不占 48-79）；第 5 轮期间机器 load 冲到 33.7（兄弟构建churn），on/off 同轮交错故中位数不受方向性影响，区间如实保留。

| 格（emu × difftest） | 5 轮 wall (s) | 中位 | 端点 |
|---|---|---:|---|
| Sep 冠军 ON（`--diff riscv64-nemu-interpreter-so`） | 10.539, 10.541, 10.590, 10.591, 10.641 | **10.590** | 67,944/50,000 ×5 |
| Sep 冠军 OFF（`--no-diff`） | 10.489, 10.490, 10.592, 10.641, 10.790 | **10.592** | 67,944/50,000 ×5（内部计数器） |
| Nov-2 ON | 5.780, 5.780, 5.829, 5.829, 5.881 | **5.829** | 67,944/50,000 ×5 |
| Nov-2 OFF（不加载 ref） | 5.731, 5.779, 5.779, 5.780, 5.830 | **5.779** | Guest 50,001 ×5 |

**结论：**
1. **difftest 税：Sep 冠军 −0.02%（区间完全重叠，=0）；Nov-2 +0.86%**。NEMU 逐指令比对完全被并行空转遮蔽。**"竞品关 difftest 所以 29kHz"不成立**（最多解释 ~1%）。
2. **4.43s 与当前实测不同口径（不同二进制/配方）**：同一会话里 Sep 冠军 10.59s、干净 dense 5.83s，而 4.43s 出自 Oct-24 冠军配方（cptopdown+dirty-chunk，2400 MTask）。Nov-2 数据与 2026-11-02 记录（5.830/5.780）逐位复现，交叉验证测量有效。
3. 29kHz 即使对 4.43s 也是 2.57×；对当前在盘最好实现是 3.35×。

**复现命令**（脚本 `/tmp/kmh29k-probeA/run-probeA.py`；结果 `probeA-results.json`；逐轮日志 `/tmp/kmh29k-probeA/*.log`）：
```bash
cd /home/zhangyangjie/test/XiangShan/ready-to-run
env GSIM_THREADS=32 GSIM_MT_EXECUTOR=dense GSIM_MT_CPU_AFFINITY=auto \
    NEMU_HOME=$PWD taskset -c 48-79 <EMU> \
    -i coremark-2-iteration.bin --diff riscv64-nemu-interpreter-so \
    -b 0 -e 0 -C 50000 -I 67944        # ON 臂；OFF 臂改 --no-diff（Sep emu）或去掉 --diff（Nov-2 off 构建）
```

## 5. 探针 B：EVENT_STATS 任务/信号级活动率 —— H2 判定

**主证据（KMH、可靠）**：`/tmp/kmh-event-rewrite/event-stats-{on,off}.csv`（2026-11-02 诊断运行，KMH V3 CoreMark 50k，2242 MTask，`GSIM_MT_EVENT_STATS`，与测速分离；on/off 各自 dense/event 哈希一致）。
- 总执行 task-body：on 53,265,078 / off 53,260,673 ⇒ **任务级稀疏度 47.52% / 47.51%**（按 2242×50k 折算），与设计文档记录一致。
- 本文新增分布分析（50,000 cycles 折算执行率）：

| 执行率桶 | 任务占比 | 备注 |
|---|---:|---|
| ≥0.99（永热） | **25.9%** | 贡献 **54.8%** 的总执行量（29.2M 次） |
| 0.95–0.99 | 0.1% | |
| 0.50–0.95 | 21.4% | |
| 0.10–0.50 | 23.6% | |
| 0.01–0.10 | 11.4% | |
| <0.01（近冷） | 17.7% | 从不执行的任务：0 个 |

- **H2 判定（当前 2242-MTask 粒度）：不成立。** 上限=跳过 52.5% 执行 ⇒ body 工作最多省 2.11×，而实测 event 执行器比同模型 dense **慢 1.26×**（7.334 vs 5.830s；cross-core 组合关键路径、spin-ready ~70%）——门控开销吃掉全部红利还倒贴。自适应 hot/cold（95% 阈值）、LA0 变体均已实测更差并回退（event-rewrite-design.md §204-221）。
- **H2 判定（信号/锥级粒度）：证据缺口，不伪造。** KMH 没有信号级 value-change 普查。最近的三个锚点（口径不同，不可混用）：
  1. 2026-08-16 旧图（12,275 MTask）动态轨迹 oracle：每周期 active MTask 76 个 = **0.62%**（CoreMark）/1.10%（Linux）——值变化视角，支持"~99% 逻辑每周期不变"；
  2. 本次 2242 粒度执行率 47.5%——输入唤醒+语义唤醒视角（clock/valid/monitor/WritePre-Post）；
  3. BOOM（非 KMH！）native 线 CSV：`/tmp/native-original-stats.csv`（135,244 单元，均值执行率 27.3%）、`/tmp/native-event-stats.csv`（28,456 单元，37.6%）、`/tmp/native-groupmask-stats.csv`（27,309 单元，38.5%）——行数与 `/tmp/eventNodeCap2-activity.csv`(28,456) 吻合 ⇒ **BOOM node-cap2 图产物，不是 KMH**，只能作机制旁证：p2sim 在 BOOM 上 6.4–7.7× 超过 gsim-mt dense，证明细粒度事件驱动上限真实；但我方同路线实现仍 3.60× 慢于 p2sim（node-sink-design.md §31）。
  - 结论：**KMH 上 H2 的真实奖金取决于信号级变化率与门控开销之比，现有数据无法定论；任务级已证伪。** 若信号级确为 ~1% 变化，门控检查本身须 ≤0.19ns/信号（0.45 cycle）才不爆预算——我方历史最好门控实现开销 115–427us/周期（4.8× wall），差距 3 个数量级。

## 6. 后续实验建议（1–2 个）

- **E1（决定性，先做）——KMH 信号级 word 粒度 value-change 普查**：在现有 dense 冠军模型生成器（`41c7fe8` 树）上按"census before emit"协议只加计数器（分组 word 写站点、单写者 plain increment），CoreMark 50k 统计每周期变化 word 占比 + 每 word 的 lane 分布。产出：H2 奖金是 ~2× 还是 ~10× 的直接答案；同时复核 Aug 0.62% 在新图上的对应值。前提：一次有界仪表构建（非长编译；可 O1 病态 TU 覆盖）。
- **E2（零构建，立即）——Oct 冠军/当前最好 dense 的 bubble+duty 画像**：对 `/tmp/kmh-event-rewrite/{on,off}/gsim-compile/emu` 跑 `GSIM_MT_BUBBLE_PROFILE=1` + `GSIM_MT_DENSE_DUTY`（已有旋钮，~6s/次），把 §2 表的"Oct 冠军无画像"缺口补上，锁定 H4 剩余 headroom 的精确值。本文已顺手完成 Nov-2 on 臂：avg_par 21.1/32、all_spin 22.2%、task_bodies+state 2.525 core-ms/cyc、headroom_pure 仅 1.19×（`/tmp/kmh29k-probeA/bubble/mt-bubble-nov2on-20261105.json`，诊断口径 wall 5.996s，含 ~2.9% 仪表失真）。
- （口径侧，零成本）向竞品口径追问三个问题：29kHz 是 wall 还是内部计时器？窗口多长（短窗稳态 vs 全窗）？difftest/检查是否在环？——按 P2SIM 先例，仅"内部 vs wall"一项就能差 1.8×。

## 7. 证据与复现路径

| 项 | 路径 |
|---|---|
| 探针A脚本/结果/逐轮日志 | `/tmp/kmh29k-probeA/{run-probeA.py, probeA-results.json, probeA-console.log, *.log}` |
| Nov-2 bubble 画像（本次新采） | `/tmp/kmh29k-probeA/bubble/mt-bubble-nov2on-20261105.json`（注意：运行时覆写了 RTR 同名文件，已通知 Main；Sep 画像关键数字保存在本文 §2） |
| KMH 任务级活动率原始 CSV | `/tmp/kmh-event-rewrite/event-stats-on.csv` / `event-stats-off.csv`（2242 行，`任务id,执行次数`） |
| Nov-2 dense/event 五轮正式数据 | `/tmp/kmh-event-rewrite/results.json`；设计记录 `wt-native-p2sim/docs/event-rewrite-design.md` §157-221 |
| 4.43s/4.44s 参考值 | `/tmp/r9-report/summary-clean.json`（T32 samples 4.39–4.48，端点 5/5）；`competitor-bench-table.md` §6 |
| Verilator/竞品对照表 | `/home/zhangyangjie/test/competitor-bench-table.md` |
| 信号数统计 | `rtl-kunminghu-v3-frozen/SimTop.fir`：`grep -c '^    node|wire|reg|inst '` + `grep -c '^  module '`；展平加权 awk（本文 §2，逐行可复跑） |
| 旧图 0.62% oracle 与门控开销 | `gsim-task-verilator-dual-default4488/candidates.jsonl`：`exp-sparse-oracle-ceiling`、`exp-sparse-gate-overhead-final` |
| Aug-16 以来调度轴关闭记录 | 同上 candidates.jsonl 及 `benchmark.csv` 尾部 |

## 8. 附注与边界

- 本文全部数字为同机实测或既有 ledger 引用，无外推值；`[INFERENCE]` 级判断仅在 §3 矩阵"预估加速"列，且都给出依据上限。
- 未修改 wt-native-p2sim；未跑 KMH 三门（coremark/linux1/linux2 全门）；无新编译；RTR 目录一个诊断 json 被覆写已通报并保全数据。
- 竞品为闭源：本文仅从公开性能量级（29kHz）与我方同工况实测做预算反推，不猜测、不复制其源码实现。
