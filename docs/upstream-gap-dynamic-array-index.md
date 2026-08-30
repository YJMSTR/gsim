# 上游缺口：splitArray 对动态索引数组写的处理（when-展开修复记录）

## Summary

上游 gsim（origin/master）的 `src/splitArray.cpp` 在 `distributeTree` 中对所有数组写
的左值索引调用 `ENode::getIdx()`，并断言结果是非负常量区间。对运行时计算的索引
（FIR 中形如 `connect arr[dynIdx], val`，`dynIdx` 是普通表达式节点而非常量），
`getIdx()` 返回 `(-1,-1)`，断言

```
Assert(begin >= 0 && end >= begin, "Invalid index for array %s: %d-%d", ...)
```

直接失败（对 `(-1,-1)` 渲染为 `... : -1--1`）。

该缺口在旧 XiangShan v86 RTL 上从不触发（无此构造），在新的 kunminghu-v3 RTL
（v86 之后 253 个提交，FIR 1.46GB、sha256 前缀 `5f97158e0064aec5`、2007 个 .sv）
上必然触发：FTQ 性能元数据 `curPerfMeta` 用运行时计算的 `cfiPosition` 写
`cfiAttr`/`isCfi` 数组。本分支已修复并完成验证（提交 `770bab7` 引入 when-展开，
`e710a43` 修正其 else 分支为验证过的 EMPTY 变体）。本文记录触发构造、根因、四种
失败尝试及其失败原因、最终修复、正确性论证、验证证据，以及按 origin/master 视角
的修补指引，供未来上游 PR、回归参考与新成员入门使用。

关键事实：**动态索引"读"在上游本来就有支持**（`ExpTree::updateWithSplittedArray`
对 `getIdx < 0` 的读构造 mux 链），缺的只是动态索引"写"在主拆分环内的处理。
`ENode::hasVarIdx()`（`src/ENode.cpp:329`）在上游已定义但全库无任何调用点——
是明显的"想加守卫而未加"的半成品。

## 1. 触发构造

新 FIR（kunminghu-v3）中 FTQ 引入 `curPerfMeta` 结构：当前盘上的
`/home/zhangyangjie/test/XiangShan/build-sv/rtl/SimTop.fir` 中 `curPerfMeta`
出现 120 次；冻结的旧 RTL `/home/zhangyangjie/test/rtl-v86-frozen/SimTop.fir`
中为 0 次——该构造是全新的。

触发写（FIR 第 2124270 行，位于 `when beforeKnownMispredict_1 :` 块内）：

```
connect curPerfMeta_1.cfiAttr[newBranchInfo_1.cfiPosition], newBranchInfo_1.attribute @[src/main/scala/xiangshan/frontend/ftq/Ftq.scala 475:56]
```

紧邻的同类写（第 2124269 行）：`connect curPerfMeta_1.isCfi[newBranchInfo_1.cfiPosition], UInt<1>(0h1)`。

索引 `newBranchInfo_1.cfiPosition` 是运行时计算的（FIR 第 2124235–2124241 行）：

```
node _newBranchInfo_cfiPosition_fullPosition_T_2 = cat(io.fromBackend.resolve[1].bits.pc.addr, UInt<1>(0h0))
node _newBranchInfo_cfiPosition_fullPosition_T_3 = bits(_newBranchInfo_cfiPosition_fullPosition_T_2, 4, 1)
node newBranchInfo_cfiPosition_fullPosition_1 = add(io.fromBackend.resolve[1].bits.ftqOffset, _newBranchInfo_cfiPosition_fullPosition_T_3)
node newBranchInfo_cfiPosition_position_1 = bits(newBranchInfo_cfiPosition_fullPosition_1, 4, 0)
connect newBranchInfo_1.cfiPosition, newBranchInfo_cfiPosition_position_1
```

即 `cfiPosition = bits(add(ftqOffset, _T_3), 4, 0)`——一个加法再截位，不是常量。
AST2Graph 解析后，数组引用 `curPerfMeta_1.cfiAttr[...]` 的索引子节点是普通表达式
树，不是 `OP_INDEX_INT`，`getIdx()` 无法解析。

历史注记：最初拉取并被该断言挡住的新 FIR 记录为 1.46GB、sha256 前缀
`5f97158e0064aec5`、2007 个 .sv（见 `champions/newrtl-*/registry.json` 的
`rtl` 字段与 candidates.jsonl `new-rtl-pull-blocker` 条目）；当前盘上 build-sv
的 FIR 是其后重新生成的（sha 前缀 `4a0f4533`），触发构造逐字仍存在，上文引用的
行号即来自当前盘文件。

## 2. 根因

三层原因叠加：

1. **索引解析只认常量。** `ENode::getIdx()`（`src/ENode.cpp:309`）遍历左值的索引
   子节点，遇到任何一个非 `OP_INDEX_INT` 就整体放弃：

   ```cpp
   for (ENode* indexENode : child) {
     if (indexENode->opType != OP_INDEX_INT) return std::make_pair(-1, -1);
     index.push_back(indexENode->values[0]);
   }
   ```

   返回 `(-1,-1)` 是"索引动态"的唯一信号。

2. **`distributeTree` 把 `(-1,-1)` 当非法值。** 上游 `origin/master:src/splitArray.cpp`
   的 `distributeTree`（`git show origin/master:src/splitArray.cpp` 可验证）在取
   `getIdx` 后直接：

   ```cpp
   Assert(begin >= 0 && end >= begin, "Invalid index for array %s: %d-%d", node->name.c_str(), begin, end);
   ```

   无任何动态索引分支。本分支修复前的代码相同；当时崩溃点即
   `new-rtl-pull-blocker` 记录的 `splitArray.cpp:216 distributeTree assertion`，
   数组 `curPerfMeta_1__DOT__cfiAttr__DOT__branchType`（FTQ 性能元数据）。

3. **架构缺口是不对称的：读有支持，写没有。** `ExpTree::updateWithSplittedArray`
   （`src/splitArray.cpp`）对数组**读**已有动态索引处理——`range.first < 0` 时
   沿成员从高到低搭 OP_MUX 链，条件是 `OP_EQ(动态索引, i)`，常量宽度取
   `upperLog2(array->arrayEntryNum())`，把整棵树里的该数组读替换成成员 mux 链。
   而主拆分环（`graph::splitArray` → `splitArrayNode` → `distributeTree`）对动态
   索引**写**没有任何处理路径。

   `graph::checkNodeSplit` 里的 `anyVarIdx`（`beg < 0` 即置位）**只服务于可选拆分
   路径** `splitOptionalArray`（无环、可拒绝拆分的数组）。崩溃发生在主拆分环：
   `splitArray` 本身是 Kahn 拓扑排序的一部分，`getSplitArray`/`point2self` 专门
   挑选指向自身的数组来打断依赖环，**必须拆**，拒绝拆 = 图不可排序（见尝试 1 的
   实测）。因此不能用"检测到动态索引就不拆"来回避——这一点经
   `advisory-audit-checknode-split` 专项审计确认：`distributeTree` 是
   `splitArrayNode` 期间全部赋值树分发（SRC 与 DST 节点）的唯一漏斗，在那里修
   一处即覆盖全部拆分写。

## 3. 为什么简单修法都失败（四种尝试）

这部分是本记录最有价值的部分——每条路为什么走不通都有实测证据。

### 尝试 1：跳过不拆（skip）

想法：`getIdx` 返回 `(-1,-1)` 就不拆这个数组。结果：数组留在 `partialVisited`，
拓扑排序无法收尾。`splitArray` 主环的结构（`src/splitArray.cpp`）决定了这一点：
节点出栈时计数 `times[next]`，全部前驱访问完才入栈并从 `partialVisited` 移除；
栈空而 `partialVisited` 非空时循环调用 `getSplitArray(this)` 拆数组解环，找不到
可拆候选时 `getSplitArray` 打印每个卡住节点的缺失前驱后落入 `Panic()`；主环收尾
还有 `Assert(partialVisited.size() == 0, "partial is not empty!")`。自环数组不拆，
环就不断，二者必触发其一。**结论：主环里的数组拆分是环打断机制，不可跳过。**

### 尝试 2：标记已访问绕过（mark-visited + continue）

想法：把卡住的数组直接标成已访问让循环继续。结果：下一段边处理立即断言失败，
环仍在——FTQ 内的组合逻辑链 `_T_149 → _T_150 → _T_151` 卡在 `partialVisited`。
绕过标记不删除真实的依赖边，只是把"发现环"推迟到下一个断言。**结论：环是真实
存在的图结构，记账技巧删不掉它。**

### 尝试 3：手搭 mux 展开（member[i] = mux(eq(idx,i), rhs, member[i])）

想法（外部建议）：对每个成员 i 发射
`member[i] = mux(idx == i, rhs, member[i])`，false 分支读成员自身表达"未选中
则保持"。结果：mux 展开本身能着火（实测 6 个数组、每个 32 项，原断言不再触发），
但 **false 分支对 member[i] 的引用在依赖图里构成成员自环**，FTQ 的
`_T_149/_T_150/_T_151` 仍卡死在 `partialVisited`——拓扑排序要求拆分后是 DAG。
（`new-rtl-mux-expansion-blocker` 条目；半成品保存在本 worktree 的
`stash@{0}`。）

深层原因：对**寄存器数组**，`connect arr[idx], val` 是时钟沿语义
（`arr[idx]$NEXT = val`）。静态索引写正是走寄存器 `$NEXT`/resetTree 机制
（`splitOptionalArray` 里 `NODE_REG_SRC` 的 `bindReg` 配对）落位的；手搭的组合
自赋值 `member[i] = mux(..., member[i])` 完全绕开了该机制。**结论：展开必须
借道既有的赋值树/when 合并机制，不能手工造含自引用的组合等式。**

### 尝试 4：when-展开 + 常量索引读 else（770bab7 误提交的变体）

想法：正确方向（when-展开，见下节）已验证通过后，"改进"else 分支——不放
EMPTY，而是放该数组在常量 i 处的读 `arr[i]`，指望
`updateWithSplittedArray` 的静态分支把它改写成 member[i]，显式表达
last-connect 的"之前的值"。结果：**生成期 panic 在 `getSplitArray`**——else 里的
数组引用让成员树含对自身的依赖边，主环拓扑排序无法满足（与尝试 3 同一类自依赖，
只是藏在了 else 读里）。

该变体被误提交为 `770bab7` 的最终形态（验证过的二进制是 EMPTY 变体生成的，
提交的却是之后手改的 const-read 变体），后由 `e710a43` 纠正。事故过程见第 7 节。

## 4. 修复方案（e710a43，纠正 770bab7）

位置：`src/splitArray.cpp` 的 `distributeTree`，在
`std::tie(begin, end) = tree->getlval()->getIdx(node);` 之后、原断言之前插入
`begin < 0` 分支。核心逻辑（当前代码）：

```cpp
if (begin < 0) {
  ENode* lval = tree->getlval();
  if (node->dimension.size() == 1 && lval->getChildNum() >= 1 &&
      lval->getChild(0) && lval->getChild(0)->getChildNum() >= 1 &&
      (int)node->arrayEntryNum() <= 4096) {
    ENode* idxExprRaw = lval->getChild(0)->getChild(0); /* raw dynamic index expr */
    int n = (int)node->arrayEntryNum();
    for (int i = 0; i < n; i ++) {
      /* lvalue: dup of array ref with index replaced by constant i */
      ENode* lval_i = lval->dup();
      ENode* constIdx = new ENode(OP_INDEX_INT);
      constIdx->addVal(i);
      lval_i->setChild(0, constIdx);
      /* root: when(eq(idx, i), original_root, EMPTY) */
      ...
      ENode* whenNode = new ENode(OP_WHEN);
      whenNode->addChild(eq);
      whenNode->addChild(tree->getRoot()->dup());
      whenNode->addChild(nullptr);   /* EMPTY else */
      ...
      ExpTree* memberTree = new ExpTree(whenNode, lval_i);
      /* 与成员既有树走 mergeWhenTree 合并，或作为首棵树 push */
    }
    return;
  }
  Assert(0, "Invalid index for array %s: dynamic index with unsupported structure (dims=%d entries=%d)", ...);
}
```

要点：

- **结构限制**：一维、索引子结构完整、项数 ≤ 4096，超限走显式 `Assert(0, ...)`
  而非静默错答。索引常量 `i` 的宽度取 `upperLog2(node->arrayEntryNum())`，与
  `updateWithSplittedArray` 动态读 mux 链里的常量宽度一致，保证 `OP_EQ` 两侧
  宽度对齐。
- **到达 `distributeTree` 的树形**是 `arr[dynIdx] = when(cond, rhs, prev)`——
  AST2Graph 的 last-connect 机制已把 when 的**两个**分支填好（else = 之前的
  值）。这是本方案成立的前提：展开只是在外面再套一层
  `when(dynIdx == i, ·, EMPTY)`，内层原树原样 `dup()`。
- **左值保留数组引用 + 常量索引 i**（`OP_INDEX_INT`）。因此
  `updateWithSplittedArray` 的静态分支
  （`range.first == range.second` → `top_enode->nodePtr = arrayMember[range.first]`）
  会把左值改写成 member[i]，与静态索引写完全同路——这就是它不产生自依赖的原因：
  树里**没有**对成员的读引用。
- **EMPTY else 的两种归宿**：
  1. 成员已有赋值树时，`mergeWhenTree` 检测到新树恰有一个空 when 分支且旧树
     根不是 `OP_WHEN`，走 `fillEmptyWhen` 用旧树根填充（即"整数组默认值已先行
     分发"的常见情形）——与静态条件 connect 的合并完全同一机制；
  2. 作为成员首棵树 push 时保持 EMPTY，发射期语义由
     `ENode::instsWhen`（`src/instsGenerator.cpp`）兜底：`!getChild(2)` 且 then
     分支可用时发射 `lvalue = cond ? trueStr : lvalue`——false 分支就是左值自身，
     即"保持之前的赋值"。
- `e710a43` 相对 `770bab7` 的差异只有一处：else 分支从误提交的常量索引读
  （`elseRead = lval->dup()` + `OP_INDEX_INT(i)`）恢复为 `addChild(nullptr)`。

## 5. 正确性论证

设原语义为：每个时钟沿，若 `dynIdx == i` 则 `member[i]` 取
`when(cond, rhs, prev)` 的值，否则保持。展开后成员 i 的赋值树为
`when(eq(dynIdx,i), when(cond,rhs,prev), EMPTY)`：

1. **选中路径**：`dynIdx == i` 时取内层原树，即原 connect 的完整 last-connect
   语义（含条件与"之前的值"），逐位等于原 RTL 行为。
2. **未选中路径**：EMPTY else 经 `mergeWhenTree`/`fillEmptyWhen` 填成成员既有
   树，或发射为 `lvalue = cond ? trueStr : lvalue`——两种归宿都是"member[i]
   保持原值"。恰是"该成员本轮未被动态索引命中"的要求。
3. **无新增依赖边**：左值是数组引用 + 常量索引（后续改写为 member[i] 引用），
   树内没有任何成员读引用，因此不像尝试 3/4 那样制造自环；拓扑排序在拆分后
   得到 DAG。寄存器数组落在 `$NEXT` 机制上与静态索引写一致（时钟沿语义保真）。
4. **互斥完备**：`i` 遍历 `0..n-1`，`eq(dynIdx,i)` 恰有一个为真（索引宽度
   `bits(·,4,0)` 五位、数组 32 项，值域恰好覆盖），多个 when-展开树经
   `mergeWhenTree` 合并后仍逐成员互斥。
5. **不动静态路径**：`begin >= 0` 的所有原有代码路径逐字节未动；本修复只在
   原本必然断言死亡的输入上激活。旧 RTL FIR 门 22/22 通过是这一点的直接证据。

上述论证的端到端确认是第 6 节的 difftest：任何语义偏差都会在 NEMU 逐指令比对
中暴露。

## 6. 验证证据

全部数字取自候选账本（`gsim-task-saturate-sparse/candidates.jsonl`）与冠军注册表
（`gsim-task-verilator-dual-default4488/champions/*/registry.json`）：

- **旧 RTL 回归**：FIR 门 22/22 PASS——修复不影响静态索引生成
  （`new-rtl-adaptation-complete` 条目 `fir_gate` 字段）。
- **可复现性（字节级）**：用 `e710a43` 重新生成（`GSIM_THREADS=16 MAXMT=800`、
  无 COMPACT）与保全的验证过 plain 模型（`champions/newrtl-v1-baseline`）字节
  匹配，合并文件哈希 `d1c39cb55729af4a`（`newrtl-t16-compact-v1` 条目
  `code_integrity` 字段与 baseline registry 注记）。
- **新 RTL 生成**：727 cpp、sccs=65965（比 v86 的 45163 大 46%），生成墙钟约
  981s。
- **端到端正确性**：CoreMark 2-iteration 跑到 **HIT GOOD TRAP**，trap
  pc=0x80001ca0，**663,758 指令 / 304,246 周期**（IPC 2.18），NEMU difftest
  全程**零失配**；且与 plain 模型 instrCnt/cycleCnt 位级一致
  （`newrtl-t16-compact-v1` correctness 字段）。
- **第二工作负载**：linux.bin 30k 周期 no-diff 基准 3 次重复通过，instrCnt
  86469 跨线程数确定（T16/T32 一致，`linux-30k-crossrtl-bench` 条目）。
- **后续下游使用**：两个新 RTL 冠军在该生成器上注册——
  `newrtl-t16-compact-v1`（linux-30k 8.21s）与 `newrtl-t32-compact-v1`
  （7.01s），种子 write==replay 9/9、哈希一致。
- 配套（非本修复但同次新 RTL 适配）：新 RTL 的 DPI extmodule
  （TopdownIQInfoHelper_80/238、TopdownRobInfoHelper_318_352）需在
  difftest-extmodule.cpp 手写 wrapper。

## 7. 工作流教训：验证过的二进制 vs 提交的代码（770bab7 → e710a43）

事故链：EMPTY-else 变体生成模型 → 全量验证（HIT GOOD TRAP 零失配）→ 验证后
"改进" else 分支为常量索引读 → **未重新生成**直接提交为 `770bab7`。之后
COMPACT 探针生成 panic（const-read else 在 `getSplitArray` 里造成员自依赖），
暴露"提交的代码无法再生验证过的模型"。`e710a43` 恢复验证过的 EMPTY 变体，并
以再生字节匹配（`d1c39cb55729af4a`）作为复现证明闭环。

规则（已沉淀到 mtwiki `wiki-ab-isolation-discipline.md`）：生成器的任何"改进"
必须走完 重新生成 → 字节比对 → 重跑门 之后才能提交；**验证用的二进制与提交
的源码必须是同一版本**。这对上游 PR 尤其重要：PR 里贴的验证数字必须能被 PR
的代码再生。

## 8. 上游修补指引（origin/master 视角）

上游结构与本分支修复前一致（已用 `git show origin/master:src/splitArray.cpp`
核实），可直接移植：

1. **改一处**：`src/splitArray.cpp` 的 `distributeTree`，在
   `std::tie(begin, end) = tree->getlval()->getIdx(node);` 与
   `Assert(begin >= 0 && end >= begin, "Invalid index for array %s: %d-%d", ...)`
   之间插入第 4 节的 `if (begin < 0) { ... }` 分支（含结构限制与兜底
   `Assert(0, ...)`）。
2. **无需改 ENode**：所需原语上游全部已备——`mergeWhenTree`/`fillEmptyWhen`
   （含空 when 分支填充语义）、`updateWithSplittedArray` 的静态分支
   （`range.first == range.second` 时把左值改写为成员引用）与动态读 mux 链
   （常量宽度约定 `upperLog2`）、发射期 `ENode::instsWhen` 对 `!getChild(2)`
   的 lvalue 兜底。
3. **不要**用 `hasVarIdx()`（`src/ENode.cpp:329`，上游定义但零调用）做拒绝式
   守卫：主拆分环里拒绝拆分会死在拓扑排序（第 3 节尝试 1）；
   `checkNodeSplit`/`nextVarConnect` 保持不动。
4. **不要**手搭 `member[i] = mux(..., member[i])` 或在 else 里放数组常量读
   （第 3 节尝试 3/4）：两者都以不同面目制造成员自依赖。
5. 若上游想覆盖多维/超大数组，需另行设计（当前显式 `Assert(0, ...)` 拒绝，
   dims>1 时上游 `updateWithSplittedArray` 动态读也有 `TODO()` 占位，可顺带
   对齐）。kunminghu-v3 实测触发面是一维 32 项数组，4096 上限余量充足。
6. PR 验证建议照搬第 6 节口径：旧 RTL FIR 门 + 再生字节匹配 + 新 RTL
  CoreMark 全量 difftest（HIT GOOD TRAP、零失配）。

## 9. 参考索引

- 修复提交：`e710a43`（splitArray: restore validated EMPTY-else in
  when-expansion，纠正 770bab7）；`770bab7`（splitArray: when-expansion for
  dynamic-index array writes）。
- 本分支代码：`src/splitArray.cpp` 的 `distributeTree`（when-展开分支）、
  `ExpTree::updateWithSplittedArray`（动态读 mux 链 + 静态改写分支）、
  `graph::splitArray` 主环（`partialVisited`/`getSplitArray`/`Panic()`）、
  `graph::checkNodeSplit`/`nextVarConnect`/`splitOptionalArray`、
  `mergeWhenTree`/`fillEmptyWhen`；`src/ENode.cpp` 的 `getIdx`（:309）与
  `hasVarIdx`（:329）；`src/instsGenerator.cpp` 的 `ENode::instsWhen`
  （EMPTY else 发射兜底）。
- 上游对照：`origin/master:src/splitArray.cpp` `distributeTree` 的无守卫断言；
  `origin/master:src/ENode.cpp:329` `hasVarIdx`（定义、零调用，git grep 可核）。
- 触发材料：`/home/zhangyangjie/test/XiangShan/build-sv/rtl/SimTop.fir`（当前盘，
  触发 connect 在第 2124270 行）；`/home/zhangyangjie/test/rtl-v86-frozen/SimTop.fir`
  （旧 RTL 冻结，sha 前缀 `04933259`，curPerfMeta 零引用）。首次拉取的新 FIR
  记录：1.46GB、sha 前缀 `5f97158e0064aec5`、2007 .sv、领先 v86 253 提交。
- 账本与注册表：`gsim-task-saturate-sparse/candidates.jsonl` 条目
  `new-rtl-pull-blocker` / `new-rtl-mux-expansion-blocker` /
  `new-rtl-adaptation-complete` / `advisory-audit-checknode-split` /
  `linux-30k-crossrtl-bench` / `newrtl-t16-compact-v1`；
  `gsim-task-verilator-dual-default4488/champions/` 下 `newrtl-v1-baseline`、
  `newrtl-t16-compact-v1`、`newrtl-t32-compact-v1` 的 registry.json。
- 过程记录：mtwiki `wiki-generator-speed.md` 追记补 4（三次失败尝试）与补 5
  （when-展开完成）；`wiki-ab-isolation-discipline.md`（验证后改码直接提交的
  陷阱）。
