#ifndef MT_TASK_LOWERING_H
#define MT_TASK_LOWERING_H

#include "mtTaskPartition.h"

class graph;

/* Post-scheduling uniform-fill run merge (GSIM_MT_UNIFORM_FILL, default
   off). Rewrites qualifying consecutive element-store runs in place. */
void mtMergeUniformFillRuns(std::vector<InstInfo>& insts);

class MtTaskLowerer {
 public:
  static void generateStmtTrees(graph& graph, MtTaskPlan& plan);
};

#endif
