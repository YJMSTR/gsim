#ifndef MT_NODE_SINK_H
#define MT_NODE_SINK_H

#include "mtTaskPartition.h"

#include <vector>

class graph;

// Result of node-level sink planning. Task IDs index plan.tasks_ after the
// planner replaced the MTask formation; chains are the global topological
// order filtered per worker, so every same-worker dependency is forward.
struct MtNodeSinkSchedule {
  std::vector<int> owner;
  std::vector<std::vector<int>> chains;
  long long makespan = 0;
};

// Node-level sink planner (GSIM_MT_NODE_SINK=1). Forms one microtask per
// surviving graph node (side-effect nodes are unique and never replicated),
// lowers each node independently after the shared MT lowering, assigns the
// microtasks with a deterministic sink-aware LPT pass and replicates pure
// combinational fanout nodes under a strict cost/budget cap. The plan keeps
// the shared state-update and ready-token machinery untouched.
class MtNodeSinkPlanner {
 public:
  static bool enabled();

  // Replaces plan.tasks_/plan.taskByNode_ with the node-level plan (plus
  // committed clones) and returns the worker schedule for it.
  static void plan(graph& graph, MtTaskPlan& plan, int workerCount,
                   MtNodeSinkSchedule& schedule);
};

#endif
