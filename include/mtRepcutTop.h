#ifndef MT_REPCUT_TOP_H
#define MT_REPCUT_TOP_H

#include <map>
#include <string>
#include <vector>

class graph;
struct MtTaskPlan;

// BubbleTool attribution-driven targeted task replication (micro RepCut).
//
// Input: a bubble profile ranking ready-token slots by spin time
// (bubble-top-wait-slots.csv: slot_id,producer_task,producer_worker,
// spinner_workers,total_ns,share_of_spin_pct,cross_ccd). For the top-N rows
// the publisher task P of the hot slot is replicated into the spinning
// consumer worker's chain: every consumer C of P on that worker stops
// waiting for P's ready token and reads a local re-computation instead.
//
// The transform runs between HEFT placement and token-table construction, so
// every table (dense owner-ready slots, localWait positions, lookahead
// admission, dispatch chains) is regenerated from the transformed graph —
// no generated table is patched afterwards.
//
// Knobs (generate time, all default-off):
//   GSIM_MT_REPCUT_TOP=<N>   replicate the top-N attribution slots (0=off)
//   GSIM_MT_REPCUT_CSV=<p>   override the default CSV path
//   GSIM_MT_REPCUT_DEPTH=<d> 1 additionally replicates same-worker input
//                            producers one level deep (escalation tier;
//                            the first shot runs with 0)
//
// Replication is bounded by a static cost cap: the replicated static cost
// must stay within one third of the expected spin savings of the admitted
// slots, converted into cost units with the profile's own compute/cost
// calibration (bubble-worker-summary.csv next to the CSV). Without
// calibration the cap falls back to one third of the RepCut 3.81% prior
// share of the total task cost.
namespace mtrepcut {

struct SlotRow {
  int rank = 0;
  int slotId = -1;
  int producerTask = -1;
  int producerWorker = -1;
  int spinnerWorker = -1;
  long long totalNs = 0;
  double sharePct = 0.0;
  int crossCcd = 0;
};

struct Report {
  bool active = false;
  std::string csvPath;
  int csvRows = 0;
  int topN = 0;
  int considered = 0;        // rows whose producer task id is in range
  int replicatedRows = 0;    // rows covered by a created replica
  int replicas = 0;          // replica tasks appended
  int firstReplicaId = -1;   // task id of the first appended replica
  int replicatedProducers = 0;
  int rewiredConsumers = 0;  // consumer edges P->C replaced by rep->C
  int killedGroups = 0;      // consumer groups whose last Wp source was P
  int residualGroups = 0;    // consumer groups keeping a shorter Wp token
  int depthReplicas = 0;     // replicas added by the depth-1 tier
  long long replicaCost = 0; // summed replica static cost
  long long totalCost = 0;   // summed original task static cost
  double costPct = 0.0;      // replicaCost / totalCost * 100
  long long expectedSavingNs = 0;  // admitted rows' total_ns
  long long budgetCost = 0;        // last evaluated cost cap, static units
  bool priorFallbackCap = false;   // profile calibration was unavailable
  std::map<std::string, int> rejected;
  std::vector<std::string> decisions;  // one line per evaluated row
};

// True when GSIM_MT_REPCUT_TOP is present with a positive value; callers
// must stay byte-identical otherwise.
bool repcutTopEnabled();

// Applies the transform to the scheduled plan. owners and chains are the
// HEFT placement outputs and are mutated in place (replicas appended into
// the consumer chains). Returns the report; when the knob is off the call
// touches nothing and report.active is false.
Report applyRepcutTop(graph& g, MtTaskPlan& plan, std::vector<int>& owners,
                      std::vector<std::vector<int>>& chains, int workerCount);

}  // namespace mtrepcut

#endif
