// Unit test for include/mtRepcutForm.h (sink-IoU task forming).
//
// Build & run after `make build-gsim` is allowed (measurement windows are
// exclusive): g++ -O2 -std=c++17 -I include -o /tmp/test_repcut_form \
//             test/test_repcut_form.cc && /tmp/test_repcut_form
//
// The randomized stress section is the load-bearing guard: every merge the
// former admits must keep the contracted cluster graph acyclic, which is
// checked by re-verifying forwardness over the produced cluster order. The
// hand-built "straddle" graph exercises the exact merge-rejection proof
// (a -> c1 enters a third cluster, c1 -> s returns from it).

#include "mtRepcutForm.h"

#include <algorithm>
#include <cstdio>
#include <random>
#include <set>
#include <vector>

using mtrepcut::Former;
using mtrepcut::Options;

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    ++failures;
    std::fprintf(stderr, "FAIL: %s\n", what);
  }
}

// Independent verification: full coverage, unique assignment, and every
// bucket edge forward across the produced cluster order.
void verifyResult(const Former& former, int n,
                  const std::vector<std::vector<int>>& successors) {
  const auto& clusters = former.orderedClusters();
  std::vector<int> position(static_cast<size_t>(n), -1);
  int seen = 0;
  for (size_t c = 0; c < clusters.size(); ++c) {
    for (int bucket : clusters[c]) {
      check(bucket >= 0 && bucket < n, "bucket id in range");
      check(position[static_cast<size_t>(bucket)] < 0, "bucket assigned once");
      position[static_cast<size_t>(bucket)] = static_cast<int>(c);
      ++seen;
    }
  }
  check(seen == n, "full coverage");
  if (seen != n) return;
  for (int from = 0; from < n; ++from) {
    for (int to : successors[static_cast<size_t>(from)]) {
      check(position[static_cast<size_t>(from)] <= position[static_cast<size_t>(to)],
            "edge stays forward across clusters");
    }
  }
  const auto& clusterOf = former.clusterOfBucket();
  for (int b = 0; b < n; ++b) {
    const auto& members = clusters[static_cast<size_t>(position[static_cast<size_t>(b)])];
    check(std::find(members.begin(), members.end(), clusterOf[static_cast<size_t>(b)]) !=
              members.end(),
          "clusterOf matches an ordered cluster member");
  }
}

void testBasics() {
  // Chain a->b->c: direct-edge merges are legal (the edge internalizes).
  {
    const std::vector<std::vector<int>> succ = {{1}, {2}, {}};
    const std::vector<long long> cost = {1, 1, 1};
    const std::vector<char> pinned(3, 0);
    const std::vector<char> sink = {0, 0, 1};
    Options options;
    options.threshold = 3;
    options.stopCount = 1;
    Former former(3, succ, cost, pinned, sink, options);
    former.run();
    check(former.valid(), "chain valid");
    verifyResult(former, 3, succ);
    check(former.stats().clusters == 1, "chain merges into one cluster");
  }
  // Pinned buckets stay singletons even with perfect sink overlap.
  {
    const std::vector<std::vector<int>> succ = {{2}, {2}, {}};
    const std::vector<long long> cost = {1, 1, 1};
    const std::vector<char> pinned = {0, 1, 0};
    const std::vector<char> sink = {0, 0, 1};
    Options options;
    options.threshold = 10;
    options.stopCount = 1;
    Former former(3, succ, cost, pinned, sink, options);
    former.run();
    const auto& clusterOf = former.clusterOfBucket();
    check(clusterOf[1] == 1, "pinned bucket stays singleton");
  }
  // Threshold: two identical-sink siblings whose union exceeds the threshold
  // must stay separate.
  {
    const std::vector<std::vector<int>> succ = {{2}, {2}, {}};
    const std::vector<long long> cost = {7, 7, 1};
    const std::vector<char> pinned(3, 0);
    const std::vector<char> sink = {0, 0, 1};
    Options options;
    options.threshold = 10;
    options.stopCount = 1;
    Former former(3, succ, cost, pinned, sink, options);
    former.run();
    const auto& clusterOf = former.clusterOfBucket();
    check(clusterOf[0] != clusterOf[1], "threshold blocks union");
  }
  // Invalid input (backward edge) is rejected up front.
  {
    const std::vector<std::vector<int>> succ = {{}, {0}};
    const std::vector<long long> cost = {1, 1};
    const std::vector<char> pinned(2, 0);
    const std::vector<char> sink = {1, 0};
    Options options;
    Former former(2, succ, cost, pinned, sink, options);
    check(!former.valid(), "backward edge rejected");
  }
  // Empty graph is a valid no-op.
  {
    Former former(0, {}, {}, {}, {}, Options{});
    former.run();
    check(former.valid() && former.orderedClusters().empty(), "empty graph no-op");
  }
}

// 13-bucket graph forcing the exact split-straddle rejection:
//   a(0) feeds c1(1) and t*(9..12); c1 -> s*(4..8); c2(2) -> b(3) and s*;
//   b -> t*. Costs make (c1,c2) the only affordable top pair, so C={c1,c2}
//   forms first. Then (a,s_i) has IoU>0 and fits the threshold, but merging
//   it must be rejected: a->c1 enters C and c1->s_i returns from C, which
//   would cycle the contracted graph. An unsound merge shows up as
//   valid()==false (Kahn fails) or a backward edge in verifyResult.
struct StraddleGraph {
  static constexpr int kN = 13;
  std::vector<std::vector<int>> succ;
  const std::vector<long long> cost{100, 50, 50, 100, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  const std::vector<char> pinned = std::vector<char>(kN, 0);
  const std::vector<char> sink{0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1};

  StraddleGraph() : succ(kN) {
    succ[0] = {1, 9, 10, 11, 12};  // a -> c1, a -> t*
    succ[1] = {4, 5, 6, 7, 8};     // c1 -> s*
    succ[2] = {3, 4, 5, 6, 7, 8};  // c2 -> b, c2 -> s*
    succ[3] = {9, 10, 11, 12};     // b -> t*
  }
};

void testStraddleScenario(int stopCount, int expectedClusters, const char* label) {
  const StraddleGraph graph;
  Options options;
  options.threshold = 105;
  options.stopCount = stopCount;
  Former former(StraddleGraph::kN, graph.succ, graph.cost, graph.pinned,
                graph.sink, options);
  former.run();
  check(former.valid(), label);
  if (!former.valid()) return;
  verifyResult(former, StraddleGraph::kN, graph.succ);
  check(former.stats().clusters == expectedClusters, label);
  const auto& clusterOf = former.clusterOfBucket();
  check(clusterOf[1] == clusterOf[2], "c1/c2 cluster forms first");
  check(clusterOf[0] != clusterOf[1], "a never joins the c1/c2 cluster");
  check(clusterOf[0] != clusterOf[4] && clusterOf[0] != clusterOf[8],
        "a/s* straddle merge rejected");
  check(clusterOf[3] == clusterOf[9], "b absorbs t sinks");
  std::fprintf(stderr, "%s: clusters=%d merges=%d rounds=%d stalled=%d\n", label,
               former.stats().clusters, former.stats().merges,
               former.stats().rounds, former.stats().stoppedEarly);
}

void testRandomized() {
  std::mt19937 rng(20261022u);
  int graphs = 0;
  for (int iter = 0; iter < 400; ++iter) {
    const int n = 2 + static_cast<int>(rng() % 60);
    std::vector<std::vector<int>> succ(static_cast<size_t>(n));
    std::vector<long long> cost(static_cast<size_t>(n), 1);
    std::vector<char> pinned(static_cast<size_t>(n), 0);
    std::vector<char> sink(static_cast<size_t>(n), 1);
    for (int b = 0; b < n; ++b) {
      const int outDegree = static_cast<int>(rng() % 4);
      std::set<int> targets;
      for (int e = 0; e < outDegree; ++e) {
        if (b + 1 >= n) break;
        targets.insert(b + 1 + static_cast<int>(rng() % static_cast<unsigned>(n - b - 1)));
      }
      succ[static_cast<size_t>(b)].assign(targets.begin(), targets.end());
      if (!succ[static_cast<size_t>(b)].empty()) sink[static_cast<size_t>(b)] = 0;
      cost[static_cast<size_t>(b)] = 1 + static_cast<long long>(rng() % 30);
      if (rng() % 13 == 0) pinned[static_cast<size_t>(b)] = 1;
    }
    Options options;
    options.threshold = 1 + static_cast<long long>(rng() % 200);
    options.stopCount = 1 + static_cast<int>(rng() % static_cast<unsigned>(n));
    Former former(n, succ, cost, pinned, sink, options);
    former.run();
    check(former.valid(), "random graph run stays valid");
    if (!former.valid()) continue;
    verifyResult(former, n, succ);
    // Determinism: identical rerun must produce identical clustering.
    Former again(n, succ, cost, pinned, sink, options);
    again.run();
    check(former.clusterOfBucket() == again.clusterOfBucket(), "deterministic rerun");
    check(former.orderedClusters() == again.orderedClusters(), "deterministic order");
    // Stop discipline: never below the stop count, never merges pinned.
    check(former.stats().clusters >= options.stopCount ||
              former.stats().stoppedEarly > 0,
          "stop count floor respected");
    const auto& clusterOf = former.clusterOfBucket();
    for (int b = 0; b < n; ++b) {
      if (!pinned[static_cast<size_t>(b)]) continue;
      int members = 0;
      for (int other = 0; other < n; ++other) {
        if (clusterOf[static_cast<size_t>(other)] == clusterOf[static_cast<size_t>(b)]) ++members;
      }
      check(members == 1, "pinned stays singleton (random)");
    }
    ++graphs;
  }
  std::fprintf(stderr, "randomized: %d graphs verified\n", graphs);
}

}  // namespace

int main() {
  testBasics();
  testStraddleScenario(10, 10, "straddle-stop10");
  testStraddleScenario(9, 9, "straddle-stop9");
  testRandomized();
  if (failures == 0) {
    std::fprintf(stderr, "test_repcut_form: ALL PASS\n");
    return 0;
  }
  std::fprintf(stderr, "test_repcut_form: %d FAILURES\n", failures);
  return 1;
}
