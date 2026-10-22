#ifndef MT_REPCUT_FORM_H
#define MT_REPCUT_FORM_H

// Sink-IoU task forming (repcut port, GSIM_MT_REPCUT_FORM).
//
// The baseline former packs topologically consecutive group buckets into tasks
// until a per-task cost budget is exhausted. This pass replaces that packing
// when the knob is on: it first packs the same groups into finer seed buckets
// (same mechanics, smaller budget), then repeatedly merges bucket clusters
// whose reachable sink sets overlap (cost-weighted Jaccard IoU — each shared
// sink counts by its structural cost, the wolvrix hyperedge-weight analog),
// so groups feeding the same state sinks (registers / memory writers) land in
// the same task and the task graph keeps fewer cross-task edges for HEFT to
// place across workers.
//
// Correctness contract: the produced cluster order is a topological order of
// the cluster DAG contracted from the input bucket DAG, so every inter-task
// bucket edge stays forward. A merge of clusters A,B is admitted only when it
// cannot create a cycle in the contracted graph, i.e. no third cluster C has
// both a path from A and a path to B (or the mirrored direction). That check
// is exact: quotient paths A~>C and C~>B exist iff some bucket of C is in
// fwd(A) and some (possibly different) bucket of C is in bwd(B); the
// single-bucket witness (fwd(A) & bwd(B)) is only the special case c1 == c2.
//
// The whole pass is deterministic: integer IoU compared by cross
// multiplication, fixed scan caps, total tie-break orderings, no hashing, no
// floating point, no time in decisions. It depends only on the C++ standard
// library so it can be unit-tested without the gsim graph.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <utility>
#include <vector>

namespace mtrepcut {

struct Options {
  // Maximum union structural cost of a merged cluster (p2sim's union-size
  // threshold analog). Merges whose combined cost exceeds it are rejected.
  long long threshold = 1;
  // Stop merging once the alive cluster count reaches this value.
  int stopCount = 1;
};

struct Stats {
  int buckets = 0;
  int sinks = 0;
  int clusters = 0;
  int merges = 0;
  int rounds = 0;
  int stoppedEarly = 0;  // alive clusters above stopCount when rounds stalled
  long long crossEdgesBefore = 0;
  long long crossEdgesAfter = 0;
  long long sinkOverlapSum = 0;   // sum over final task edges of |sink(A)&sink(B)|
  long long sinkOverlapEdges = 0;
  long long passMs = 0;
};

class Former {
 public:
  // `successors[i]` lists the buckets i has DAG edges to. Bucket ids must be
  // topologically ordered: every edge goes i -> j with i < j. Duplicate and
  // self edges are dropped. `cost` >= 0 per bucket. `pinned` buckets never
  // merge (task kinds that must stay singletons). `sink` marks sink buckets.
  Former(int bucketCount, const std::vector<std::vector<int>>& successors,
         const std::vector<long long>& cost, const std::vector<char>& pinned,
         const std::vector<char>& sink, Options options)
      : n_(bucketCount),
        succ_(bucketCount),
        pred_(bucketCount),
        sink_(sink),
        opt_(options),
        parent_(static_cast<size_t>(bucketCount)),
        ccost_(cost.begin(), cost.end()),
        cpinned_(pinned.begin(), pinned.end()),
        cnt_(static_cast<size_t>(bucketCount), 0),
        cntStamp_(static_cast<size_t>(bucketCount), -1),
        clusterOf_(static_cast<size_t>(bucketCount)) {
    valid_ = bucketCount >= 0 && cost.size() == static_cast<size_t>(bucketCount) &&
             pinned.size() == static_cast<size_t>(bucketCount) &&
             sink.size() == static_cast<size_t>(bucketCount) &&
             successors.size() == static_cast<size_t>(bucketCount);
    if (!valid_) return;
    for (int from = 0; from < n_; ++from) {
      std::vector<int> list = successors[static_cast<size_t>(from)];
      std::sort(list.begin(), list.end());
      list.erase(std::unique(list.begin(), list.end()), list.end());
      for (int to : list) {
        if (to <= from || to >= n_) {
          valid_ = false;
          return;
        }
        succ_[static_cast<size_t>(from)].push_back(to);
        pred_[static_cast<size_t>(to)].push_back(from);
      }
    }
    for (size_t i = 0; i < parent_.size(); ++i) parent_[i] = static_cast<int>(i);
    for (size_t i = 0; i < clusterOf_.size(); ++i) clusterOf_[i] = static_cast<int>(i);
  }

  bool valid() const { return valid_; }
  const Stats& stats() const { return stats_; }

  void run() {
    if (!valid_ || n_ == 0) return;
    const auto start = std::chrono::steady_clock::now();
    buildClosures();
    buildColumns();
    weightColumns();
    stats_.buckets = n_;
    stats_.sinks = countSinks();
    alive_ = n_;
    mergeRounds();
    finishOrder();
    measureEdges();
    stats_.clusters = alive_;
    stats_.passMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - start)
                        .count();
  }

  // Canonical cluster id per bucket (min member bucket id).
  const std::vector<int>& clusterOfBucket() const { return clusterOf_; }

  // Clusters in topological (Kahn, min-id tie-break) order, members ascending.
  const std::vector<std::vector<int>>& orderedClusters() const { return ordered_; }
  // Weighted sink columns (grhsim-repcut essence absorption): IoU scores are
  // cost-weighted by each sink bucket's structural cost — the analog of
  // wolvrix's hyperedge weight — so merges prioritize fat shared sinks over
  // trivial ones instead of counting sink buckets equally.
  using Words = std::vector<uint64_t>;

  void weightedIoU(const Words& mine, const Words& theirs, long long* intersection,
                   long long* unionWeight) const {
    long long inter = 0, uni = 0;
    for (size_t i = 0; i < mine.size(); ++i) {
      uint64_t bits = mine[i] & theirs[i];
      while (bits != 0) {
        const int bit = static_cast<int>(__builtin_ctzll(bits));
        bits &= bits - 1;
        inter += columnWeight_[static_cast<size_t>(i * 64 + static_cast<size_t>(bit))];
      }
      bits = mine[i] | theirs[i];
      while (bits != 0) {
        const int bit = static_cast<int>(__builtin_ctzll(bits));
        bits &= bits - 1;
        uni += columnWeight_[static_cast<size_t>(i * 64 + static_cast<size_t>(bit))];
      }
    }
    *intersection = inter;
    *unionWeight = uni;
  }

 private:


  int n_;
  std::vector<std::vector<int>> succ_, pred_;
  std::vector<char> sink_;
  Options opt_;
  Stats stats_;
  bool valid_ = false;

  std::vector<int> parent_;
  std::vector<long long> ccost_;
  std::vector<long long> columnWeight_;
  std::vector<char> cpinned_;
  int alive_ = 0;
  std::vector<long long> cnt_;
  std::vector<int> cntStamp_;
  int words_ = 0;
  Words sinkMaskWords_;
  std::vector<Words> fwd_, bwd_, sinkRow_, memRow_;

  // Capped inverted index: for sink bucket s, the first kColumnCap buckets
  // whose sink row contains s, ascending (candidate partner generation only).
  std::vector<std::vector<int32_t>> column_;
  int cntStampValue_ = 0;

  std::vector<int> clusterOf_;
  std::vector<std::vector<int>> ordered_;

  static constexpr size_t kColumnCap = 8;
  static constexpr size_t kColumnScanCap = 1024;
  static constexpr size_t kPartnerShortlist = 24;
  static constexpr int kMaxRounds = 64;

  int find(int x) {
    while (parent_[static_cast<size_t>(x)] != x) {
      parent_[static_cast<size_t>(x)] = parent_[static_cast<size_t>(parent_[static_cast<size_t>(x)])];
      x = parent_[static_cast<size_t>(x)];
    }
    return x;
  }

  void buildClosures() {
    words_ = static_cast<int>((static_cast<size_t>(n_) + 63) / 64);
    const size_t w = static_cast<size_t>(words_);
    const Words zero(w, 0);
    sinkMaskWords_.assign(w, 0);
    for (int b = 0; b < n_; ++b) {
      if (sink_[static_cast<size_t>(b)]) {
        sinkMaskWords_[static_cast<size_t>(b) >> 6] |= 1ull << (static_cast<uint64_t>(b) & 63);
      }
    }
    fwd_.assign(static_cast<size_t>(n_), zero);
    bwd_.assign(static_cast<size_t>(n_), zero);
    sinkRow_.assign(static_cast<size_t>(n_), zero);
    memRow_.assign(static_cast<size_t>(n_), zero);
    for (int b = n_ - 1; b >= 0; --b) {
      Words& row = fwd_[static_cast<size_t>(b)];
      row[static_cast<size_t>(b) >> 6] |= 1ull << (static_cast<uint64_t>(b) & 63);
      for (int to : succ_[static_cast<size_t>(b)]) {
        const Words& succRow = fwd_[static_cast<size_t>(to)];
        for (size_t i = 0; i < w; ++i) row[i] |= succRow[i];
      }
    }
    for (int b = 0; b < n_; ++b) {
      Words& row = bwd_[static_cast<size_t>(b)];
      row[static_cast<size_t>(b) >> 6] |= 1ull << (static_cast<uint64_t>(b) & 63);
      for (int from : pred_[static_cast<size_t>(b)]) {
        const Words& predRow = bwd_[static_cast<size_t>(from)];
        for (size_t i = 0; i < w; ++i) row[i] |= predRow[i];
      }
    }
    for (int b = 0; b < n_; ++b) {
      Words& row = sinkRow_[static_cast<size_t>(b)];
      for (size_t i = 0; i < w; ++i) row[i] = fwd_[static_cast<size_t>(b)][i] & sinkMaskWords_[i];
      memRow_[static_cast<size_t>(b)][static_cast<size_t>(b) >> 6] |=
          1ull << (static_cast<uint64_t>(b) & 63);
    }
  }

  void buildColumns() {
    column_.assign(static_cast<size_t>(n_), {});
    for (int b = 0; b < n_; ++b) {
      forEachSetBit(sinkRow_[static_cast<size_t>(b)], [&](int s) {
        std::vector<int32_t>& list = column_[static_cast<size_t>(s)];
        if (list.size() < kColumnCap) list.push_back(static_cast<int32_t>(b));
      });
    }
  }

  // Column weight = sink bucket cost x fanoutScale (playbook D4/D5
  // absorption): a sink fed by k producers saves k-way dispatch/tokens when
  // its producers land in one task, so the merge score scales its weight by
  // 1 + 0.12*floorLog2(1 + fanout), wolvrix's fanoutScale curve, in integer
  // percent. Exact fanout needs one full sink-row pass (no list cap).
  void weightColumns() {
    columnWeight_.assign(static_cast<size_t>(n_), 0);
    std::vector<long long> fanout(static_cast<size_t>(n_), 0);
    for (int b = 0; b < n_; ++b) {
      const Words& row = sinkRow_[static_cast<size_t>(b)];
      for (size_t i = 0; i < row.size(); ++i) {
        uint64_t bits = row[i];
        while (bits != 0) {
          const int bit = static_cast<int>(__builtin_ctzll(bits));
          bits &= bits - 1;
          ++fanout[static_cast<size_t>(i * 64 + static_cast<size_t>(bit))];
        }
      }
    }
    for (int s = 0; s < n_; ++s) {
      if (!sink_[static_cast<size_t>(s)]) continue;
      long long scale = 100 + 12 * floorLog2(1 + fanout[static_cast<size_t>(s)]);
      columnWeight_[static_cast<size_t>(s)] =
          std::max<long long>(1, ccost_[static_cast<size_t>(s)] * scale / 100);
    }
  }

  static int floorLog2(long long value) {
    int result = 0;
    while (value > 1) {
      value >>= 1;
      ++result;
    }
    return result;
  }


  int countSinks() const {
    int total = 0;
    for (int b = 0; b < n_; ++b) total += sink_[static_cast<size_t>(b)] ? 1 : 0;
    return total;
  }

  template <typename Fn>
  static void forEachSetBit(const Words& row, Fn&& fn) {
    for (size_t i = 0; i < row.size(); ++i) {
      uint64_t bits = row[i];
      while (bits != 0) {
        const int bit = static_cast<int>(__builtin_ctzll(bits));
        bits &= bits - 1;
        fn(static_cast<int>(i * 64 + static_cast<size_t>(bit)));
      }
    }
  }

  static long long popcount(const Words& row) {
    long long total = 0;
    for (uint64_t word : row) total += __builtin_popcountll(word);
    return total;
  }

  // True when merging clusters a and b would create a cycle in the contracted
  // graph: unsafe iff some third cluster C lies on a quotient path between
  // them, i.e. (a reaches C and C reaches b) or mirrored. fwd_/bwd_ hold the
  // exact transitive quotient closure over cluster ids (see mergeInto).
  bool mergeCreatesCycle(int a, int b) {
    const size_t w = static_cast<size_t>(words_);
    const Words &fa = fwd_[static_cast<size_t>(a)], &fb = fwd_[static_cast<size_t>(b)];
    const Words &ba = bwd_[static_cast<size_t>(a)], &bb = bwd_[static_cast<size_t>(b)];
    // A witness C only counts when it is outside both clusters: memRow_
    // holds each cluster's member ids, and any a~>b quotient path whose
    // intermediates all lie inside members(a) | members(b) is internalized
    // by the merge itself. Paths of length >= 2 with an external
    // intermediate always show one in fwd_[a] & bwd_[b].
    const Words &ma = memRow_[static_cast<size_t>(a)], &mb = memRow_[static_cast<size_t>(b)];
    for (size_t i = 0; i < w; ++i) {
      const uint64_t members = ~(ma[i] | mb[i]);
      if ((fa[i] & bb[i] & members) != 0) return true;
      if ((fb[i] & ba[i] & members) != 0) return true;
    }
    return false;
  }



  // Best merge partner for cluster a by shared-sink counting, exact IoU on a
  // counted shortlist. Returns -1 when no candidate exists.
  int bestPartner(int a) {
    const Words& row = sinkRow_[static_cast<size_t>(a)];
    ++cntStampValue_;
    std::vector<int> candidates;
    const long long columns = popcount(row);
    long long stride = 1;
    if (columns > static_cast<long long>(kColumnScanCap)) {
      stride = columns / static_cast<long long>(kColumnScanCap);
    }
    long long seen = 0;
    bool done = false;
    for (size_t i = 0; i < row.size() && !done; ++i) {
      uint64_t bits = row[i];
      while (bits != 0) {
        const int bit = static_cast<int>(__builtin_ctzll(bits));
        bits &= bits - 1;
        if (seen++ % stride == 0) {
          const std::vector<int32_t>& list = column_[static_cast<size_t>(i * 64 + static_cast<size_t>(bit))];
          for (int32_t entry : list) {
            const int other = find(entry);
            if (other == a || cpinned_[static_cast<size_t>(other)]) continue;
            if (cntStamp_[static_cast<size_t>(other)] != cntStampValue_) {
              cntStamp_[static_cast<size_t>(other)] = cntStampValue_;
              cnt_[static_cast<size_t>(other)] = 0;
              candidates.push_back(other);
            }
            ++cnt_[static_cast<size_t>(other)];
          }
        }
      }
    }
    if (candidates.empty()) return -1;
    std::sort(candidates.begin(), candidates.end(), [&](int l, int r) {
      if (cnt_[static_cast<size_t>(l)] != cnt_[static_cast<size_t>(r)]) {
        return cnt_[static_cast<size_t>(l)] > cnt_[static_cast<size_t>(r)];
      }
      return l < r;
    });
    const size_t shortlist = std::min(candidates.size(), kPartnerShortlist);
    int best = -1;
    long long bestNum = 0, bestDen = 1, bestUnion = 0;
    for (size_t i = 0; i < shortlist; ++i) {
      const int other = candidates[i];
      long long inter = 0, uni = 0;
      weightedIoU(sinkRow_[static_cast<size_t>(a)],
                  sinkRow_[static_cast<size_t>(other)], &inter, &uni);
      const long long unionCost =
          ccost_[static_cast<size_t>(a)] + ccost_[static_cast<size_t>(other)];
      const bool better = best < 0 || bestNum * uni < inter * bestDen ||
                          (bestNum * uni == inter * bestDen && unionCost < bestUnion);
      if (better) {
        best = other;
        bestNum = inter;
        bestDen = uni == 0 ? 1 : uni;
        bestUnion = unionCost;
      }
    }
    return best;
  }

  // Merge b into a (a < b). fwd_/bwd_ are the transitive quotient closure
  // over cluster ids. Merging changes reachability for every cluster that
  // reaches either side (they now reach everything the union reaches) and,
  // mirrored, everything the union reaches gains those ancestors. Rows of
  // dead cluster ids are never queried again; stale references to them are
  // inert, and live roots received their update at the merge that killed
  // the id, so skipping dead ids while cascading is exact.
  void mergeInto(int a, int b) {
    const size_t w = static_cast<size_t>(words_);
    Words outDelta(w, 0), inDelta(w, 0);
    for (size_t i = 0; i < w; ++i) {
      outDelta[i] = fwd_[static_cast<size_t>(a)][i] | fwd_[static_cast<size_t>(b)][i];
      inDelta[i] = bwd_[static_cast<size_t>(a)][i] | bwd_[static_cast<size_t>(b)][i];
      (void)0;
    }
    outDelta[static_cast<size_t>(a) >> 6] |= 1ull << (static_cast<uint64_t>(a) & 63);
    inDelta[static_cast<size_t>(a) >> 6] |= 1ull << (static_cast<uint64_t>(a) & 63);
    for (size_t i = 0; i < w; ++i) {
      uint64_t bits = inDelta[i];
      while (bits != 0) {
        const int bit = static_cast<int>(__builtin_ctzll(bits));
        bits &= bits - 1;
        const int z = static_cast<int>(i * 64 + static_cast<size_t>(bit));
        if (z == a || z == b || find(z) != z) continue;
        Words& row = fwd_[static_cast<size_t>(z)];
        for (size_t j = 0; j < w; ++j) row[j] |= outDelta[j];
      }
    }
    for (size_t i = 0; i < w; ++i) {
      uint64_t bits = outDelta[i];
      while (bits != 0) {
        const int bit = static_cast<int>(__builtin_ctzll(bits));
        bits &= bits - 1;
        const int t = static_cast<int>(i * 64 + static_cast<size_t>(bit));
        if (t == a || t == b || find(t) != t) continue;
        Words& row = bwd_[static_cast<size_t>(t)];
        for (size_t j = 0; j < w; ++j) row[j] |= inDelta[j];
      }
    }
    for (size_t i = 0; i < w; ++i) {
      fwd_[static_cast<size_t>(a)][i] |= fwd_[static_cast<size_t>(b)][i];
      bwd_[static_cast<size_t>(a)][i] |= bwd_[static_cast<size_t>(b)][i];
      sinkRow_[static_cast<size_t>(a)][i] |= sinkRow_[static_cast<size_t>(b)][i];
      memRow_[static_cast<size_t>(a)][i] |= memRow_[static_cast<size_t>(b)][i];
    }
    ccost_[static_cast<size_t>(a)] += ccost_[static_cast<size_t>(b)];
    parent_[static_cast<size_t>(b)] = a;
    --alive_;
    ++stats_.merges;
  }

  void mergeRounds() {
    const int stop = std::max(1, std::min(opt_.stopCount, n_));
    std::vector<char> matched;
    while (alive_ > stop && stats_.rounds < kMaxRounds) {
      ++stats_.rounds;
      struct Candidate {
        long long num, den, unionCost;
        int a, b;
      };
      std::vector<Candidate> candidates;
      for (int a = 0; a < n_; ++a) {
        if (find(a) != a || cpinned_[static_cast<size_t>(a)]) continue;
        const int partner = bestPartner(a);
        if (partner < 0 || partner == a) continue;
        long long inter = 0, uni = 0;
        weightedIoU(sinkRow_[static_cast<size_t>(a)],
                    sinkRow_[static_cast<size_t>(partner)], &inter, &uni);
        candidates.push_back(Candidate{inter, uni == 0 ? 1 : uni,
                                       ccost_[static_cast<size_t>(a)] +
                                           ccost_[static_cast<size_t>(partner)],
                                       std::min(a, partner), std::max(a, partner)});
      }
      std::sort(candidates.begin(), candidates.end(), [](const Candidate& l, const Candidate& r) {
        const long long lScore = l.num * r.den, rScore = r.num * l.den;
        if (lScore != rScore) return lScore > rScore;
        if (l.unionCost != r.unionCost) return l.unionCost < r.unionCost;
        if (l.a != r.a) return l.a < r.a;
        return l.b < r.b;
      });
      matched.assign(static_cast<size_t>(n_), 0);
      int mergedThisRound = 0;
      for (const Candidate& candidate : candidates) {
        if (alive_ <= stop) break;
        int a = find(candidate.a), b = find(candidate.b);
        if (a == b || matched[static_cast<size_t>(a)] || matched[static_cast<size_t>(b)]) continue;
        if (cpinned_[static_cast<size_t>(a)] || cpinned_[static_cast<size_t>(b)]) continue;
        if (ccost_[static_cast<size_t>(a)] + ccost_[static_cast<size_t>(b)] > opt_.threshold) {
          continue;
        }
        if (mergeCreatesCycle(a, b)) continue;
        if (a > b) std::swap(a, b);
        mergeInto(a, b);
        matched[static_cast<size_t>(a)] = 1;
        matched[static_cast<size_t>(b)] = 1;
        ++mergedThisRound;
      }
      if (mergedThisRound == 0) break;
    }
    stats_.stoppedEarly = alive_ > opt_.stopCount ? alive_ - opt_.stopCount : 0;
  }

  void finishOrder() {
    // Contract bucket edges to cluster edges and run Kahn with min-id
    // tie-breaking. Members ascending == topological order of the input.
    std::vector<std::vector<int>> members(static_cast<size_t>(n_));
    for (int b = 0; b < n_; ++b) {
      members[static_cast<size_t>(find(b))].push_back(b);
      clusterOf_[static_cast<size_t>(b)] = find(b);
    }
    std::vector<std::vector<int>> clusterSucc(static_cast<size_t>(n_));
    std::vector<int> indegree(static_cast<size_t>(n_), 0);
    for (int b = 0; b < n_; ++b) {
      const int from = find(b);
      for (int to : succ_[static_cast<size_t>(b)]) {
        const int target = find(to);
        if (target == from) continue;
        std::vector<int>& list = clusterSucc[static_cast<size_t>(from)];
        if (std::find(list.begin(), list.end(), target) == list.end()) {
          list.push_back(target);
          ++indegree[static_cast<size_t>(target)];
        }
      }
    }
    std::vector<int> heap;
    auto push = [&](int id) {
      heap.push_back(id);
      std::push_heap(heap.begin(), heap.end(), std::greater<int>());
    };
    for (int b = 0; b < n_; ++b) {
      if (find(b) == b && indegree[static_cast<size_t>(b)] == 0) push(b);
    }
    ordered_.clear();
    int emitted = 0;
    while (!heap.empty()) {
      std::pop_heap(heap.begin(), heap.end(), std::greater<int>());
      const int cluster = heap.back();
      heap.pop_back();
      ordered_.push_back(members[static_cast<size_t>(cluster)]);
      emitted += static_cast<int>(members[static_cast<size_t>(cluster)].size());
      for (int target : clusterSucc[static_cast<size_t>(cluster)]) {
        if (--indegree[static_cast<size_t>(target)] == 0) push(target);
      }
    }
    if (emitted != n_) {
      valid_ = false;  // cyclic contraction must never happen; stay safe
      ordered_.clear();
    }
  }

  void measureEdges() {
    for (int b = 0; b < n_; ++b) {
      stats_.crossEdgesBefore += static_cast<long long>(succ_[static_cast<size_t>(b)].size());
    }
    if (!valid_) return;
    for (int b = 0; b < n_; ++b) {
      const int from = find(b);
      for (int to : succ_[static_cast<size_t>(b)]) {
        const int target = find(to);
        if (target == from) continue;
        ++stats_.crossEdgesAfter;
        const Words &l = sinkRow_[static_cast<size_t>(from)];
        const Words& r = sinkRow_[static_cast<size_t>(target)];
        long long inter = 0;
        for (size_t i = 0; i < l.size(); ++i) {
          inter += __builtin_popcountll(l[i] & r[i]);
        }
        stats_.sinkOverlapSum += inter;
        ++stats_.sinkOverlapEdges;
      }
    }
  }
};

}  // namespace mtrepcut

#endif  // MT_REPCUT_FORM_H
