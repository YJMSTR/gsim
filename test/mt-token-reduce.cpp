#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "MtTokenReduce.h"

/* Worker width baked into the dense MT model this driver may run against.
   The default single-thread emitter ignores the environment entirely; the
   MT model aborts in mtInit() unless GSIM_THREADS matches its generated
   width, so keep this in sync with GSIM_THREADS at generation time. */
#ifndef MT_TOKEN_REDUCE_THREADS
#define MT_TOKEN_REDUCE_THREADS 2
#endif

namespace {

constexpr int kLanes = 4;
constexpr int kCycles = 256;

uint16_t nextInput(uint32_t& state) {
  state = state * 1664525u + 1013904223u;
  return static_cast<uint16_t>(state >> 16);
}

}  // namespace

int main() {
  setenv("GSIM_MT_EXECUTOR", "dense", 1);
  char threads[16];
  std::snprintf(threads, sizeof(threads), "%d", MT_TOKEN_REDUCE_THREADS);
  setenv("GSIM_THREADS", threads, 1);

  SMtTokenReduce dut;
  uint32_t randomState = 0x5eed1234u;
  uint16_t r1[kLanes] = {};
  uint16_t r2[kLanes] = {};
  uint16_t r3[kLanes] = {};
  // The emitted model recomputes outputs inside step() before the register
  // commit, so observable outputs lag the oracle by exactly one step; cycle 0
  // observes the all-zero reset state.
  uint16_t lag1[kLanes] = {};
  uint16_t lag2[kLanes] = {};
  uint16_t lag3[kLanes] = {};

  for (int cycle = 0; cycle < kCycles; ++cycle) {
    uint16_t in[kLanes];
    for (int lane = 0; lane < kLanes; ++lane) in[lane] = nextInput(randomState);
    dut.set_io_in0(in[0]);
    dut.set_io_in1(in[1]);
    dut.set_io_in2(in[2]);
    dut.set_io_in3(in[3]);
    dut.step();
    /* All three stages sample the pre-edge register values:
       r1' = in, r2' = r1[lane] + r1[lane+1], r3' = r2[lane] + r1[lane] + r1[lane+1],
       with UInt<16> wraparound. */
    uint16_t old1[kLanes];
    uint16_t old2[kLanes];
    for (int lane = 0; lane < kLanes; ++lane) {
      old1[lane] = r1[lane];
      old2[lane] = r2[lane];
    }
    for (int lane = 0; lane < kLanes; ++lane) {
      const int next = (lane + 1) % kLanes;
      r1[lane] = in[lane];
      r2[lane] = static_cast<uint16_t>(old1[lane] + old1[next]);
      r3[lane] = static_cast<uint16_t>(old2[lane] + old1[lane] + old1[next]);
    }

    const uint16_t got1[kLanes] = {dut.get_io_r10(), dut.get_io_r11(), dut.get_io_r12(),
                                   dut.get_io_r13()};
    const uint16_t got2[kLanes] = {dut.get_io_r20(), dut.get_io_r21(), dut.get_io_r22(),
                                   dut.get_io_r23()};
    const uint16_t got3[kLanes] = {dut.get_io_r30(), dut.get_io_r31(), dut.get_io_r32(),
                                   dut.get_io_r33()};
    bool ok = true;
    for (int lane = 0; lane < kLanes; ++lane) {
      if (got1[lane] != lag1[lane] || got2[lane] != lag2[lane] || got3[lane] != lag3[lane]) {
        std::fprintf(stderr,
                     "mt-token-reduce: cycle %d lane %d: r1=%u/%u r2=%u/%u r3=%u/%u\n",
                     cycle, lane, got1[lane], lag1[lane], got2[lane], lag2[lane],
                     got3[lane], lag3[lane]);
        ok = false;
      }
    }
    if (!ok) return 1;
    for (int lane = 0; lane < kLanes; ++lane) {
      lag1[lane] = r1[lane];
      lag2[lane] = r2[lane];
      lag3[lane] = r3[lane];
    }
  }

  std::puts("mt token reduce: PASS");
  return 0;
}
