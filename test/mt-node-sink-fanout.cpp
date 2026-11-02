#include "MtNodeSinkFanout.h"

#include <cstdint>
#include <cstdio>

int main() {
  SMtNodeSinkFanout dut;
  uint64_t signature = 1469598103934665603ULL;
  for (unsigned cycle = 0; cycle < 4096; ++cycle) {
    dut.set_clock(cycle & 1);
    dut.set_reset(cycle < 4);
    dut.set_a(static_cast<uint8_t>(cycle * 7 + 1));
    dut.set_b(static_cast<uint8_t>(cycle * 13 + 5));
    dut.step();
    signature = (signature ^ dut.get_o0()) * 1099511628211ULL;
    signature = (signature ^ dut.get_o1()) * 1099511628211ULL;
    signature = (signature ^ dut.get_o2()) * 1099511628211ULL;
    signature = (signature ^ dut.get_o3()) * 1099511628211ULL;
    signature = (signature ^ dut.get_o4()) * 1099511628211ULL;
    signature = (signature ^ dut.get_o5()) * 1099511628211ULL;
  }
  std::printf("%016llx\n", static_cast<unsigned long long>(signature));
}
