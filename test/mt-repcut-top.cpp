#include "MtRepcutTop.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>

int main() {
  setenv("GSIM_MT_EXECUTOR", "dense", 1);
  SMtRepcutTop dut;
  uint64_t signature = 1469598103934665603ULL;
  for (unsigned cycle = 0; cycle < 20000; ++cycle) {
    dut.set_clock(cycle & 1);
    dut.set_reset(cycle < 4);
    dut.set_seed(static_cast<uint8_t>(cycle * 7 + 1));
    dut.step();
    signature = (signature ^ dut.get_out0()) * 1099511628211ULL;
    signature = (signature ^ dut.get_out1()) * 1099511628211ULL;
    signature = (signature ^ dut.get_mem_out()) * 1099511628211ULL;
  }
  std::printf("%016llx\n", static_cast<unsigned long long>(signature));
}
