#include "MtSpecialHot.h"

#include <cstdint>
#include <cstdio>

int main() {
  SMtSpecialHot dut;
  uint64_t signature = 1469598103934665603ULL;
  for (unsigned cycle = 0; cycle < 32; ++cycle) {
    dut.set_clock(cycle & 1);
    dut.set_reset(cycle < 2);
    dut.set_enable((cycle >= 4 && cycle < 12) || cycle >= 20);
    dut.step();
    signature ^= dut.get_count();
    signature *= 1099511628211ULL;
  }
  std::printf("signature=%016llx\n", static_cast<unsigned long long>(signature));
  return 0;
}
