FIRRTL version 3.3.0
circuit MtSpecialHot :
  module MtSpecialHot :
    input clock : Clock
    input reset : UInt<1>
    input enable : UInt<1>
    output count : UInt<8>

    regreset counter : UInt<8>, clock, reset, UInt<8>(0)
    connect counter, bits(add(counter, UInt<1>(1)), 7, 0)
    connect count, counter
    when enable :
      printf(clock, UInt<1>(1), "HOT\n")
