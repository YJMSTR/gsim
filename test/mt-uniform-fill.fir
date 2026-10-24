FIRRTL version 3.3.0
circuit UFTest :
  module UFTest :
    input clock : Clock
    input reset : UInt<1>
    input en : UInt<1>
    input a : UInt<8>
    input idx : UInt<4>
    output out : UInt<8>
    output out2 : UInt<8>
    wire tmp : UInt<8>[16]
    connect tmp[0], UInt<8>(0)
    connect tmp[1], UInt<8>(0)
    connect tmp[2], UInt<8>(0)
    connect tmp[3], UInt<8>(0)
    connect tmp[4], UInt<8>(0)
    connect tmp[5], UInt<8>(0)
    connect tmp[6], UInt<8>(0)
    connect tmp[7], UInt<8>(0)
    connect tmp[8], UInt<8>(0)
    connect tmp[9], UInt<8>(0)
    connect tmp[10], UInt<8>(0)
    connect tmp[11], UInt<8>(0)
    when en :
      connect tmp[12], a
      connect tmp[13], a
      connect tmp[14], a
      connect tmp[15], a
    connect out, tmp[idx]
    connect out2, tmp[15]
