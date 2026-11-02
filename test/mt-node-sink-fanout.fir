FIRRTL version 3.3.0
circuit MtNodeSinkFanout :
  module MtNodeSinkFanout :
    input clock : Clock
    input reset : UInt<1>
    input a : UInt<8>
    input b : UInt<8>
    output o0 : UInt<16>
    output o1 : UInt<16>
    output o2 : UInt<16>
    output o3 : UInt<16>
    output o4 : UInt<16>
    output o5 : UInt<16>

    node fan = xor(a, b)
    node m0 = bits(add(fan, a), 15, 0)
    node m1 = bits(add(fan, b), 15, 0)
    node m2 = bits(xor(fan, a), 15, 0)
    node m3 = bits(xor(fan, b), 15, 0)
    node m4 = bits(add(xor(fan, a), b), 15, 0)
    node m5 = bits(add(xor(fan, b), a), 15, 0)
    node c00 = bits(add(m0, b), 15, 0)
    node c01 = bits(xor(c00, a), 15, 0)
    node c02 = bits(add(c01, b), 15, 0)
    node c10 = bits(add(m1, a), 15, 0)
    node c11 = bits(xor(c10, b), 15, 0)
    node c12 = bits(add(c11, a), 15, 0)
    node c20 = bits(add(m2, b), 15, 0)
    node c21 = bits(xor(c20, a), 15, 0)
    node c22 = bits(add(c21, b), 15, 0)
    node c30 = bits(add(m3, a), 15, 0)
    node c31 = bits(xor(c30, b), 15, 0)
    node c32 = bits(add(c31, a), 15, 0)
    node c40 = bits(add(m4, b), 15, 0)
    node c41 = bits(xor(c40, a), 15, 0)
    node c42 = bits(add(c41, b), 15, 0)
    node c50 = bits(add(m5, a), 15, 0)
    node c51 = bits(xor(c50, b), 15, 0)
    node c52 = bits(add(c51, a), 15, 0)
    regreset q0 : UInt<16>, clock, reset, UInt<16>(0)
    regreset q1 : UInt<16>, clock, reset, UInt<16>(0)
    regreset q2 : UInt<16>, clock, reset, UInt<16>(0)
    regreset q3 : UInt<16>, clock, reset, UInt<16>(0)
    regreset q4 : UInt<16>, clock, reset, UInt<16>(0)
    regreset q5 : UInt<16>, clock, reset, UInt<16>(0)
    connect q0, c02
    connect q1, c12
    connect q2, c22
    connect q3, c32
    connect q4, c42
    connect q5, c52
    connect o0, q0
    connect o1, q1
    connect o2, q2
    connect o3, q3
    connect o4, q4
    connect o5, q5
