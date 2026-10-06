FIRRTL version 3.3.0
circuit MtTokenReduce :
  module MtTokenReduce :
    input clock : Clock
    input reset : UInt<1>
    input io_in0 : UInt<16>
    input io_in1 : UInt<16>
    input io_in2 : UInt<16>
    input io_in3 : UInt<16>
    output io_r10 : UInt<16>
    output io_r11 : UInt<16>
    output io_r12 : UInt<16>
    output io_r13 : UInt<16>
    output io_r20 : UInt<16>
    output io_r21 : UInt<16>
    output io_r22 : UInt<16>
    output io_r23 : UInt<16>
    output io_r30 : UInt<16>
    output io_r31 : UInt<16>
    output io_r32 : UInt<16>
    output io_r33 : UInt<16>

    regreset r10 : UInt<16>, clock, reset, UInt<16>(0h0000)
    regreset r11 : UInt<16>, clock, reset, UInt<16>(0h0000)
    regreset r12 : UInt<16>, clock, reset, UInt<16>(0h0000)
    regreset r13 : UInt<16>, clock, reset, UInt<16>(0h0000)
    regreset r20 : UInt<16>, clock, reset, UInt<16>(0h0000)
    regreset r21 : UInt<16>, clock, reset, UInt<16>(0h0000)
    regreset r22 : UInt<16>, clock, reset, UInt<16>(0h0000)
    regreset r23 : UInt<16>, clock, reset, UInt<16>(0h0000)
    regreset r30 : UInt<16>, clock, reset, UInt<16>(0h0000)
    regreset r31 : UInt<16>, clock, reset, UInt<16>(0h0000)
    regreset r32 : UInt<16>, clock, reset, UInt<16>(0h0000)
    regreset r33 : UInt<16>, clock, reset, UInt<16>(0h0000)

    connect r10, io_in0
    connect r11, io_in1
    connect r12, io_in2
    connect r13, io_in3

    node sum20 = add(r10, r11)
    node trunc20 = tail(sum20, 1)
    connect r20, trunc20
    node sum21 = add(r11, r12)
    node trunc21 = tail(sum21, 1)
    connect r21, trunc21
    node sum22 = add(r12, r13)
    node trunc22 = tail(sum22, 1)
    connect r22, trunc22
    node sum23 = add(r13, r10)
    node trunc23 = tail(sum23, 1)
    connect r23, trunc23

    node sum30a = add(r20, r10)
    node sum30b = add(sum30a, r11)
    node trunc30 = tail(sum30b, 2)
    connect r30, trunc30
    node sum31a = add(r21, r11)
    node sum31b = add(sum31a, r12)
    node trunc31 = tail(sum31b, 2)
    connect r31, trunc31
    node sum32a = add(r22, r12)
    node sum32b = add(sum32a, r13)
    node trunc32 = tail(sum32b, 2)
    connect r32, trunc32
    node sum33a = add(r23, r13)
    node sum33b = add(sum33a, r10)
    node trunc33 = tail(sum33b, 2)
    connect r33, trunc33

    connect io_r10, r10
    connect io_r11, r11
    connect io_r12, r12
    connect io_r13, r13
    connect io_r20, r20
    connect io_r21, r21
    connect io_r22, r22
    connect io_r23, r23
    connect io_r30, r30
    connect io_r31, r31
    connect io_r32, r32
    connect io_r33, r33
