FIRRTL version 3.3.0
circuit MtRepcutTop :
  module MtRepcutTop :
    input clock : Clock
    input reset : UInt<1>
    input seed : UInt<8>
    output out0 : UInt<8>
    output out1 : UInt<8>
    output mem_out : UInt<8>

    regreset q0 : UInt<8>, clock, reset, UInt<8>(0)
    regreset q1 : UInt<8>, clock, reset, UInt<8>(0h3c)
    regreset q2 : UInt<8>, clock, reset, UInt<8>(0h5a)

    node a0 = xor(seed, q0)
    node b0 = xor(q1, q2)
    node s0 = bits(add(a0, b0), 7, 0)
    node a1 = xor(a0, s0)
    node b1 = xor(b0, s0)
    node s1 = bits(add(a1, b1), 7, 0)
    node a2 = xor(a1, s1)
    node b2 = xor(b1, s1)
    node s2 = bits(add(a2, b2), 7, 0)
    node a3 = xor(a2, s2)
    node b3 = xor(b2, s2)
    node s3 = bits(add(a3, b3), 7, 0)
    node a4 = xor(a3, s3)
    node b4 = xor(b3, s3)
    node s4 = bits(add(a4, b4), 7, 0)
    node a5 = xor(a4, s4)
    node b5 = xor(b4, s4)
    node s5 = bits(add(a5, b5), 7, 0)

    cmem tbl : UInt<8> [4]
    read mport tr = tbl[bits(a3, 1, 0)], clock
    when bits(b3, 0, 0) :
      write mport tw = tbl[bits(s3, 1, 0)], clock
      connect tw, bits(add(s4, a4), 7, 0)
    connect mem_out, tr

    printf(clock, bits(s5, 3, 3), "tick %x\n", s5)

    connect q0, bits(add(s5, seed), 7, 0)
    connect q1, xor(a5, seed)
    connect q2, bits(add(b5, s4), 7, 0)
    connect out0, s4
    connect out1, s5
