FIRRTL version 3.3.0
circuit MtRepCutLiteRealCut :
  module MtRepCutLiteRealCut :
    input clock : Clock
    input reset : UInt<1>
    input io_a : UInt<8>
    input io_b : UInt<8>
    output io_x : UInt<8>
    output io_y : UInt<8>

    node a_inc_w = add(io_a, UInt<1>(0h1))
    node a_inc = bits(a_inc_w, 7, 0)
    node b_mix = xor(io_b, UInt<8>(0haa))
    node x_val = and(a_inc, b_mix)
    node y_val = or(a_inc, b_mix)

    connect io_x, x_val
    connect io_y, y_val
