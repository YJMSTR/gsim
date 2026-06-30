FIRRTL version 3.3.0
circuit MtRepCutLiteLocalDep :
  module MtRepCutLiteLocalDep :
    input clock : Clock
    input reset : UInt<1>
    input io_a : UInt<8>
    input io_b : UInt<8>
    output io_x : UInt<8>
    output io_y : UInt<8>

    node a_wide = add(pad(io_a, 9), UInt<9>(0h001))
    node source_value = bits(a_wide, 8, 1)
    node b_mix = xor(io_b, UInt<8>(0haa))
    node x_val = and(source_value, b_mix)
    node y_val = or(source_value, b_mix)

    connect io_x, x_val
    connect io_y, y_val
