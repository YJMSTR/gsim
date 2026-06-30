FIRRTL version 3.3.0
circuit MtRepCutLiteSameSourceDep :
  module MtRepCutLiteSameSourceDep :
    input clock : Clock
    input reset : UInt<1>
    input io_a : UInt<8>
    input io_b : UInt<8>
    output io_x : UInt<8>
    output io_y : UInt<8>
    output io_z : UInt<8>

    reg z_reg : UInt<8>, clock
    node shared = xor(io_a, UInt<8>(0h33))
    node source_value = and(shared, UInt<8>(0hf0))
    node b_mix = xor(io_b, UInt<8>(0haa))
    node x_val = and(source_value, b_mix)
    node y_val = or(source_value, b_mix)
    node z_val = xor(shared, x_val)

    connect io_x, x_val
    connect io_y, y_val
    connect z_reg, z_val
    connect io_z, z_reg
