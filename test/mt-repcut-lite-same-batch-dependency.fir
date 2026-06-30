FIRRTL version 3.3.0
circuit MtRepCutLiteSameBatchDependency :
  module MtRepCutLiteSameBatchDependency :
    input clock : Clock
    input reset : UInt<1>
    input io_a : UInt<8>
    input io_b : UInt<8>
    input io_c : UInt<8>
    output io_x : UInt<8>
    output io_y : UInt<8>
    output io_z : UInt<8>

    node shared = xor(io_a, io_b)
    node b_mix = xor(io_b, UInt<8>(0haa))
    node source_value = and(shared, io_c)
    node shared_tap = xor(shared, io_c)
    node x_val = and(source_value, b_mix)
    node y_val = or(source_value, b_mix)

    connect io_z, shared_tap
    connect io_x, x_val
    connect io_y, y_val
