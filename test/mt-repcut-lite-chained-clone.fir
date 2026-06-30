FIRRTL version 3.3.0
circuit MtRepCutLiteChainedClone :
  module MtRepCutLiteChainedClone :
    input clock : Clock
    input reset : UInt<1>
    input io_a : UInt<8>
    input io_b : UInt<8>
    output io_x : UInt<8>
    output io_y : UInt<8>
    output io_base : UInt<8>
    output io_derived : UInt<8>

    node base = xor(io_b, UInt<8>(0haa))
    node derived = and(base, io_a)
    node x_val = xor(derived, base)
    node y_val = or(derived, base)
    node base_tap = xor(base, io_a)
    node derived_tap = or(derived, io_b)

    connect io_x, x_val
    connect io_y, y_val
    connect io_base, base_tap
    connect io_derived, derived_tap
