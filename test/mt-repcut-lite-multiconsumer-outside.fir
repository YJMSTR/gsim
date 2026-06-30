FIRRTL version 3.3.0
circuit MtRepCutLiteMultiConsumerOutside :
  module MtRepCutLiteMultiConsumerOutside :
    input clock : Clock
    input reset : UInt<1>
    input io_a : UInt<8>
    input io_b : UInt<8>
    output io_x : UInt<8>
    output io_y : UInt<8>
    output io_tap : UInt<8>

    reg tap_reg : UInt<8>, clock
    node b_mix = xor(io_b, UInt<8>(0haa))
    node a_mix = xor(io_a, UInt<8>(0h55))
    node x_val = and(a_mix, b_mix)
    node y_val = or(a_mix, b_mix)
    node tap_val = xor(b_mix, x_val)

    connect io_x, x_val
    connect io_y, y_val
    connect tap_reg, tap_val
    connect io_tap, tap_reg
