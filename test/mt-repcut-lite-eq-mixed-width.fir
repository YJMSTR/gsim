FIRRTL version 3.3.0
circuit MtRepCutLiteEqMixedWidth :
  module MtRepCutLiteEqMixedWidth :
    input clock : Clock
    input reset : UInt<1>
    input io_a : UInt<8>
    input io_b : UInt<8>
    output io_x : UInt<1>
    output io_y : UInt<1>
    output io_tap : UInt<1>

    reg tap_reg : UInt<1>, clock
    node a_low = bits(io_a, 3, 0)
    node eq_hit = eq(a_low, UInt<8>(0h0a))
    node neq_hit = neq(io_b, UInt<8>(0h00))
    node x_val = and(eq_hit, neq_hit)
    node y_val = or(eq_hit, neq_hit)
    node tap_val = xor(eq_hit, x_val)

    connect io_x, x_val
    connect io_y, y_val
    connect tap_reg, tap_val
    connect io_tap, tap_reg
