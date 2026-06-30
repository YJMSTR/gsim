FIRRTL version 3.3.0
circuit MtRepCutLiteEqWideLiteral :
  module MtRepCutLiteEqWideLiteral :
    input clock : Clock
    input reset : UInt<1>
    input io_a : UInt<128>
    input io_b : UInt<1>
    output io_x : UInt<1>
    output io_y : UInt<1>
    output io_tap : UInt<1>

    reg tap_reg : UInt<1>, clock
    node eq_hit = eq(io_a, UInt<128>(0h0123456789abcdef0123456789abcdef))
    node neq_hit = neq(io_b, UInt<1>(0h0))
    node x_val = and(eq_hit, neq_hit)
    node y_val = or(eq_hit, neq_hit)
    node tap_val = xor(eq_hit, x_val)

    connect io_x, x_val
    connect io_y, y_val
    connect tap_reg, tap_val
    connect io_tap, tap_reg
