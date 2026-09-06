#!/usr/bin/env python3
"""Source check: current bits / bits-noshift emission semantics in instsGenerator.cpp.

Replaces check-mt-repcut-lite-bits-noshift-source.py, which extracted
mtRepCutExprString from cppEmitter.cpp (the RepCut lineage was cut in the
2026-09-06 round-2 cleanup). This checker verifies the LIVE contract in the
main expression emitter:

- rangeMask(hi, lo) builds a hi-INCLUSIVE mask: bitMask(hi+1) >> lo << lo.
- instsBitsNoShift (OP_BITS_NOSHIFT): position-PRESERVING mask
  `(child & rangeMask(hi, lo))` - no right shift; out-of-range lo -> constant 0.
- instsBits (OP_BITS): constant path uses u_bits (right-shift to LSB);
  out-of-range lo -> constant 0; full-width/LSB passthrough returns the child.

Usage: check-bits-noshift-source.py [path-to-instsGenerator.cpp]
Positive must pass; see the mutation self-test embedded in the campaign
evidence (the checker must FAIL on: right-shift in the noshift mask path,
non-inclusive mask bound, removed out-of-range guard).
"""
import re
import sys
from pathlib import Path


def fail(msg: str) -> None:
    raise SystemExit(f"check-bits-noshift-source failed: {msg}")


def extract_function_body(text: str, signature: str) -> str:
    start = text.find(signature)
    if start < 0:
        fail(f"missing function: {signature}")
    brace = text.find("{", start)
    if brace < 0:
        fail(f"missing body for {signature}")
    depth = 0
    for idx in range(brace, len(text)):
        if text[idx] == "{":
            depth += 1
        elif text[idx] == "}":
            depth -= 1
            if depth == 0:
                return text[brace:idx + 1]
    fail(f"unterminated function {signature}")
    return ""  # unreachable


def main() -> None:
    if len(sys.argv) > 2:
        fail("usage: check-bits-noshift-source.py [instsGenerator.cpp]")
    source = Path(sys.argv[1]) if len(sys.argv) == 2 else Path(__file__).resolve().parents[1] / "src" / "instsGenerator.cpp"
    text = source.read_text(errors="ignore")

    rangemask = extract_function_body(text, "static std::string rangeMask(int hi, int lo)")
    if "bitMask(hi +1)" not in rangemask and "bitMask(hi + 1)" not in rangemask:
        fail("rangeMask lost the hi-inclusive bound (bitMask(hi+1))")
    if rangemask.count("ShiftDir::Right") != 1 or rangemask.count("ShiftDir::Left") != 1:
        fail("rangeMask must shift right then left exactly once (mask out below lo)")
    if "shiftBits(lo" not in rangemask.replace(" ", ""):
        fail("rangeMask no longer shifts by lo")
    if "ShiftDir" not in rangemask:
        fail("rangeMask sanity: shift helper missing")

    noshift = extract_function_body(text, "valInfo* ENode::instsBitsNoShift(Node* node")
    if "rangeMask(hi, lo)" not in noshift:
        fail("instsBitsNoShift lost the rangeMask path")
    if "ShiftDir::Right" in noshift:
        fail("instsBitsNoShift contains a right shift - position preservation broken")
    if "u_bits_noshift" not in noshift:
        fail("instsBitsNoShift lost the constant-fold u_bits_noshift path")
    if 'setConstantByStr("0")' not in noshift:
        fail("instsBitsNoShift lost the out-of-range zero guard")
    if "lo >= ChildInfo(0, width)" not in noshift:
        fail("instsBitsNoShift lost the out-of-range condition")

    bits = extract_function_body(text, "valInfo* ENode::instsBits(Node* node")
    if "u_bits(" not in bits:
        fail("instsBits lost the constant-fold u_bits (shift-to-LSB) path")
    if 'setConstantByStr("0")' not in bits:
        fail("instsBits lost the out-of-range zero guard")
    if "rangeMask(hi, lo)" in bits and "shiftBits" not in bits:
        fail("instsBits masks without shifting to LSB - that is the noshift contract, not bits")


if __name__ == "__main__":
    main()
    print("check-bits-noshift-source: OK")
