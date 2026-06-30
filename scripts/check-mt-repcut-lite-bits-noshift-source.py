#!/usr/bin/env python3
import re
import sys
from pathlib import Path


def fail(msg: str) -> None:
    raise SystemExit(f"mt-repcut-lite-bits-noshift-source failed: {msg}")


def extract_function_body(text: str, name: str) -> str:
    marker = f"static bool {name}("
    start = text.find(marker)
    if start < 0:
        fail(f"missing function {name}")
    paren = text.find("(", start)
    if paren < 0:
        fail(f"missing function signature for {name}")
    paren_depth = 0
    brace = -1
    for idx in range(paren, len(text)):
        ch = text[idx]
        if ch == "(":
            paren_depth += 1
        elif ch == ")":
            paren_depth -= 1
            if paren_depth == 0:
                brace = text.find("{", idx)
                break
    if brace < 0:
        fail(f"missing function body for {name}")
    brace_depth = 0
    for idx in range(brace, len(text)):
        ch = text[idx]
        if ch == "{":
            brace_depth += 1
        elif ch == "}":
            brace_depth -= 1
            if brace_depth == 0:
                return text[brace:idx + 1]
    fail(f"unterminated function {name}")


def extract_case(text: str, label: str) -> str:
    marker = f"case {label}:"
    start = text.find(marker)
    if start < 0:
        fail(f"missing {marker}")
    next_case = re.search(r"\n\s*case\s+OP_[A-Z0-9_]+\s*:", text[start + len(marker):])
    if not next_case:
        fail(f"could not find following case after {label}")
    end = start + len(marker) + next_case.start()
    return text[start:end]


def main() -> None:
    if len(sys.argv) > 2:
        fail("usage: check-mt-repcut-lite-bits-noshift-source.py [cppEmitter.cpp]")
    if len(sys.argv) == 2:
        source = Path(sys.argv[1])
    else:
        source = Path(__file__).resolve().parents[1] / "src" / "cppEmitter.cpp"
    text = source.read_text(errors="ignore")
    body = extract_function_body(text, "mtRepCutExprString")
    bits_case = extract_case(body, "OP_BITS")
    noshift_case = extract_case(body, "OP_BITS_NOSHIFT")

    if "std::to_string(enode->values[1])" not in bits_case:
        fail("OP_BITS case no longer emits the expected right-shift by lo")
    if "enode->values[1] >= childNode->width" not in bits_case:
        fail("OP_BITS case is missing the out-of-range zero guard")
    if "bitMask(enode->width)" not in bits_case:
        fail("OP_BITS case is missing width mask")

    if "std::to_string(enode->values[1])" in noshift_case:
        fail("OP_BITS_NOSHIFT case still emits shifted OP_BITS form")
    if "bitMask(enode->values[0] + 1)" not in noshift_case:
        fail("OP_BITS_NOSHIFT case is missing hi-inclusive range mask")
    if "shiftBits(enode->values[1], ShiftDir::Right)" not in noshift_case:
        fail("OP_BITS_NOSHIFT case is missing range-mask right shift")
    if "shiftBits(enode->values[1], ShiftDir::Left)" not in noshift_case:
        fail("OP_BITS_NOSHIFT case is missing range-mask left shift")
    if "enode->values[1] >= childNode->width" not in noshift_case:
        fail("OP_BITS_NOSHIFT case is missing the out-of-range zero guard")

    print(f"mt-repcut-lite-bits-noshift-source ok: distinct no-shift range mask in {source}")


if __name__ == "__main__":
    main()
