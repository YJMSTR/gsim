#!/usr/bin/env python3
"""Validate v280 compile-exclusive dense-MTask owner-banked generated code."""

import argparse
import mmap
import re
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set, Tuple


CHECKER = "check-mt-dense-owner-bank"
OWNER_COMPILE_MACRO = "GSIM_MT_DENSE_OWNER_BANK_COUNTERS_COMPILE"
SLOTS_PER_CACHE_LINE = 16


def fail(message: str) -> None:
    raise SystemExit(f"{CHECKER} failed: {message}")


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError as error:
        fail(f"could not read {path}: {error}")
    raise AssertionError("unreachable")


def mask_cpp_noncode(text: str) -> str:
    """Replace comments and quoted contents with spaces while retaining positions."""
    chars = list(text)
    index = 0
    state = "code"
    while index < len(text):
        char = text[index]
        next_char = text[index + 1] if index + 1 < len(text) else ""
        if state == "code":
            if char == "/" and next_char == "/":
                chars[index] = chars[index + 1] = " "
                index += 2
                state = "line-comment"
                continue
            if char == "/" and next_char == "*":
                chars[index] = chars[index + 1] = " "
                index += 2
                state = "block-comment"
                continue
            if char == '"':
                chars[index] = " "
                state = "string"
            elif char == "'":
                chars[index] = " "
                state = "char"
        elif state == "line-comment":
            if char == "\n":
                state = "code"
            else:
                chars[index] = " "
        elif state == "block-comment":
            if char == "*" and next_char == "/":
                chars[index] = chars[index + 1] = " "
                index += 2
                state = "code"
                continue
            if char != "\n":
                chars[index] = " "
        else:
            quote = '"' if state == "string" else "'"
            if char == "\\":
                chars[index] = " "
                if index + 1 < len(text):
                    if text[index + 1] != "\n":
                        chars[index + 1] = " "
                    index += 2
                    continue
            elif char == quote:
                chars[index] = " "
                state = "code"
            elif char != "\n":
                chars[index] = " "
        index += 1
    return "".join(chars)


def find_matching_brace(masked: str, opening: int, context: str) -> int:
    if opening >= len(masked) or masked[opening] != "{":
        fail(f"internal parse error: {context} does not begin with '{{'")
    depth = 0
    for index in range(opening, len(masked)):
        if masked[index] == "{":
            depth += 1
        elif masked[index] == "}":
            depth -= 1
            if depth == 0:
                return index
    fail(f"unterminated brace-delimited {context}")
    raise AssertionError("unreachable")


def unique_header(model_dir: Path, label: str) -> Tuple[Path, str]:
    headers = sorted(
        path for path in model_dir.glob("*.h")
        if path.is_file() and not path.name.endswith(".tmp.h")
    )
    if len(headers) != 1:
        names = ", ".join(path.name for path in headers) or "none"
        fail(f"{label}: expected one generated *.h header, found {len(headers)} ({names})")
    return headers[0], read_text(headers[0])


COMPILE_BLOCK_RE = re.compile(
    r"^[ \t]*#if\s+defined\s*\(\s*" + OWNER_COMPILE_MACRO
    + r"\s*\)\s*&&\s*" + OWNER_COMPILE_MACRO + r"\s*(?:\r?\n)"
    + r"(?P<owner>.*?)"
    + r"^[ \t]*#else\s*(?:\r?\n)"
    + r"(?P<identity>.*?)"
    + r"^[ \t]*#endif[^\r\n]*(?:\r?\n|$)",
    re.MULTILINE | re.DOTALL,
)
WORKER_SIGNATURE_RE = re.compile(
    r"\bstepDenseThreadWorker\s*\(\s*int\s+threadId\s*\)\s*\{"
)
WORKER_PROLOGUE_RE = re.compile(
    r"\bstepDenseThreadWorker\s*\(\s*int\s+threadId\s*\)\s*\{\s*"
    r"bool\s+evenCycle\s*=\s*\(\s*cycles\s*&\s*1\s*\)\s*==\s*0\s*;"
)


def compile_blocks(text: str) -> List[Tuple[str, str]]:
    return [
        (match.group("owner"), match.group("identity"))
        for match in COMPILE_BLOCK_RE.finditer(text)
    ]


def select_compile_variant(text: str, branch: str, context: str) -> Tuple[str, int]:
    if branch not in ("owner", "identity"):
        fail(f"internal parse error: invalid compile branch {branch}")
    selected, count = COMPILE_BLOCK_RE.subn(lambda match: match.group(branch), text)
    if OWNER_COMPILE_MACRO in selected:
        fail(f"{context}: unrecognized or non-two-way compile-macro use")
    return selected, count


def natural_name_key(path: Path) -> Tuple[Tuple[int, object], ...]:
    return tuple(
        (1, int(part)) if part.isdigit() else (0, part)
        for part in re.split(r"([0-9]+)", path.name)
    )


def find_worker_source(model_dir: Path, label: str) -> Tuple[Path, str]:
    cpp_paths = sorted(
        (path for path in model_dir.glob("*.cpp") if path.is_file()),
        key=natural_name_key,
        reverse=True,
    )
    if not cpp_paths:
        fail(f"{label}: no generated *.cpp files under {model_dir}")
    needle = b"stepDenseThreadWorker(int threadId)"
    matched_path: Optional[Path] = None
    for path in cpp_paths:
        try:
            if path.stat().st_size == 0:
                continue
            with path.open("rb") as stream:
                with mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as mapped:
                    if mapped.find(needle) != -1:
                        matched_path = path
                        break
        except OSError as error:
            fail(f"{label}: could not scan {path}: {error}")
    if matched_path is None:
        fail(f"{label}: no C++ file contains the fixed worker signature")
    text = read_text(matched_path)
    masked = mask_cpp_noncode(text)
    signatures = list(WORKER_SIGNATURE_RE.finditer(masked))
    if len(signatures) != 1:
        fail(
            f"{label}: expected exactly one stepDenseThreadWorker definition in "
            f"{matched_path}, found {len(signatures)}"
        )
    nondirective = re.sub(
        r"(?m)^[ \t]*#(?:if|elif|else|endif)[^\r\n]*$", "", text
    )
    if OWNER_COMPILE_MACRO in nondirective:
        fail(f"{matched_path}: runtime/non-directive compile-macro use")
    if re.search(
        r"\bgetenv\s*\([^)]*GSIM_MT_DENSE_OWNER_BANK_COUNTERS", text, re.DOTALL
    ):
        fail(f"{matched_path}: generated runtime owner-bank environment lookup")
    return matched_path, text


def extract_worker_definition(source: str, source_path: Path, label: str) -> str:
    masked_source = mask_cpp_noncode(source)
    matches = list(WORKER_SIGNATURE_RE.finditer(masked_source))
    if len(matches) != 1:
        fail(
            f"{label}: expected one surviving stepDenseThreadWorker definition in "
            f"{source_path}, found {len(matches)}"
        )
    opening = masked_source.find("{", matches[0].start(), matches[0].end())
    closing = find_matching_brace(
        masked_source, opening, f"stepDenseThreadWorker in {source_path}"
    )
    worker = masked_source[matches[0].start():closing + 1]
    if WORKER_PROLOGUE_RE.match(worker) is None:
        fail(
            f"{source_path}: {label} worker must begin with "
            "bool evenCycle = (cycles & 1) == 0;"
        )
    return worker


def parse_integer_literal(token: str, context: str) -> int:
    compact = token.strip().replace("'", "")
    if not re.fullmatch(
        r"[+-]?(?:(?:0[xX][0-9a-fA-F]+)|(?:0[bB][01]+)|(?:0[0-7]*)|(?:[1-9][0-9]*))[uUlL]*",
        compact,
    ):
        fail(f"{context}: unsupported non-integer initializer {token.strip()!r}")
    core = re.sub(r"[uUlL]+$", "", compact)
    sign = -1 if core.startswith("-") else 1
    unsigned = core[1:] if core[:1] in "+-" else core
    if unsigned.lower().startswith("0x"):
        value = int(unsigned[2:], 16)
    elif unsigned.lower().startswith("0b"):
        value = int(unsigned[2:], 2)
    elif len(unsigned) > 1 and unsigned.startswith("0"):
        value = int(unsigned, 8)
    else:
        value = int(unsigned, 10)
    return sign * value


def parse_array(masked_header: str, name: str, label: str) -> List[int]:
    pattern = re.compile(
        r"\bstatic\s+constexpr\s+[^;={}]*?\b"
        + re.escape(name)
        + r"\s*\[\s*(\d+)\s*\]\s*=\s*\{(.*?)\}\s*;",
        re.DOTALL,
    )
    matches = list(pattern.finditer(masked_header))
    if len(matches) != 1:
        fail(f"{label}: expected one initialized {name} array, found {len(matches)}")
    declared_count = int(matches[0].group(1))
    tokens = [token for token in matches[0].group(2).split(",") if token.strip()]
    values = [
        parse_integer_literal(token, f"{label} {name}[{index}]")
        for index, token in enumerate(tokens)
    ]
    if len(values) != declared_count:
        fail(
            f"{label}: {name} declares {declared_count} entries but initializes {len(values)}"
        )
    return values


def parse_optional_array(masked_header: str, name: str, label: str) -> Optional[List[int]]:
    if re.search(r"\b" + re.escape(name) + r"\b", masked_header) is None:
        return None
    return parse_array(masked_header, name, label)


def parse_optional_scalar(masked_header: str, name: str, label: str) -> Optional[int]:
    if re.search(r"\b" + re.escape(name) + r"\b", masked_header) is None:
        return None
    pattern = re.compile(
        r"\bstatic\s+constexpr\s+[^;={}]*?\b"
        + re.escape(name)
        + r"\s*=\s*([^;]+);"
    )
    matches = list(pattern.finditer(masked_header))
    if len(matches) != 1:
        fail(f"{label}: expected one initialized {name} constant, found {len(matches)}")
    return parse_integer_literal(matches[0].group(1), f"{label} {name}")


def parse_storage(masked_header: str, label: str) -> Tuple[int, bool]:
    pattern = re.compile(
        r"^[ \t]*(?P<alignment>alignas\s*\(\s*64\s*\)\s*)?"
        r"MtDenseMTaskVertex\s+mtDenseMTaskVertices\s*\[\s*(\d+)\s*\]\s*;",
        re.MULTILINE,
    )
    matches = list(pattern.finditer(masked_header))
    if len(matches) != 1:
        fail(
            f"{label}: expected one literal-sized MtDenseMTaskVertex storage declaration, "
            f"found {len(matches)}"
        )
    return int(matches[0].group(2)), matches[0].group("alignment") is not None


def first_difference(left: Sequence[int], right: Sequence[int]) -> Optional[int]:
    for index, (left_value, right_value) in enumerate(zip(left, right)):
        if left_value != right_value:
            return index
    if len(left) != len(right):
        return min(len(left), len(right))
    return None


def require_equal(
    actual: Sequence[int], expected: Sequence[int], what: str,
    actual_label: str, expected_label: str,
) -> None:
    difference = first_difference(actual, expected)
    if difference is None:
        return
    if difference >= len(actual) or difference >= len(expected):
        fail(
            f"{what}: {actual_label} has {len(actual)} entries, "
            f"{expected_label} has {len(expected)}"
        )
    fail(
        f"{what}: entry {difference} is {actual[difference]} in {actual_label}, "
        f"expected {expected[difference]} from {expected_label}"
    )


def extract_worker_cases(masked_function: str, source_path: Path) -> Dict[int, str]:
    switch_re = re.compile(r"\bswitch\s*\(\s*threadId\s*\)\s*\{")
    switches = list(switch_re.finditer(masked_function))
    if len(switches) != 1:
        fail(
            f"{source_path}: expected one fixed switch(threadId) in stepDenseThreadWorker, "
            f"found {len(switches)}"
        )
    opening = masked_function.find("{", switches[0].start(), switches[0].end())
    closing = find_matching_brace(
        masked_function, opening, f"switch(threadId) in {source_path}"
    )
    body = masked_function[opening + 1:closing]
    labels: List[Tuple[int, int, Optional[int]]] = []
    offset = 0
    depth = 0
    label_re = re.compile(r"^[ \t]*(?:case\s+([0-9]+)\s*:|default\s*:)")
    for line in body.splitlines(keepends=True):
        match = label_re.match(line) if depth == 0 else None
        if match is not None:
            worker = int(match.group(1)) if match.group(1) is not None else None
            labels.append((offset + match.start(), offset + match.end(), worker))
        depth += line.count("{") - line.count("}")
        if depth < 0:
            fail(f"{source_path}: malformed braces in fixed worker switch")
        offset += len(line)
    if depth != 0 or not labels:
        fail(f"{source_path}: malformed or empty fixed worker switch")
    cases: Dict[int, str] = {}
    covered_calls = 0
    for index, (_, content_start, worker) in enumerate(labels):
        end = labels[index + 1][0] if index + 1 < len(labels) else len(body)
        chunk = body[content_start:end]
        chunk_calls = len(re.findall(r"\bstepDenseMTask", chunk))
        covered_calls += chunk_calls
        if worker is None:
            if chunk_calls:
                fail(f"{source_path}: default worker case calls a dense MTask")
            continue
        if worker in cases:
            fail(f"{source_path}: duplicate fixed worker case {worker}")
        cases[worker] = chunk
    if covered_calls != len(re.findall(r"\bstepDenseMTask", body)):
        fail(f"{source_path}: dense MTask call appears outside a numeric worker case")
    return cases


CALL_RE = re.compile(r"\bstepDenseMTask\s*([0-9]+)\s*\(\s*\)\s*;")
WAIT_TARGET_RE = re.compile(
    r"\bconst\s+uint32_t\s+target\s*=\s*evenCycle\s*\?\s*"
    r"kDenseMTaskDepCount\s*\[\s*([0-9]+)\s*\]\s*:\s*0[uUlL]*\s*;"
)
WAIT_PREDICATE_RE = re.compile(
    r"\bwhile\s*\(\s*mtDenseMTaskVertices\s*\[\s*([0-9]+)\s*\]\s*\.\s*"
    r"depsDone\s*\.\s*load\s*\(\s*std::memory_order_acquire\s*\)\s*"
    r"!=\s*target\s*\)\s*\{"
)
ANY_WAIT_LOAD_RE = re.compile(
    r"\bmtDenseMTaskVertices\s*\[[^\]]+\]\s*\.\s*depsDone\s*\.\s*load\s*\("
)
SIGNAL_PAIR_RE = re.compile(
    r"""
    \bif\s*\(\s*evenCycle\s*\)\s*\{\s*
    for\s*\(\s*int\s+j\s*=\s*kDenseMTaskSuccOffsets\s*\[\s*([0-9]+)\s*\]\s*;
    \s*j\s*<\s*kDenseMTaskSuccOffsets\s*\[\s*([0-9]+)\s*\]\s*;
    \s*(?:j\s*\+\+|\+\+\s*j)\s*\)
    \s*mtDenseMTaskVertices\s*\[\s*kDenseMTaskSuccList\s*\[\s*j\s*\]\s*\]\s*
    \.\s*depsDone\s*\.\s*fetch_add\s*\(\s*1\s*,\s*std::memory_order_release\s*\)\s*;
    \s*\}\s*else\s*\{\s*
    for\s*\(\s*int\s+j\s*=\s*kDenseMTaskSuccOffsets\s*\[\s*([0-9]+)\s*\]\s*;
    \s*j\s*<\s*kDenseMTaskSuccOffsets\s*\[\s*([0-9]+)\s*\]\s*;
    \s*(?:j\s*\+\+|\+\+\s*j)\s*\)
    \s*mtDenseMTaskVertices\s*\[\s*kDenseMTaskSuccList\s*\[\s*j\s*\]\s*\]\s*
    \.\s*depsDone\s*\.\s*fetch_sub\s*\(\s*1\s*,\s*std::memory_order_release\s*\)\s*;
    \s*\}
    """,
    re.VERBOSE,
)
SIGNAL_LOOP_RE = re.compile(r"\bfor\s*\([^)]*\bkDenseMTaskSuccOffsets\s*\[", re.DOTALL)
SIGNAL_FETCH_RE = re.compile(r"\bfetch_(?:add|sub)\s*\(")
SIGNAL_OFFSET_REF_RE = re.compile(r"\bkDenseMTaskSuccOffsets\s*\[")
SIGNAL_LIST_REF_RE = re.compile(r"\bkDenseMTaskSuccList\s*\[")
FOUR_BYTE_ASSERT_RE = re.compile(
    r"\bstatic_assert\s*\(\s*sizeof\s*\(\s*MtDenseMTaskVertex\s*\)\s*"
    r"==\s*4[uUlL]*\s*,"
)
RUNTIME_MAPPING_RE = re.compile(
    r"\b(?:kDenseMTaskPhysicalSlotCount|kDenseMTaskVertexSlot|kDenseMTaskVertexOwner)\b"
)


def ownership_from_cases(
    cases: Dict[int, str], mtask_count: int, source_path: Path
) -> Tuple[Dict[int, int], Dict[int, List[int]]]:
    owners: Dict[int, int] = {}
    groups: Dict[int, List[int]] = {}
    occurrences: Dict[int, List[int]] = {}
    for worker in sorted(cases):
        calls = [int(match.group(1)) for match in CALL_RE.finditer(cases[worker])]
        if len(re.findall(r"\bstepDenseMTask", cases[worker])) != len(calls):
            fail(f"{source_path}: worker {worker} has a malformed dense MTask call")
        if calls != sorted(calls):
            fail(f"{source_path}: worker {worker} calls logical MTasks out of order: {calls}")
        groups[worker] = calls
        for logical_id in calls:
            occurrences.setdefault(logical_id, []).append(worker)
            if 0 <= logical_id < mtask_count and logical_id not in owners:
                owners[logical_id] = worker
    out_of_range = sorted(value for value in occurrences if not 0 <= value < mtask_count)
    if out_of_range:
        fail(f"{source_path}: fixed worker calls out-of-range MTasks {out_of_range}")
    duplicates = {
        logical_id: workers for logical_id, workers in sorted(occurrences.items())
        if len(workers) != 1
    }
    if duplicates:
        fail(f"{source_path}: logical MTasks are called more than once: {duplicates}")
    missing = sorted(set(range(mtask_count)) - set(occurrences))
    if missing:
        fail(f"{source_path}: logical MTasks missing from fixed worker switch: {missing}")
    return owners, groups


def expected_owner_bank_map(
    owners: Dict[int, int], groups: Dict[int, List[int]], mtask_count: int
) -> Tuple[List[int], int]:
    slots = [-1] * mtask_count
    next_slot = 0
    for owner in sorted(groups):
        logical_ids = groups[owner]
        next_slot = (
            (next_slot + SLOTS_PER_CACHE_LINE - 1) // SLOTS_PER_CACHE_LINE
        ) * SLOTS_PER_CACHE_LINE
        for logical_id in logical_ids:
            slots[logical_id] = next_slot
            next_slot += 1
        next_slot = (
            (next_slot + SLOTS_PER_CACHE_LINE - 1) // SLOTS_PER_CACHE_LINE
        ) * SLOTS_PER_CACHE_LINE
    if any(slot < 0 for slot in slots) or len(set(slots)) != mtask_count:
        fail(f"recomputed logical-to-physical map is not bijective: {slots}")
    line_owners: Dict[int, set] = {}
    for logical_id, slot in enumerate(slots):
        line_owners.setdefault(slot // SLOTS_PER_CACHE_LINE, set()).add(owners[logical_id])
    mixed = {line: sorted(values) for line, values in line_owners.items() if len(values) > 1}
    if mixed:
        fail(f"recomputed occupied cache lines mix fixed owners: {mixed}")
    return slots, next_slot


def successor_entries(offsets: Sequence[int], stored: Sequence[int], label: str) -> List[int]:
    if not offsets:
        fail(f"{label}: empty kDenseMTaskSuccOffsets")
    edge_count = offsets[-1]
    if edge_count == 0:
        if list(stored) != [0]:
            fail(f"{label}: zero-edge successor list must be sentinel [0]")
        return []
    if len(stored) != edge_count:
        fail(f"{label}: successor list has {len(stored)} entries, expected {edge_count}")
    return list(stored)


def validate_graph(
    dep_counts: Sequence[int], offsets: Sequence[int], successors: Sequence[int], label: str
) -> None:
    mtask_count = len(dep_counts)
    if len(offsets) != mtask_count + 1 or offsets[0] != 0:
        fail(f"{label}: invalid successor-offset array shape")
    for logical_id, (begin, end) in enumerate(zip(offsets, offsets[1:])):
        if begin < 0 or end < begin:
            fail(f"{label}: invalid successor interval for MTask {logical_id}: [{begin}, {end})")
    if offsets[-1] != len(successors):
        fail(f"{label}: final successor offset does not match successor entries")
    invalid = [(index, value) for index, value in enumerate(successors) if not 0 <= value < mtask_count]
    if invalid:
        fail(f"{label}: out-of-range logical successors {invalid}")
    incoming = [0] * mtask_count
    for successor in successors:
        incoming[successor] += 1
    require_equal(dep_counts, incoming, f"{label} graph", "dep counts", "incoming edges")


def parse_waits(
    cases: Dict[int, str], dep_counts: Sequence[int], expected_slots: Sequence[int],
    source_path: Path,
) -> Set[int]:
    waits: List[Tuple[int, int, int, int]] = []
    target_count = load_count = predicate_count = dep_ref_count = 0
    for worker in sorted(cases):
        chunk = cases[worker]
        calls = list(CALL_RE.finditer(chunk))
        targets = list(WAIT_TARGET_RE.finditer(chunk))
        target_count += len(targets)
        load_count += len(ANY_WAIT_LOAD_RE.findall(chunk))
        predicate_count += len(WAIT_PREDICATE_RE.findall(chunk))
        dep_ref_count += len(re.findall(r"\bkDenseMTaskDepCount\s*\[", chunk))
        for index, target in enumerate(targets):
            following = [call for call in calls if call.start() > target.end()]
            if not following:
                fail(f"{source_path}: worker {worker} wait has no following MTask call")
            call = following[0]
            if index + 1 < len(targets) and targets[index + 1].start() < call.start():
                fail(f"{source_path}: worker {worker} has multiple waits before one call")
            segment = chunk[target.start():call.start()]
            predicates = list(WAIT_PREDICATE_RE.finditer(segment))
            all_loads = list(ANY_WAIT_LOAD_RE.finditer(segment))
            while_count = len(re.findall(r"\bwhile\s*\(", segment))
            if len(predicates) != 1 or len(all_loads) != 1 or while_count != 1:
                fail(
                    f"{source_path}: wait before logical MTask {call.group(1)} must use "
                    "one complete literal-slot acquire predicate != target"
                )
            waits.append((
                int(call.group(1)), int(target.group(1)),
                int(predicates[0].group(1)), worker,
            ))
    if not (dep_ref_count == target_count == load_count == predicate_count):
        fail(f"{source_path}: unrecognized or incomplete dependency wait code")
    occurrences: Dict[int, List[int]] = {}
    for logical_id, dep_index, slot, worker in waits:
        occurrences.setdefault(logical_id, []).append(worker)
        if dep_index != logical_id:
            fail(f"{source_path}: MTask {logical_id} waits on dep count {dep_index}")
        if not 0 <= logical_id < len(dep_counts):
            fail(f"{source_path}: wait attached to out-of-range MTask {logical_id}")
        if slot != expected_slots[logical_id]:
            fail(
                f"{source_path}: MTask {logical_id} waits on slot {slot}, "
                f"expected {expected_slots[logical_id]}"
            )
    duplicates = {key: value for key, value in occurrences.items() if len(value) != 1}
    if duplicates:
        fail(f"{source_path}: duplicate fixed waits: {duplicates}")
    actual = set(occurrences)
    full = set(range(len(dep_counts)))
    elided = {logical_id for logical_id, count in enumerate(dep_counts) if count != 0}
    if actual != full and actual != elided:
        fail(
            f"{source_path}: fixed waits are neither complete nor consistently empty-elided; "
            f"actual={sorted(actual)}, full={sorted(full)}, elided={sorted(elided)}"
        )
    return actual


def parse_signal_sources(
    cases: Dict[int, str], offsets: Sequence[int], owners: Dict[int, int], source_path: Path
) -> Dict[int, int]:
    mtask_count = len(offsets) - 1
    occurrences: Dict[int, List[int]] = {}
    for worker in sorted(cases):
        chunk = cases[worker]
        pairs = list(SIGNAL_PAIR_RE.finditer(chunk))
        if (
            len(SIGNAL_LOOP_RE.findall(chunk)) != 2 * len(pairs)
            or len(SIGNAL_FETCH_RE.findall(chunk)) != 2 * len(pairs)
            or len(SIGNAL_OFFSET_REF_RE.findall(chunk)) != 4 * len(pairs)
            or len(SIGNAL_LIST_REF_RE.findall(chunk)) != 2 * len(pairs)
        ):
            fail(
                f"{source_path}: worker {worker} signal code is not exact paired "
                "even fetch_add/odd fetch_sub release loops"
            )
        calls = list(CALL_RE.finditer(chunk))
        for pair in pairs:
            add_begin, add_end, sub_begin, sub_end = (
                int(pair.group(index)) for index in range(1, 5)
            )
            if (add_begin, add_end) != (sub_begin, sub_end):
                fail(f"{source_path}: worker {worker} add/sub signal intervals differ")
            source = add_begin
            if not 0 <= source < mtask_count or add_end != source + 1:
                fail(f"{source_path}: worker {worker} has invalid signal interval")
            preceding = [call for call in calls if call.end() <= pair.start()]
            if not preceding or int(preceding[-1].group(1)) != source:
                fail(f"{source_path}: source {source} signal pair does not follow its call")
            if owners[source] != worker:
                fail(f"{source_path}: source {source} signals outside its fixed owner")
            occurrences.setdefault(source, []).append(worker)
    duplicates = {key: value for key, value in occurrences.items() if len(value) != 1}
    if duplicates:
        fail(f"{source_path}: duplicate signal pairs: {duplicates}")
    actual = set(occurrences)
    full = set(range(mtask_count))
    elided = {
        logical_id for logical_id in range(mtask_count)
        if offsets[logical_id] != offsets[logical_id + 1]
    }
    if actual != full and actual != elided:
        fail(
            f"{source_path}: signal pairs are neither complete nor consistently empty-elided; "
            f"actual={sorted(actual)}, full={sorted(full)}, elided={sorted(elided)}"
        )
    return {source: workers[0] for source, workers in occurrences.items()}

def validate_sync_emission_mode(
    wait_ids: Set[int], signal_sources: Dict[int, int], dep_counts: Sequence[int],
    offsets: Sequence[int], context: str,
) -> str:
    mtask_count = len(dep_counts)
    full_ids = set(range(mtask_count))
    elided_wait_ids = {
        logical_id for logical_id, count in enumerate(dep_counts) if count != 0
    }
    elided_signal_ids = {
        logical_id for logical_id in range(mtask_count)
        if offsets[logical_id] != offsets[logical_id + 1]
    }
    actual_signal_ids = set(signal_sources)
    modes: List[str] = []
    if wait_ids == full_ids and actual_signal_ids == full_ids:
        modes.append("default")
    if wait_ids == elided_wait_ids and actual_signal_ids == elided_signal_ids:
        modes.append("static-empty-elided")
    if not modes:
        fail(
            f"{context}: wait/signal emission mixes default and empty-elided shapes; "
            f"waits={sorted(wait_ids)}, signals={sorted(actual_signal_ids)}"
        )
    return "/".join(modes)


def validate_diagnostics(
    header: str, expected_slots: Sequence[int], expected_owners: Sequence[int],
    expected_physical_count: int, header_path: Path,
) -> None:
    physical_count = parse_optional_scalar(
        header, "kDenseMTaskPhysicalSlotCount", "model"
    )
    if physical_count is not None and physical_count != expected_physical_count:
        fail(f"{header_path}: diagnostic physical slot count is {physical_count}, expected {expected_physical_count}")
    diagnostic_slots = parse_optional_array(header, "kDenseMTaskVertexSlot", "model")
    if diagnostic_slots is not None:
        require_equal(
            diagnostic_slots, expected_slots, "diagnostic slot map",
            "generated", "recomputed",
        )
    diagnostic_owners = parse_optional_array(header, "kDenseMTaskVertexOwner", "model")
    if diagnostic_owners is not None:
        require_equal(
            diagnostic_owners, expected_owners, "diagnostic owner map",
            "generated", "fixed worker ownership",
        )


def validate_bank_storage(header: str, header_path: Path, physical_count: int) -> None:
    storage_count, aligned = parse_storage(header, "owner-bank branch")
    if not aligned or storage_count != physical_count:
        fail(
            f"{header_path}: owner-bank storage is aligned={aligned}, size={storage_count}; "
            f"expected aligned size {physical_count}"
        )
    if len(FOUR_BYTE_ASSERT_RE.findall(header)) != 1:
        fail(f"{header_path}: owner-bank branch needs one four-byte vertex static_assert")


def validate_identity_storage(header: str, header_path: Path, mtask_count: int) -> None:
    storage_count, aligned = parse_storage(header, "identity branch")
    if aligned or storage_count != mtask_count:
        fail(
            f"{header_path}: identity storage is aligned={aligned}, size={storage_count}; "
            f"expected unaligned logical size {mtask_count}"
        )
    if FOUR_BYTE_ASSERT_RE.search(header):
        fail(f"{header_path}: owner-bank static_assert leaked into identity branch")


def reject_runtime_mapping(worker_source: str, worker_path: Path) -> None:
    if RUNTIME_MAPPING_RE.search(worker_source):
        fail(f"{worker_path}: generated worker performs diagnostic mapping/count lookup")
    nondirective = re.sub(
        r"(?m)^[ \t]*#(?:if|elif|else|endif)[^\r\n]*$", "", worker_source
    )
    if re.search(r"\b[A-Za-z_][A-Za-z0-9_]*owner_?bank[A-Za-z0-9_]*\b", nondirective, re.IGNORECASE):
        fail(f"{worker_path}: generated worker contains runtime owner-bank dispatch state")


def classify_header_blocks(blocks: Sequence[Tuple[str, str]], header_path: Path) -> None:
    if len(blocks) != 2:
        fail(f"{header_path}: expected two compile-exclusive blocks, found {len(blocks)}")
    categories: List[str] = []
    for index, (owner, identity) in enumerate(blocks):
        if any(RUNTIME_MAPPING_RE.search(branch) for branch in (owner, identity)):
            fail(f"{header_path}: diagnostics appear inside compile block {index}")
        successor_counts = tuple(
            len(re.findall(r"\bkDenseMTaskSuccList\b", branch))
            for branch in (owner, identity)
        )
        storage_counts = tuple(
            len(re.findall(r"\bmtDenseMTaskVertices\b", branch))
            for branch in (owner, identity)
        )
        if successor_counts == (1, 1) and storage_counts == (0, 0):
            categories.append("successors")
        elif storage_counts == (1, 1) and successor_counts == (0, 0):
            categories.append("storage")
        else:
            fail(f"{header_path}: compile block {index} is not paired successors or storage")
    if sorted(categories) != ["storage", "successors"]:
        fail(f"{header_path}: wrong compile block categories {categories}")


def validate_wait_blocks(
    blocks: Sequence[Tuple[str, str]], expected_count: int, worker_path: Path
) -> None:
    if len(blocks) != expected_count:
        fail(
            f"{worker_path}: found {len(blocks)} compile wait blocks, "
            f"expected {expected_count}"
        )
    for index, branches in enumerate(blocks):
        for label, branch in zip(("owner-bank", "identity"), branches):
            masked = mask_cpp_noncode(branch)
            predicates = list(WAIT_PREDICATE_RE.finditer(masked))
            vertex_refs = len(re.findall(r"\bmtDenseMTaskVertices\s*\[", masked))
            load_count = len(ANY_WAIT_LOAD_RE.findall(masked))
            while_count = len(re.findall(r"\bwhile\s*\(", masked))
            if len(predicates) != 1 or vertex_refs != 1 or load_count != 1 or while_count != 1:
                fail(
                    f"{worker_path}: wait block {index} {label} branch must contain "
                    "one complete literal-slot acquire predicate != target"
                )
            if SIGNAL_FETCH_RE.search(masked) or RUNTIME_MAPPING_RE.search(masked):
                fail(f"{worker_path}: wait block {index} contains signal or mapping code")


def validate_legacy_parent(
    parent_dir: Path, dep_counts: Sequence[int], offsets: Sequence[int],
    logical_successors: Sequence[int], owners: Dict[int, int],
    groups: Dict[int, List[int]], signal_sources: Dict[int, int],
    wait_ids: Set[int],
) -> None:
    header_path, header_text = unique_header(parent_dir, "legacy parent")
    if OWNER_COMPILE_MACRO in header_text:
        fail(f"{header_path}: legacy parent contains the compile-exclusive macro")
    header = mask_cpp_noncode(header_text)
    parent_dep = parse_array(header, "kDenseMTaskDepCount", "legacy parent")
    parent_offsets = parse_array(header, "kDenseMTaskSuccOffsets", "legacy parent")
    parent_stored = parse_array(header, "kDenseMTaskSuccList", "legacy parent")
    parent_successors = successor_entries(parent_offsets, parent_stored, "legacy parent")
    require_equal(parent_dep, dep_counts, "legacy dependency counts", "parent", "enabled identity")
    require_equal(parent_offsets, offsets, "legacy successor offsets", "parent", "enabled identity")
    require_equal(parent_successors, logical_successors, "legacy successors", "parent", "enabled identity")
    storage_count, aligned = parse_storage(header, "legacy parent")
    if aligned or storage_count != len(dep_counts):
        fail(f"{header_path}: legacy storage is not unaligned logical-size storage")
    if RUNTIME_MAPPING_RE.search(header):
        fail(f"{header_path}: legacy parent contains owner-bank diagnostics")
    worker_path, source = find_worker_source(parent_dir, "legacy parent")
    worker = extract_worker_definition(source, worker_path, "legacy parent")
    reject_runtime_mapping(source, worker_path)
    cases = extract_worker_cases(worker, worker_path)
    parent_owners, parent_groups = ownership_from_cases(cases, len(dep_counts), worker_path)
    if parent_owners != owners or parent_groups != groups:
        fail("legacy parent and enabled model fixed worker ownership differ")
    parent_wait_ids = parse_waits(
        cases, parent_dep, list(range(len(dep_counts))), worker_path
    )
    if parent_wait_ids != wait_ids:
        fail("legacy parent and enabled identity wait sets differ")
    parent_signals = parse_signal_sources(cases, parent_offsets, parent_owners, worker_path)
    if parent_signals != signal_sources:
        fail("legacy parent and enabled model signal sources differ")
    validate_sync_emission_mode(
        parent_wait_ids, parent_signals, parent_dep, parent_offsets, "legacy parent"
    )


def validate_enabled_model(model_dir: Path, parent_dir: Optional[Path]) -> None:
    header_path, header_text = unique_header(model_dir, "enabled model")
    if OWNER_COMPILE_MACRO not in header_text:
        fail(f"{header_path}: enabled model lacks {OWNER_COMPILE_MACRO}")
    header_nondirective = re.sub(
        r"(?m)^[ \t]*#(?:if|elif|else|endif)[^\r\n]*$", "", header_text
    )
    if OWNER_COMPILE_MACRO in header_nondirective:
        fail(f"{header_path}: runtime/non-directive compile-macro use")
    if re.search(
        r"\bgetenv\s*\([^)]*GSIM_MT_DENSE_OWNER_BANK_COUNTERS",
        header_nondirective,
        re.DOTALL,
    ):
        fail(f"{header_path}: generated runtime owner-bank environment lookup")
    blocks = compile_blocks(header_text)
    classify_header_blocks(blocks, header_path)
    owner_text, owner_count = select_compile_variant(header_text, "owner", str(header_path))
    identity_text, identity_count = select_compile_variant(
        header_text, "identity", str(header_path)
    )
    if owner_count != 2 or identity_count != 2:
        fail(f"{header_path}: incomplete compile-exclusive header alternatives")
    owner_header = mask_cpp_noncode(owner_text)
    identity_header = mask_cpp_noncode(identity_text)
    owner_dep = parse_array(owner_header, "kDenseMTaskDepCount", "owner-bank branch")
    identity_dep = parse_array(identity_header, "kDenseMTaskDepCount", "identity branch")
    owner_offsets = parse_array(owner_header, "kDenseMTaskSuccOffsets", "owner-bank branch")
    identity_offsets = parse_array(identity_header, "kDenseMTaskSuccOffsets", "identity branch")
    require_equal(owner_dep, identity_dep, "dependency counts changed", "owner-bank", "identity")
    require_equal(owner_offsets, identity_offsets, "successor offsets changed", "owner-bank", "identity")
    if not identity_dep:
        fail(f"{header_path}: empty kDenseMTaskDepCount")
    mtask_count = len(identity_dep)
    identity_stored = parse_array(identity_header, "kDenseMTaskSuccList", "identity branch")
    owner_stored = parse_array(owner_header, "kDenseMTaskSuccList", "owner-bank branch")
    logical_successors = successor_entries(identity_offsets, identity_stored, "identity branch")
    successor_entries(owner_offsets, owner_stored, "owner-bank branch")
    validate_graph(identity_dep, identity_offsets, logical_successors, "identity branch")
    for label, header in (("owner-bank", owner_header), ("identity", identity_header)):
        struct_count = len(re.findall(r"\bstruct\s+MtDenseMTaskVertex\s*\{", header))
        if struct_count != 1:
            fail(f"{header_path}: {label} branch has {struct_count} vertex structs")

    worker_path, worker_source = find_worker_source(model_dir, "enabled model")
    worker_blocks = compile_blocks(worker_source)
    owner_source, owner_worker_count = select_compile_variant(
        worker_source, "owner", str(worker_path)
    )
    identity_source, identity_worker_count = select_compile_variant(
        worker_source, "identity", str(worker_path)
    )
    if owner_worker_count != identity_worker_count:
        fail(f"{worker_path}: compile variants select different wait-block counts")
    owner_worker = extract_worker_definition(owner_source, worker_path, "owner-bank branch")
    identity_worker = extract_worker_definition(identity_source, worker_path, "identity branch")
    reject_runtime_mapping(worker_source, worker_path)
    owner_cases = extract_worker_cases(owner_worker, worker_path)
    identity_cases = extract_worker_cases(identity_worker, worker_path)
    owners, groups = ownership_from_cases(owner_cases, mtask_count, worker_path)
    identity_owners, identity_groups = ownership_from_cases(
        identity_cases, mtask_count, worker_path
    )
    if owners != identity_owners or groups != identity_groups:
        fail(f"{worker_path}: fixed worker ownership changes across compile variants")
    expected_slots, physical_count = expected_owner_bank_map(
        owners, groups, mtask_count
    )
    expected_owner_stored = (
        [expected_slots[logical_id] for logical_id in logical_successors]
        if logical_successors else [0]
    )
    require_equal(
        owner_stored, expected_owner_stored, "owner-bank successor translation",
        "generated", "mapped identity successors",
    )
    validate_bank_storage(owner_header, header_path, physical_count)
    validate_identity_storage(identity_header, header_path, mtask_count)
    validate_diagnostics(
        owner_header, expected_slots,
        [owners[logical_id] for logical_id in range(mtask_count)],
        physical_count, header_path,
    )
    owner_wait_ids = parse_waits(
        owner_cases, owner_dep, expected_slots, worker_path
    )
    identity_wait_ids = parse_waits(
        identity_cases, identity_dep, list(range(mtask_count)), worker_path
    )
    if owner_wait_ids != identity_wait_ids:
        fail(f"{worker_path}: wait set changes across compile variants")
    validate_wait_blocks(worker_blocks, len(owner_wait_ids), worker_path)
    owner_signals = parse_signal_sources(
        owner_cases, owner_offsets, owners, worker_path
    )
    identity_signals = parse_signal_sources(
        identity_cases, identity_offsets, identity_owners, worker_path
    )
    if owner_signals != identity_signals:
        fail(f"{worker_path}: signal sources change across compile variants")
    sync_mode = validate_sync_emission_mode(
        identity_wait_ids, identity_signals, identity_dep, identity_offsets,
        "compile variants",
    )
    if parent_dir is not None:
        validate_legacy_parent(
            parent_dir, identity_dep, identity_offsets, logical_successors,
            owners, groups, identity_signals, identity_wait_ids,
        )
    padding = physical_count - mtask_count
    growth = physical_count / mtask_count
    owner_count = sum(bool(logical_ids) for logical_ids in groups.values())
    print(
        f"mt-dense-owner-bank ok: sync={sync_mode} mtasks={mtask_count} owners={owner_count} "
        f"slots={physical_count} padding={padding} growth={growth:.2f}x "
        f"edges={len(logical_successors)} signal-sources={len(owner_signals)} "
        f"waits={len(owner_wait_ids)} mixed-owner-lines=0"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Check v280 compile-exclusive dense owner-bank generated synchronization."
    )
    parser.add_argument(
        "enabled_model_dir", type=Path,
        help="generated model directory emitted with owner banking enabled",
    )
    parser.add_argument(
        "legacy_parent_dir", type=Path, nargs="?",
        help="optional knob-off generated parent directory",
    )
    args = parser.parse_args()
    if not args.enabled_model_dir.is_dir():
        fail(f"enabled model directory does not exist: {args.enabled_model_dir}")
    if args.legacy_parent_dir is not None and not args.legacy_parent_dir.is_dir():
        fail(f"legacy parent directory does not exist: {args.legacy_parent_dir}")
    validate_enabled_model(args.enabled_model_dir, args.legacy_parent_dir)


if __name__ == "__main__":
    main()
