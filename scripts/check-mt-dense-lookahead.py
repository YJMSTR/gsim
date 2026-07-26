#!/usr/bin/env python3
"""Independently validate B1 constrained dense-table lookahead metadata."""

import argparse
import json
import re
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Set, Tuple

CHECKER = "check-mt-dense-lookahead"


def fail(message: str) -> None:
    raise SystemExit(f"{CHECKER} failed: {message}")


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError as error:
        fail(f"could not read {path}: {error}")
    raise AssertionError("unreachable")


def unique_path(paths: Iterable[Path], label: str) -> Path:
    found = sorted(paths)
    if len(found) != 1:
        fail(f"expected one {label}, found {len(found)} ({', '.join(path.name for path in found) or 'none'})")
    return found[0]


def parse_array(header: str, ctype: str, name: str) -> List[int]:
    match = re.search(rf"static constexpr {re.escape(ctype)} {re.escape(name)}\[(\d+)\] = \{{([^}}]*)\}};", header, re.DOTALL)
    if match is None:
        fail(f"missing {ctype} array {name}")
    count = int(match.group(1))
    contents = match.group(2).strip()
    values = [] if not contents else [int(token.strip().rstrip("uU")) for token in contents.split(",")]
    if len(values) != count:
        fail(f"{name}: declared {count} values, parsed {len(values)}")
    return values


def find_matching_brace(text: str, opening: int, context: str) -> int:
    if opening < 0 or opening >= len(text) or text[opening] != "{":
        fail(f"internal parse error: {context} does not start with '{{'")
    depth = 0
    for index in range(opening, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return index
    fail(f"unterminated {context}")
    raise AssertionError("unreachable")


def parse_tables(source: str) -> Tuple[Dict[int, int], Dict[int, int], Dict[int, Tuple[int, int, int, int, int, int]]]:
    owners: Dict[int, int] = {}
    positions: Dict[int, int] = {}
    entries_by_gid: Dict[int, Tuple[int, int, int, int, int, int]] = {}
    table_re = re.compile(r"kDenseDispatchTableW(\d+)\[(\d+)\] = \{")
    for match in table_re.finditer(source):
        worker = int(match.group(1))
        declared_count = int(match.group(2))
        opening = source.find("{", match.start())
        body = source[opening + 1:find_matching_brace(source, opening, f"worker {worker} table")]
        rows = re.findall(r"\{([^{}]*)\}", body)
        real = [row for row in rows if "stepDenseMTask" in row]
        if not real:
            if declared_count != 1 or len(rows) != 1 or "nullptr" not in rows[0]:
                fail(f"worker {worker}: malformed padded empty dispatch table")
            continue
        if len(real) != declared_count:
            fail(f"worker {worker}: declared {declared_count} entries, parsed {len(real)}")
        for position, row in enumerate(real):
            gid_match = re.search(r"stepDenseMTask(\d+)", row)
            if gid_match is None:
                fail(f"worker {worker} position {position}: missing MTask method")
            gid = int(gid_match.group(1))
            values = [int(value) for value in re.findall(r"(\d+)u", row)]
            if len(values) != 6:
                fail(f"worker {worker} position {position}: expected six spans, got {len(values)}")
            if gid in owners:
                fail(f"global MTask {gid} occurs in multiple dispatch tables")
            owners[gid] = worker
            positions[gid] = position
            entries_by_gid[gid] = tuple(values)  # wait, store, local spans
    if not owners:
        fail("no lookahead dispatch entries found")
    return owners, positions, entries_by_gid


def dag_only_transitive_reduction(succs: Sequence[Set[int]]) -> List[List[int]]:
    count = len(succs)
    reachable: List[int] = [0] * count
    for source in range(count - 1, -1, -1):
        for successor in sorted(succs[source]):
            if successor <= source or successor >= count:
                fail(f"schedule DAG has non-forward edge {source}->{successor}")
            reachable[source] |= (1 << successor) | reachable[successor]
    kept: List[List[int]] = [[] for _ in range(count)]
    for source in range(count):
        for successor in sorted(succs[source]):
            if not any(alternative != successor and (reachable[alternative] & (1 << successor)) for alternative in succs[source]):
                kept[source].append(successor)
    return kept


def reconstruct_dag(schedule: dict, gids: Set[int]) -> List[Set[int]]:
    task_mtask = {int(task["cpp_id"]): int(task["dense_mtask_id"]) for task in schedule.get("tasks", [])}
    if not task_mtask:
        fail("schedule JSON has no dense_mtask_id task mapping")
    count = max(gids) + 1
    if gids != set(range(count)):
        fail(f"dispatch table gids are not a complete 0..{count - 1} range")
    scc_mtask: Dict[int, int] = {}
    scc_succs: Dict[int, List[int]] = {}
    for scc in schedule.get("sccs", []):
        scc_id = int(scc["scc_id"])
        memberships = {task_mtask.get(int(cpp_id), -1) for cpp_id in scc.get("cpp_ids", [])}
        if len(memberships) != 1 or -1 in memberships:
            fail(f"SCC {scc_id} does not map to one MTask: {sorted(memberships)}")
        scc_mtask[scc_id] = next(iter(memberships))
        scc_succs[scc_id] = [int(successor) for successor in scc.get("succ_sccs", [])]
    if not scc_mtask:
        fail("schedule JSON has no SCC DAG")
    succs: List[Set[int]] = [set() for _ in range(count)]
    for source_scc, successors in scc_succs.items():
        source = scc_mtask[source_scc]
        for successor_scc in successors:
            if successor_scc not in scc_mtask:
                fail(f"SCC {source_scc} references missing successor SCC {successor_scc}")
            successor = scc_mtask[successor_scc]
            if source != successor:
                succs[source].add(successor)
    return succs


def expected_groups(kept: Sequence[Sequence[int]], owners: Dict[int, int], positions: Dict[int, int]) -> Tuple[List[Tuple[int, int, List[int]]], List[Set[int]], List[Set[int]]]:
    count = len(kept)
    grouped: Dict[Tuple[int, int], List[int]] = {}
    direct: List[Set[int]] = [set() for _ in range(count)]
    for source, successors in enumerate(kept):
        for destination in successors:
            if owners[source] == owners[destination]:
                if positions[source] >= positions[destination]:
                    fail(f"same-worker reduced edge {source}->{destination} is not forward in table order")
                direct[destination].add(positions[source])
            else:
                grouped.setdefault((destination, owners[source]), []).append(source)
    groups = [(destination, producer_owner, sources) for (destination, producer_owner), sources in sorted(grouped.items())]
    siblings: List[Set[int]] = [set() for _ in range(count)]
    for destination, producer_owner, sources in groups:
        if sources != sorted(sources, key=lambda source: positions[source]):
            fail(f"group ({destination}, {producer_owner}) sources are not fixed-worker ordered")
        publisher = sources[-1]
        for source in sources[:-1]:
            if owners[source] != producer_owner or positions[source] >= positions[publisher]:
                fail(f"group publisher {publisher} does not follow sibling {source}")
            siblings[publisher].add(positions[source])
    return groups, direct, siblings


def expected_slots(groups: Sequence[Tuple[int, int, List[int]]], owners: Dict[int, int]) -> List[int]:
    pairs: Dict[Tuple[int, int], List[int]] = {}
    for index, (destination, producer_owner, _) in enumerate(groups):
        pairs.setdefault((producer_owner, owners[destination]), []).append(index)
    slots = [-1] * len(groups)
    next_slot = 0
    for pair, group_ids in sorted(pairs.items()):
        next_slot = (next_slot + 63) & ~63
        for group_id in group_ids:
            slots[group_id] = next_slot
            next_slot += 1
        next_slot = (next_slot + 63) & ~63
    if any(slot < 0 for slot in slots):
        fail("internal expected slot construction failed")
    return slots


def extract_tail_body(source: str) -> str:
    match = re.search(r"::stepDenseLookaheadTail\([^\n]*\) \{", source)
    if match is None:
        fail("generated source lacks stepDenseLookaheadTail")
    opening = source.find("{", match.start())
    return source[opening + 1:find_matching_brace(source, opening, "stepDenseLookaheadTail")]


def verify_shape(header: str, source: str) -> None:
    tail = extract_tail_body(source)
    if "uint32_t startHead" not in source or "uint32_t head = startHead;" not in tail:
        fail("cold helper does not preserve global table coordinates from startHead")
    if "uint64_t mtDenseDoneBits[kDenseLookaheadDoneWordCount] = {};" not in tail:
        fail("cold helper lacks full-table done bitmap")
    if "kDenseLookaheadWindow" not in tail:
        fail("cold helper lacks bounded window scan")
    if "mtDenseCandidate->localBegin" not in tail or "mtDenseCandidate->localEnd" not in tail:
        fail("cold helper does not check local prerequisites")
    if "mtDenseCandidate->waitBegin" not in tail or "mtDenseCandidate->waitEnd" not in tail:
        fail("cold helper does not check candidate owner-ready tokens")
    forbidden = ("mtDenseEpochs", "uint32_t(cycles) + 1", "mtDenseDeferred", "kDenseLookaheadCrossPreds")
    if any(token in source or token in header for token in forbidden):
        fail("B7 epoch/deferred-publication remnants remain in emitted model")
    worker_match = re.search(r"::stepDenseThreadWorker\(int threadId\) \{", source)
    if worker_match is None:
        fail("generated source lacks fixed worker function")
    worker_open = source.find("{", worker_match.start())
    worker = source[worker_open + 1:find_matching_brace(source, worker_open, "stepDenseThreadWorker")]
    if "mtDenseInlineReady" not in worker or "stepDenseLookaheadTail(kDenseDispatchTableW" not in worker:
        fail("inline hot path lacks nonblocking readiness and cold-tail handoff")
    if not re.search(r"stepDenseMTask\d+\(\);", worker):
        fail("inline hot path lacks direct MTask calls")
    tail_start = source.find(tail)
    for invocation in re.finditer(r"\(this->\*mtDense(?:DispatchEntry|Candidate)->fn\)\(\);", source):
        if not (tail_start <= invocation.start() < tail_start + len(tail)):
            fail("member-function dispatch appears outside the cold helper")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model_dir", type=Path, help="directory generated with GSIM_MT_DENSE_LOOKAHEAD")
    parser.add_argument("schedule_json", type=Path, nargs="?", help="generated *_mt_dense_schedule.json (auto-discovered when omitted)")
    args = parser.parse_args()
    model_dir = args.model_dir
    header_path = unique_path(model_dir.glob("*.h"), "generated header")
    source_paths = sorted(path for path in model_dir.glob("*.cpp") if "stepDenseLookaheadTail" in read_text(path))
    if not source_paths:
        fail("expected at least one lookahead source, found none")
    schedule_path = args.schedule_json or unique_path(model_dir.glob("*_mt_dense_schedule.json"), "dense schedule JSON")
    header = read_text(header_path)
    # The helper definition, dispatch tables, and worker call sites may span multiple
    # source files depending on the schedule's file split; validate their union.
    source = "\n".join(read_text(path) for path in source_paths)
    try:
        schedule = json.loads(read_text(schedule_path))
    except json.JSONDecodeError as error:
        fail(f"invalid schedule JSON {schedule_path}: {error}")

    local_prereqs = parse_array(header, "uint32_t", "kDenseLookaheadLocalPrereqs")
    local_kinds = parse_array(header, "uint8_t", "kDenseLookaheadLocalPrereqKinds")
    group_destination = parse_array(header, "uint32_t", "kDenseLookaheadGroupDestination")
    group_owner = parse_array(header, "uint32_t", "kDenseLookaheadGroupProducerOwner")
    group_publisher = parse_array(header, "uint32_t", "kDenseLookaheadGroupPublisher")
    group_offsets = parse_array(header, "uint32_t", "kDenseLookaheadGroupSourceOffsets")
    group_sources = parse_array(header, "uint32_t", "kDenseLookaheadGroupSources")
    wait_list = parse_array(header, "int", "kDenseOwnerReadyWaitList")
    store_offsets = parse_array(header, "int", "kDenseOwnerReadyStoreOffsets")
    store_list = parse_array(header, "int", "kDenseOwnerReadyStoreList")
    owners, positions, entries = parse_tables(source)

    kept = dag_only_transitive_reduction(reconstruct_dag(schedule, set(owners)))
    groups, direct, siblings = expected_groups(kept, owners, positions)
    slots = expected_slots(groups, owners)
    if len(group_destination) == 1 and not groups and group_destination == [0]:
        group_destination = group_owner = group_publisher = []
    if len(group_sources) == 1 and not groups and group_sources == [0]:
        group_sources = []
    if len(group_destination) != len(groups) or len(group_owner) != len(groups) or len(group_publisher) != len(groups):
        fail("emitted group metadata length does not match recomputed groups")
    if len(group_offsets) != len(groups) + 1:
        fail("emitted group source offsets do not cover all groups")
    for index, (destination, producer_owner, sources) in enumerate(groups):
        if (group_destination[index], group_owner[index], group_publisher[index]) != (destination, producer_owner, sources[-1]):
            fail(f"group {index}: destination/owner/publisher metadata differs from recomputation")
        begin, end = group_offsets[index:index + 2]
        if not (0 <= begin <= end <= len(group_sources)) or group_sources[begin:end] != sources:
            fail(f"group {index}: source membership differs from recomputation")

    expected_waits: List[List[int]] = [[] for _ in entries]
    expected_stores: List[List[int]] = [[] for _ in entries]
    for (destination, _, sources), slot in zip(groups, slots):
        expected_waits[destination].append(slot)
        expected_stores[sources[-1]].append(slot)
    for values in expected_waits + expected_stores:
        values.sort()
    table_order = [gid for _, gid in sorted((owners[gid], gid) for gid in owners)]
    flattened_waits = [slot for gid in table_order for slot in expected_waits[gid]]
    if wait_list[:len(flattened_waits)] != flattened_waits:
        fail("emitted owner-ready wait list differs from recomputed B1 layout")
    flattened_stores = [slot for gid in range(len(entries)) for slot in expected_stores[gid]]
    if store_list[:len(flattened_stores)] != flattened_stores:
        fail("emitted owner-ready store list differs from recomputed B1 layout")

    if len(local_kinds) != len(local_prereqs):
        fail("local prerequisite kind array length differs from prerequisite array")
    for gid in range(len(entries)):
        wait_begin, wait_end, store_begin, store_end, local_begin, local_end = entries[gid]
        if wait_list[wait_begin:wait_end] != expected_waits[gid]:
            fail(f"MTask {gid}: table wait span differs from recomputed layout")
        if store_list[store_begin:store_end] != expected_stores[gid]:
            fail(f"MTask {gid}: table store span differs from recomputed layout")
        if not (0 <= local_begin <= local_end <= len(local_prereqs)):
            fail(f"MTask {gid}: malformed local prerequisite span")
        actual_positions = local_prereqs[local_begin:local_end]
        actual_kinds = local_kinds[local_begin:local_end]
        expected_positions = sorted(direct[gid] | siblings[gid])
        expected_kinds = [
            (1 if position in direct[gid] else 0) | (2 if position in siblings[gid] else 0)
            for position in expected_positions
        ]
        if actual_positions != expected_positions or actual_kinds != expected_kinds:
            fail(f"MTask {gid}: sorted local prerequisites/categories differ from recomputation")
        if any(position >= positions[gid] for position in actual_positions):
            fail(f"MTask {gid}: local prerequisite is not earlier in the worker table")
        if store_offsets[gid:gid + 2] != [store_begin, store_end]:
            fail(f"MTask {gid}: store offsets do not match dispatch span")
    expected_sibling_positions = sum(map(len, siblings))
    emitted_sibling_positions = sum(1 for kind in local_kinds if kind & 2)
    if expected_sibling_positions > 0 and emitted_sibling_positions == 0:
        fail("publisher siblings exist in the DAG-only token groups but no kind-2 local prerequisite was emitted")
    if emitted_sibling_positions != expected_sibling_positions:
        fail(f"emitted kind-2 local prerequisites {emitted_sibling_positions} != recomputed publisher siblings {expected_sibling_positions}")
    publisher_constrained_entries = sum(1 for values in siblings if values)

    verify_shape(header, source)
    print(f"{CHECKER} PASS: mtasks={len(entries)} dag_only_kept_edges={sum(map(len, kept))} groups={len(groups)} cross_edges={sum(len(group[2]) for group in groups)} same_worker_preds={sum(map(len, direct))} publisher_constrained_entries={publisher_constrained_entries} sibling_positions={expected_sibling_positions}")


if __name__ == "__main__":
    main()
