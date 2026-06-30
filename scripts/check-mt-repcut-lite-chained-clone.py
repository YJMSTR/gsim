#!/usr/bin/env python3
import json
import re
import sys
from pathlib import Path


def fail(msg: str) -> None:
    raise SystemExit(f"mt-repcut-lite-chained-clone failed: {msg}")


def extract_function(text: str, name: str) -> str:
    match = re.search(rf"void\s+\w+::{re.escape(name)}\s*\([^)]*\)\s*\{{", text)
    if not match:
        fail(f"missing helper {name}")
    start = match.start()
    brace = text.find("{", match.end() - 1)
    depth = 0
    for idx in range(brace, len(text)):
        ch = text[idx]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return text[start:idx + 1]
    fail(f"unterminated helper {name}")


def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: check-mt-repcut-lite-chained-clone.py <model_dir>")
    model_dir = Path(sys.argv[1])
    reports = sorted(model_dir.glob("*_mt_repcut_lite.json"))
    if len(reports) != 1:
        fail(f"expected one report, found {len(reports)} under {model_dir}")
    report = json.loads(reports[0].read_text())
    parallel_batches = [b for b in report.get("cut_batches", []) if b.get("parallel_safe")]
    if len(parallel_batches) != 1:
        fail(f"expected one parallel-safe batch, found {len(parallel_batches)}")
    clones = report.get("duplicated_nodes", [])
    clones_by_sink = {}
    for clone in clones:
        clones_by_sink.setdefault(clone.get("sink_cpp_id"), {})[clone.get("source_node")] = clone
    sink_cpp_id = None
    clone_by_source = None
    for candidate_sink, candidate_clones in clones_by_sink.items():
        if "base" in candidate_clones and "derived" in candidate_clones:
            sink_cpp_id = candidate_sink
            clone_by_source = candidate_clones
            break
    if sink_cpp_id is None or clone_by_source is None:
        fail(f"expected one sink with base and derived clones, got {clones}")
    helper_name = f"mtRepCutLiteTask{sink_cpp_id}"
    text = "\n".join(path.read_text(errors="ignore") for path in sorted(model_dir.glob("*.cpp")))
    helper = extract_function(text, helper_name)
    base_name = clone_by_source["base"].get("clone_name")
    derived_name = clone_by_source["derived"].get("clone_name")
    base_decl = re.search(rf"uint\w+_t\s+{re.escape(base_name)}\s*=\s*([^;]+);", helper)
    derived_decl = re.search(rf"uint\w+_t\s+{re.escape(derived_name)}\s*=\s*([^;]+);", helper)
    if not base_decl:
        fail(f"missing base clone declaration {base_name} in helper")
    if not derived_decl:
        fail(f"missing derived clone declaration {derived_name} in helper")
    if base_decl.start() > derived_decl.start():
        fail("derived clone declared before its base clone")
    if base_name not in derived_decl.group(1):
        fail(f"derived clone RHS does not use earlier clone {base_name}: {derived_decl.group(1)}")
    print(f"mt-repcut-lite-chained-clone ok: helper={helper_name} base={base_name} derived={derived_name} from {reports[0]}")


if __name__ == "__main__":
    main()
