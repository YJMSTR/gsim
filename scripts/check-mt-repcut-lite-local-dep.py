#!/usr/bin/env python3
import json
import re
import sys
from pathlib import Path


def fail(msg: str) -> None:
    raise SystemExit(f"mt-repcut-lite-local-dep failed: {msg}")



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


def has_bare_name(expr: str, name: str) -> bool:
    return re.search(rf"(?<![A-Za-z0-9_]){re.escape(name)}(?![A-Za-z0-9_])", expr) is not None

def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: check-mt-repcut-lite-local-dep.py <model_dir>")
    model_dir = Path(sys.argv[1])
    reports = sorted(model_dir.glob("*_mt_repcut_lite.json"))
    if len(reports) != 1:
        fail(f"expected one report, found {len(reports)} under {model_dir}")
    report = json.loads(reports[0].read_text())
    reasons = {}
    for batch in report.get("cut_batches", []):
        for reason, count in batch.get("clone_fallback_reasons", {}).items():
            reasons[reason] = reasons.get(reason, 0) + count
    if reasons.get("local_expr_dependency", 0) != 0:
        fail(f"local closure still reports local_expr_dependency; reasons={reasons}")
    parallel_safe = [b for b in report.get("cut_batches", []) if b.get("parallel_safe")]
    if len(parallel_safe) != 1:
        fail(f"expected one parallel-safe batch, found {len(parallel_safe)}")
    runtime_tasks = [task for task in report.get("tasks", []) if task.get("runtime_applied")]
    if not runtime_tasks:
        fail("expected runtime_applied sink after local closure")
    clones = report.get("duplicated_nodes", [])
    source_clone = next((clone for clone in clones if clone.get("source_node") == "source_value"), None)
    if source_clone is None:
        fail(f"missing source_value clone in duplicated_nodes: {clones}")
    sink_cpp_id = source_clone.get("sink_cpp_id")
    source_clone_name = source_clone.get("clone_name")
    if sink_cpp_id is None or not source_clone_name:
        fail(f"malformed source_value clone entry: {source_clone}")
    helper_name = f"mtRepCutLiteTask{sink_cpp_id}"
    text = "\n".join(path.read_text(errors="ignore") for path in sorted(model_dir.glob("*.cpp")))
    helper = extract_function(text, helper_name)
    local_clone_name = f"{source_clone_name}_local_a_wide"
    local_decl = re.search(rf"uint\w+_t\s+{re.escape(local_clone_name)}\s*=\s*([^;]+);", helper)
    source_decl = re.search(rf"uint\w+_t\s+{re.escape(source_clone_name)}\s*=\s*([^;]+);", helper)
    if not local_decl:
        fail(f"missing local a_wide clone declaration {local_clone_name} in {helper_name}")
    if not source_decl:
        fail(f"missing source clone declaration {source_clone_name} in {helper_name}")
    if local_decl.start() > source_decl.start():
        fail("source clone declared before its local a_wide clone")
    source_rhs = source_decl.group(1)
    if local_clone_name not in source_rhs:
        fail(f"source clone RHS does not use {local_clone_name}: {source_rhs}")
    if has_bare_name(source_rhs, "a_wide"):
        fail(f"source clone RHS still references bare a_wide: {source_rhs}")
    print(f"mt-repcut-lite-local-dep ok: helper={helper_name} source={source_clone_name} local={local_clone_name} from {reports[0]}")


if __name__ == "__main__":
    main()
