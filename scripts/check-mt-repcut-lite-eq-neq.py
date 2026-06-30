#!/usr/bin/env python3
import json
import re
import sys
from pathlib import Path


def fail(msg: str) -> None:
    raise SystemExit(f"mt-repcut-lite-eq-neq failed: {msg}")


def extract_function(text: str, name: str) -> str:
    match = re.search(rf"void\s+\w+::{re.escape(name)}\s*\([^)]*\)\s*\{{", text)
    if not match:
        fail(f"missing helper {name}")
    brace = text.find("{", match.end() - 1)
    depth = 0
    for idx in range(brace, len(text)):
        ch = text[idx]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return text[match.start():idx + 1]
    fail(f"unterminated helper {name}")


def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: check-mt-repcut-lite-eq-neq.py <model_dir>")
    model_dir = Path(sys.argv[1])
    reports = sorted(model_dir.glob("*_mt_repcut_lite.json"))
    if len(reports) != 1:
        fail(f"expected one report, found {len(reports)} under {model_dir}")
    report = json.loads(reports[0].read_text())

    reasons = {}
    parallel_batches = []
    for batch in report.get("cut_batches", []):
        for reason, count in batch.get("clone_fallback_reasons", {}).items():
            reasons[reason] = reasons.get(reason, 0) + count
        if batch.get("parallel_safe"):
            parallel_batches.append(batch)
    if reasons:
        fail(f"unexpected clone fallback reasons for unsigned EQ/NEQ fixture: {reasons}")
    if len(parallel_batches) != 1:
        fail(f"expected one parallel-safe batch, got {parallel_batches}")
    if not any(task.get("runtime_applied") for task in report.get("tasks", [])):
        fail("expected runtime_applied sink for unsigned EQ/NEQ cut")

    clones = report.get("duplicated_nodes", [])
    source_nodes = {clone.get("source_node") for clone in clones}
    if "eq_hit" not in source_nodes:
        fail(f"expected eq_hit clone to exercise OP_EQ support, got {clones}")
    if "neq_hit" not in source_nodes:
        fail(f"expected neq_hit clone to exercise OP_NEQ support, got {clones}")

    text = "\n".join(path.read_text(errors="ignore") for path in sorted(model_dir.glob("*.cpp")))
    for clone in clones:
        source_node = clone.get("source_node")
        if source_node not in {"eq_hit", "neq_hit"}:
            continue
        helper_name = f"mtRepCutLiteTask{clone.get('sink_cpp_id')}"
        helper = extract_function(text, helper_name)
        clone_name = re.escape(clone.get("clone_name", ""))
        decl = re.search(rf"uint\w+_t\s+{clone_name}\s*=\s*([^;]+);", helper)
        if not decl:
            fail(f"missing clone declaration for {source_node} in {helper_name}")
        rhs = decl.group(1)
        if source_node == "eq_hit" and " == " not in rhs:
            fail(f"eq_hit clone does not contain == expression: {rhs}")
        if source_node == "neq_hit" and " != " not in rhs:
            fail(f"neq_hit clone does not contain != expression: {rhs}")
        if "0h" in rhs:
            fail(f"FIRRTL radix literal leaked into C++ clone expression: {rhs}")

    print(f"mt-repcut-lite-eq-neq ok: clones={sorted(source_nodes)} parallel_safe={len(parallel_batches)} from {reports[0]}")


if __name__ == "__main__":
    main()
