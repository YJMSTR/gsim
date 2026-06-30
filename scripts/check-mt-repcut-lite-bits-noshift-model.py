#!/usr/bin/env python3
import json
import sys
from pathlib import Path


def fail(msg: str) -> None:
    raise SystemExit(f"mt-repcut-lite-bits-noshift-model failed: {msg}")


def load_after_split(model_dir: Path) -> tuple[Path, dict]:
    dumps = sorted(model_dir.glob("*AfterSplitNodes.json"))
    if len(dumps) != 1:
        fail(f"expected one *AfterSplitNodes.json dump, found {len(dumps)} under {model_dir}")
    dump = json.loads(dumps[0].read_text())
    return dumps[0], dump


def check_emitted_mid_expression(model_dir: Path) -> str:
    text = "\n".join(path.read_text(errors="ignore") for path in sorted(model_dir.glob("*.cpp")))
    matches = []
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("mid = ") and stripped.endswith(";"):
            matches.append(stripped[len("mid = "):-1])
    if not matches:
        fail(f"missing emitted mid assignment in generated C++ under {model_dir}")
    good = []
    for expr in matches:
        compact = " ".join(expr.split())
        if ("src$15_8 << 2" in compact and
                "& (0xff >> 2 << 2)" in compact and
                "| (src$7_0 >> 6)" in compact):
            good.append(compact)
    if not good:
        fail(f"emitted mid expression does not align split high/low slices correctly: {matches[:3]}")
    return good[0]


def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: check-mt-repcut-lite-bits-noshift-model.py <model_dir>")
    model_dir = Path(sys.argv[1])
    dump_path, dump = load_after_split(model_dir)

    nodes = dump.get("nodes")
    if not isinstance(nodes, list):
        fail(f"malformed dump: missing nodes array in {dump_path}")

    split_src_nodes = {node.get("name") for node in nodes if str(node.get("name", "")).startswith("src$")}
    if {"src$15_8", "src$7_0"} - split_src_nodes:
        fail(f"src did not split into expected high/low nodes; saw {sorted(split_src_nodes)}")

    noshift_examples = []
    any_assign_trees = False
    for node in nodes:
        if "assignTrees" not in node:
            fail("dump lacks assignTrees; rerun gsim with --dump-assign-tree")
        trees = node.get("assignTrees")
        if not isinstance(trees, list):
            fail(f"node {node.get('name')} has malformed assignTrees")
        any_assign_trees = any_assign_trees or bool(trees)
        for tree in trees:
            for enode in tree.get("nodes", []):
                if enode.get("op") == "OP_BITS_NOSHIFT":
                    noshift_examples.append((node.get("name"), enode))

    if not any_assign_trees:
        fail("dump has no assignTrees payload; rerun gsim with --dump-assign-tree")
    if not noshift_examples:
        fail(f"src split exists but no OP_BITS_NOSHIFT appears in assignTrees from {dump_path}")

    mid_examples = [example for example in noshift_examples if example[0] == "mid"]
    if not mid_examples:
        fail(f"expected cross-boundary mid consumer to contain OP_BITS_NOSHIFT, got {noshift_examples[:5]}")

    emitted_mid = check_emitted_mid_expression(model_dir)

    print(
        "mt-repcut-lite-bits-noshift-model ok: "
        f"src_split={sorted(split_src_nodes)} noshift_examples={len(noshift_examples)} "
        f"emitted_mid={emitted_mid} from {dump_path}"
    )


if __name__ == "__main__":
    main()
