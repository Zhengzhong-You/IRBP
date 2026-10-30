#!/usr/bin/env python3
"""Check published benchmark integrity without importing an optimizer."""
import ast
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def check_repository():
    with (ROOT / "instances/manifest.csv").open(newline="", encoding="utf-8") as stream:
        records = list(csv.DictReader(stream))
    require(records, "Instance manifest is empty")
    expected = {record["file"] for record in records}
    require(len(expected) == len(records), "Duplicate instance in manifest")
    actual = {path.relative_to(ROOT).as_posix() for group in ("small", "larger")
              for path in (ROOT / "instances" / group).glob("*.json")}
    require(actual == expected, f"Manifest mismatch: missing={expected - actual}, extra={actual - expected}")
    for record in records:
        raw = (ROOT / record["file"]).read_bytes()
        require(hashlib.sha256(raw).hexdigest() == record["sha256"],
                f"Instance hash mismatch: {record['file']}")
        instance = json.loads(raw)
        dimensions = {key: len(instance[key]) for key in ("nodes", "arcs", "blocks", "demands")}
        dimensions["links"] = len(instance["link_capacities"])
        dimensions["yards"] = sum(bool(node["yard"]) for node in instance["nodes"])
        for key, value in dimensions.items():
            require(value == int(record[key]), f"{record['file']}: incorrect {key}")

    spec = json.loads((ROOT / "experiments/configs/variants.json").read_text())
    base = json.loads((ROOT / spec["base"]).read_text())
    for name, overrides in spec["variants"].items():
        expected_config = {**base, **spec["common"], **overrides}
        output = subprocess.check_output(
            [sys.executable, str(ROOT / "experiments/make_config.py"), name], cwd=ROOT, text=True)
        require(json.loads(output) == expected_config, f"Configuration generation failed: {name}")

    tracked = subprocess.check_output(["git", "ls-files", "-z"], cwd=ROOT).decode().split("\0")
    for filename in filter(None, tracked):
        path = ROOT / filename
        if path.suffix == ".json":
            json.loads(path.read_text(encoding="utf-8"))
        elif path.suffix == ".py":
            ast.parse(path.read_text(encoding="utf-8"), filename=filename)
        elif path.suffix == ".csv":
            with path.open(newline="", encoding="utf-8-sig") as stream:
                reader = csv.DictReader(stream, strict=True)
                require(reader.fieldnames and len(set(reader.fieldnames)) == len(reader.fieldnames),
                        f"Invalid CSV header: {filename}")
                for line, row in enumerate(reader, start=2):
                    require(None not in row and all(value is not None for value in row.values()),
                            f"Invalid CSV row width: {filename}:{line}")
    print(f"PASS: {len(records)} instance hashes/dimensions, {len(spec['variants'])} configurations, "
          "and tracked JSON/CSV/Python files. No optimization was run.")


if __name__ == "__main__":
    try:
        check_repository()
    except (ValueError, KeyError, OSError, SyntaxError, subprocess.CalledProcessError) as error:
        sys.exit(f"Repository check failed: {error}")
