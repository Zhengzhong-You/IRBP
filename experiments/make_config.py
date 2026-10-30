#!/usr/bin/env python3
# Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
# Software: RouteOpt. Application: IRBP.
"""Write the DG-CGM configuration of one experiment variant.

Every configuration used in the paper is the default configuration plus the overrides listed in
experiments/configs/variants.json. Example: python experiments/make_config.py NoPG > NoPG.json
"""
import json
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[1]
spec = json.loads((root / "experiments/configs/variants.json").read_text())
if len(sys.argv) != 2 or sys.argv[1] not in spec["variants"]:
    sys.exit("usage: make_config.py VARIANT\nvariants: " + ", ".join(spec["variants"]))
config = json.loads((root / spec["base"]).read_text())
config.update(spec["common"])
config.update(spec["variants"][sys.argv[1]])
print(json.dumps(config, indent=2))
