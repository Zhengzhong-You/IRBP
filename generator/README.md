# Instance generator

`instance_generator.py` draws a network and demands from the distributions of data set L1 of the 2026 INFORMS RAS Problem Solving Competition (Section EC.3 of the paper) and writes `node.csv`, `link.csv`, `demand.csv`, and `setting.csv`. `convert_mixed_generated.py` converts the CSV files with `heuristic/scripts/import_gencat.py`, and `expand_legacy_input.py` writes the instance JSON read by all executables.

```sh
python generator/instance_generator.py --reference PATH/TO/L1 --output raw --nodes 100 --demands 12 --seeds 2026102348
```

Python 3.11 with numpy 1.26.4, scipy 1.11.4, pandas 2.2.3, networkx 3.4.2, NEMtropy 4.0.0, numba 0.61.2, llvmlite 0.44.0, powerlaw 1.5, bicm 3.4.0, and threadpoolctl. The seeds and the retained draws of every instance set are listed in `instances/sampling.csv`. The competition data are not redistributed.
