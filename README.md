# IRBP: integrated railroad blocking

Code, instances, and computational results for

> Yida Xu, Zhengzhong Ricky You, and Baichuan Mo. *A Dual-Guided Column-Generation-Based Matheuristic for Integrated Railroad Blocking.* Submitted to Transportation Science, 2026.

The repository contains the dual-guided column-generation-based matheuristic (DG-CGM), the benchmark methods of the paper (ARC LP, ARC MIP, and the Dantzig-Wolfe LP bound), the 190 generated instances, the experiment drivers and configurations, and the per-run results behind every table and figure of Section 6.

| Directory | Contents |
|---|---|
| `heuristic/` | DG-CGM (`irbp`): construction, destroy and local repair, path generation, joint repair, and an independent plan validator |
| `exact/` | `irbp_exact`: the compact ARC formulation (LP and MIP) and the DW LP solved by column generation |
| `experiments/` | Drivers for the component, time-limit, seed, sensitivity, path-generation, and RAS studies, and the configuration of every experiment |
| `instances/` | The 150 small and 40 larger generated instances, their manifest, and the seed sampling record |
| `generator/` | The instance generator and conversion chain |
| `results/` | Per-run results, one CSV file per experiment |
| `gurobi-issue/` | A reproducible case in which Gurobi 12.0.3 reports an optimal solution that a feasible plan contradicts |
| `examples/` | Small instances used by the tests and the quick start |
| `third_party/` | RouteOpt solver interface, nlohmann/json, and miniz |

## Requirements

- CMake 3.20 or later and a C++17 compiler. The paper uses Microsoft Visual C++ 14.44 (Visual Studio 2022); the code also builds with Clang and GCC on macOS and Linux.
- Gurobi 12 or 13 with a valid license. The paper uses Gurobi 12.0.3. Only `third_party/routeopt` calls the Gurobi C API.
- Boost headers for `irbp_exact` (for example `brew install boost` or `apt install libboost-dev`). Configure with `-DIRBP_BUILD_EXACT=OFF` to build DG-CGM without Boost.
- Python 3.11 for the scripts and tests.

## Build and test

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 8
ctest --test-dir build --output-on-failure
```

Set `GUROBI_HOME`, or pass `-DGUROBI_INCLUDE_DIR=... -DGUROBI_LIBRARY=...`, if CMake does not find Gurobi. With Visual Studio, add `--config Release` to the build and test commands. The executables are `build/heuristic/irbp`, `build/exact/irbp_exact`, and the drivers in `build/experiments/`.

## Quick start

```sh
# DG-CGM with the default controls (Table EC.1) and a 60-second limit
./build/heuristic/irbp instances/small/n50-s2026102304.json heuristic 60 dgcgm.json --config heuristic/config/local_search.json --threads 1

# ARC LP, ARC MIP, and DW LP with eight threads and a two-hour limit
./build/exact/irbp_exact instances/small/n50-s2026102304.json arc-lp 7200 arc_lp.json 8 0
./build/exact/irbp_exact instances/small/n50-s2026102304.json arc 7200 arc_mip.json 8 0
./build/exact/irbp_exact instances/small/n50-s2026102304.json bpc 7200 dw_lp.json 8 0 --pricing-workers 8 --root-only --dw-root --dual-after-columns
```

Every returned plan is validated against the original model, and the result file records its cost components, service, and resource use. The time limit includes input and *construction*. `heuristic/config/README.md` explains each control.

## Reproducing the paper

`experiments/README.md` lists, for every table and figure, the executable, the arguments, and the configuration variant. `python experiments/make_config.py VARIANT` writes the configuration of a variant. The per-run results are in `results/`.

The RAS scenarios are built from the data set of the 2026 INFORMS Railway Applications Section Problem Solving Competition. The competition rules do not allow redistribution, so the repository contains neither these data nor the RAS input files; with access to the data, the commands in `experiments/README.md` reproduce the RAS runs.

## License

Copyright (c) 2026 Zhengzhong (Ricky) You. Released under the GNU General Public License v3.0; see `LICENSE` and `NOTICE`. Gurobi is commercial software and is not distributed here.

## Citation

Please cite the paper above. `CITATION.cff` contains the metadata.
