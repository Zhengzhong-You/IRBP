# IRBP · Integrated Railroad Blocking

[![Repository checks](https://github.com/Zhengzhong-You/IRBP/actions/workflows/repository-checks.yml/badge.svg)](https://github.com/Zhengzhong-You/IRBP/actions/workflows/repository-checks.yml)
[![License: GPL v3](https://img.shields.io/badge/License-GPL_v3-blue.svg)](LICENSE)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C.svg)](CMakeLists.txt)
[![Instances: 190](https://img.shields.io/badge/Generated_instances-190-476A6F.svg)](instances/README.md)

Research software for jointly choosing freight service, railroad blocks, and physical routes under minimum block loads and shared network capacities.

**[Quick start](#quick-start)** · **[Experiments](experiments/README.md)** · **[Instances](instances/README.md)** · **[Results](results/README.md)** · **[Contributing](CONTRIBUTING.md)** · **[Citation](#citation)**

This repository accompanies the manuscript

> Yida Xu, Zhengzhong Ricky You, and Baichuan Mo. *A Dual-Guided Column-Generation-Based Matheuristic for Integrated Railroad Blocking.* Submitted to Transportation Science, 2026.

It contains the dual-guided column-generation-based matheuristic (DG-CGM), the ARC and Dantzig–Wolfe benchmark implementations, 190 generated instances, experiment configurations, and per-run computational results. The manuscript is submitted work; no publication DOI is currently listed.

## What is included

| Method or artifact | Purpose |
|---|---|
| **DG-CGM** | Feasible solutions through construction, local repair, dual-guided path generation, and joint repair |
| **ARC LP / ARC MIP** | Continuous-relaxation bounds and direct mixed-integer optimization of the compact formulation |
| **DW LP** | Root LP bounds from the Dantzig–Wolfe formulation and logical/physical column generation |
| **190 generated instances** | 150 small and 40 larger instances, with a SHA-256 manifest and sampling records |
| **Experiment records** | Per-run cost, runtime, bound, and service information; component and sensitivity comparisons |

DG-CGM first constructs a feasible plan, then reoptimizes selected demands while accounting for all outside resource use. When local improvement slows, LP dual prices guide additional path generation. Joint repair coordinates a larger selected demand set before returning to local search. Candidate generation is heuristic; an integer optimum over a restricted pool is not a proof of global optimality. See the [configuration guide](heuristic/config/README.md) and [source provenance](PROVENANCE.md) for implementation details.

## Repository layout

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
- Python 3.11 for the scripts and tests. Reading the supplied results and running the repository integrity checks require no Gurobi installation.

The instance generator has additional Python dependencies listed in its [guide](generator/README.md). A separate Gurobi installation and valid license are required to build and run the optimization executables.

## Build and test

```sh
git clone https://github.com/Zhengzhong-You/IRBP.git
cd IRBP
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release -j 8
ctest --test-dir build-release --output-on-failure
```

Set `GUROBI_HOME` to the platform directory containing `include/` and `lib/`, or pass `-DGUROBI_INCLUDE_DIR=... -DGUROBI_LIBRARY=...` if CMake does not find Gurobi. Use a fresh build directory when changing the compiler, solver installation, or repository layout.

With Visual Studio, use `cmake --build build-release --config Release` and `ctest --test-dir build-release -C Release --output-on-failure`. Executables are placed under `build-release/heuristic/Release/` and `build-release/exact/Release/` and have the `.exe` extension. On a single-configuration Unix build, the corresponding directories are `build-release/heuristic/` and `build-release/exact/`.

## Quick start

```sh
# A small example with DG-CGM
./build-release/heuristic/irbp examples/partial_service.json heuristic 5 example-result.json --config heuristic/config/local_search.json --threads 1

# A generated instance with the default controls and a 60-second limit
./build-release/heuristic/irbp instances/small/n50-s2026102304.json heuristic 60 dgcgm.json --config heuristic/config/local_search.json --threads 1

# ARC LP, ARC MIP, and DW LP with eight threads and a two-hour limit
./build-release/exact/irbp_exact instances/small/n50-s2026102304.json arc-lp 7200 arc_lp.json 8 0
./build-release/exact/irbp_exact instances/small/n50-s2026102304.json arc 7200 arc_mip.json 8 0
./build-release/exact/irbp_exact instances/small/n50-s2026102304.json bpc 7200 dw_lp.json 8 0 --pricing-workers 8 --root-only --dw-root --dual-after-columns
```

Every returned plan is validated against the original model, and the result file records its cost components, service, and resource use. The time limit includes input and *construction*. The [configuration guide](heuristic/config/README.md) explains each control.

## Reproducing the paper

[The experiment guide](experiments/README.md) maps the reported comparisons to executables, arguments, and configuration variants. `python experiments/make_config.py VARIANT` writes a variant configuration. The [result guide](results/README.md) defines the reporting fields and explains the distinction between proven optima and feasible reference costs.

The RAS scenarios use the data set of the 2026 INFORMS Railway Applications Section Problem Solving Competition. The competition rules do not allow redistribution, so the repository contains neither the raw data nor the converted RAS inputs. RAS runs require separately prepared, compatible input files. The resource-scenario preparation script used in the experiments is not included in this release; that part of the pipeline is documented as an outstanding reproduction dependency in the [experiment guide](experiments/README.md#ras-data-and-reproduction-scope).

For reproducible comparisons, record the commit, configuration, solver version, thread counts, time limit, and instance hash. [PROVENANCE.md](PROVENANCE.md) describes the relationship between the published source and the experimental builds. The [Gurobi case study](gurobi-issue/README.md) documents an optimality-status discrepancy observed on one instance.

## Repository checks

```sh
python3 scripts/check_repository.py
```

The [GitHub Actions workflow](https://github.com/Zhengzhong-You/IRBP/actions/workflows/repository-checks.yml) checks the instance manifest, experiment configurations, JSON/CSV files, Python syntax, and solver-interface boundaries without requiring a commercial solver license. It does not run optimization benchmarks or certify the recorded numerical results. Run the CTest suite above in an environment with Gurobi to test solver behavior.

## Questions and contributions

Use [GitHub Issues](https://github.com/Zhengzhong-You/IRBP/issues/new/choose) for reproducible bugs, questions, and proposed improvements. Include the instance, exact command, configuration, and solver version when reporting numerical behavior. The [contribution guide](CONTRIBUTING.md) describes validation expectations and how to keep benchmark evidence reproducible.

## License

Copyright (c) 2026 Zhengzhong (Ricky) You. Released under the GNU General Public License v3.0; see [LICENSE](LICENSE) and [NOTICE](NOTICE). Third-party components retain their respective licenses. Gurobi is commercial software and is not distributed here.

## Citation

Please cite the manuscript above and record the repository commit used in your experiments. [CITATION.cff](CITATION.cff) supplies GitHub's **Cite this repository** metadata, including the manuscript's submitted status.
