# Experiments

All runs of Section 6 use the executables built from this repository and the configurations in `configs/variants.json`. A variant is the default configuration `heuristic/config/local_search.json` plus `benchmark_record_initial_pool = true` (which only records the initial candidate pool in the result) plus the overrides listed for the variant:

```sh
python experiments/make_config.py Full > Full.json
python experiments/make_config.py NoPG > NoPG.json
```

## Command lines

| Executable | Arguments |
|---|---|
| `irbp_exact` | `INSTANCE arc-lp 7200 RESULT.json 8 0` (ARC LP) |
| | `INSTANCE arc 7200 RESULT.json 8 0` (ARC MIP; set `IRBP_ARC_TRAJECTORY=TRAJECTORY.json` to record the incumbent trajectory) |
| | `INSTANCE bpc 7200 RESULT.json 8 0 --pricing-workers 8 --root-only --dw-root --dual-after-columns` (DW LP) |
| `ts_harness` | `INSTANCE ARM SECONDS RESULT.json CONFIG.json 1 C0.json\|-` with `ARM` in `ConstructionOnly`, `Full`, `NoPG`, `NoLocal`, `NoJoint`, `FullNoImplicit`. The component switches in `CONFIG.json` must match the arm. The optional `C0.json` is a construction snapshot that the harness compares with its own construction. |
| `post_c0` | `INSTANCE capture SECONDS C0.json CONFIG.json ALPHA` writes the construction snapshot C0; `INSTANCE continue SECONDS RESULT.json CONFIG.json ALPHA C0.json` restores C0 and runs the search for `SECONDS` |
| `pg_criteria` | `INSTANCE C0.json Current\|Travel\|EUC RESULT.json CONFIG.json 1` |
| `irbp` | `INSTANCE heuristic 7200 RESULT.json --config CONFIG.json --threads 8 --demand-multiplier ALPHA` |

All generated-instance runs of DG-CGM use one solver thread. The ARC and DW benchmarks use eight threads and a two-hour limit. The paper runs all jobs on one workstation and starts a new job only when at most 185 of its 384 logical processors are busy; the LP times of the bound table are measured separately with at most four concurrent runs.

## Paper sections

| Section 6 result | Runs | Results |
|---|---|---|
| LP bounds of ARC and DW, small instances | `irbp_exact` ARC LP and DW LP on `instances/small` | `results/small/bounds_*.csv` |
| ARC MIP and DG-CGM, incumbent cost over time | `irbp_exact` ARC MIP; `ts_harness ... Full 60 ...` with variant `Full` | `results/small/arc_mip.csv`, `arc_mip_trajectories.json`, `dgcgm_components.csv`, `reference_costs.csv` |
| Search components (60 s) | `ts_harness` with arms `ConstructionOnly`, `Full`, `NoPG`, `NoLocal`, `NoJoint` (variants of the same names; `ConstructionOnly` uses `Full`) | `results/small/dgcgm_components.csv` |
| Time limits 10, 30, 60, 120, 300 s | `ts_harness` with arms `Full`, `NoPG`, `NoLocal`, `NoJoint` | `results/small/dgcgm_time_limits.csv` |
| Joint repair restricted to the incumbent paths and the shared pool | `ts_harness` arm `Full` with variant `PoolJ_Full`, arm `NoPG` with `PoolJ_NoPG` | `results/small/dgcgm_pool_only_joint.csv` |
| Value of dual guidance (path-generation criteria) | `post_c0 ... capture 60 ...` with variant `Full`, then `pg_criteria` with variant `PGCriteria` | `results/small/pg_criteria.csv` |
| Robustness: seeds 2 to 5 | `ts_harness ... Full 60 ...` with variants `seed2` to `seed5` | `results/small/dgcgm_seeds.csv` |
| Sensitivity to search controls (electronic companion) | `ts_harness ... Full 60 ...` with variants `NLP2` ... `EPSH1e-5` | `results/small/dgcgm_sensitivity.csv` |
| Larger generated instances | `ts_harness` with a 300 s limit (arms `Full`, `NoPG`, `NoLocal`, `NoJoint`, `ConstructionOnly`; seeds 1 to 3 for `Full`); `irbp_exact` DW LP, ARC LP, ARC MIP; pool-only variants | `results/larger/*.csv` |
| RAS: construction and complete search, demand scenarios, cost components | `irbp` with variants `RAS`, `RAS_seed2`, `RAS_seed3` and `ALPHA` in 0.5, 1, 2 | `results/ras/dgcgm_main.csv` |
| RAS: search components | `post_c0 ... capture`, then `post_c0 ... continue` from the same C0 (time limit as in Section 6.4.2) with variants `RAS`, `RAS_NoPG`, `RAS_NoLocal`, `RAS_NoJoint`, `RAS_PoolJ_Full`, `RAS_PoolJ_NoPG` | `results/ras/dgcgm_components.csv`, `dgcgm_pool_only_joint.csv` |
| RAS: resource limits and the joint-repair cap | `irbp` on separately prepared resource-scenario inputs; variants `RAS_joint100`, `RAS_joint500` (see scope below) | `results/ras/resource_limits.csv` |

## RAS data and reproduction scope

The RAS inputs are converted from the competition data set, which is not redistributed. The commands above require compatible converted inputs obtained separately. The resource-scenario preparation script used for `results/ras/resource_limits.csv` is not included in the public tree. Consequently, the resource-scenario study cannot yet be reproduced from this repository alone, even with the raw competition data. Its archived CSV results remain available for inspection.

The supplied generated JSON instances and experiment configurations are available directly in this repository. Use the published manifest to identify the input, and record the source commit, solver version, threads, time limit, and generated configuration with any rerun. Raw wall-clock timings depend on the machine and workload; preserve the measured values when comparing runs.
