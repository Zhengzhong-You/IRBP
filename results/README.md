# Results

One row per run. Costs are in the units of the instance, and times are wall-clock seconds. Every plan was replayed on the original model (`replay_objective`, `max_row_violation`).

| Folder | Files |
|---|---|
| `small/` | `bounds_arc_lp.csv`, `bounds_dw_lp.csv` (two-hour runs), `bounds_timing_low_load.csv` (the LP times reported in the paper), `arc_mip.csv`, `arc_mip_trajectories.json`, `dgcgm_components.csv`, `dgcgm_time_limits.csv`, `dgcgm_seeds.csv`, `dgcgm_sensitivity.csv`, `dgcgm_pool_only_joint.csv`, `pg_criteria.csv`, `reference_costs.csv` |
| `larger/` | `dgcgm.csv`, `dgcgm_pool_only_joint.csv`, `dw_lp.csv`, `arc_lp.csv`, `arc_mip.csv` |
| `ras/` | `dgcgm_main.csv`, `dgcgm_components.csv`, `dgcgm_pool_only_joint.csv`, `resource_limits.csv` |

Common columns: `case` (instance), `arm` or `variant` (DG-CGM variant), `seed`, `seconds` (time limit), `objective` (final cost), `C0_objective` (cost after *construction*), `lower_bound`, `status`, `native_seconds` (wall-clock time of the process), and the cost, service, and resource-use columns of the final plan.

`reference_costs.csv` gives the reference cost `z_ref` of each small instance. It is the ARC MIP optimum when Gurobi reports optimality and no run found a cheaper feasible plan, and otherwise the best feasible cost found by any run. On `n100-s2026102348` Gurobi reports optimality although DG-CGM found a cheaper feasible plan, so this instance is not counted as solved to optimality (see `gurobi-issue/`).
