# Gurobi reports a wrong optimum on n100-s2026102348

On the N100 instance generated with seed 2026102348 (`instances/small/n100-s2026102348.json`), Gurobi 12.0.3 solves the ARC MIP and reports an optimal solution of cost 12,625,591.91 with best bound equal to this cost, after exploring only the root node. A feasible plan found by DG-CGM and validated on the original model costs 12,405,275.34, which is 1.7% lower; `witness_solution.json` gives the values of the 260,960 ARC variables of this plan, in the column order of the model. The paper therefore does not count this instance as solved to optimality.

- `n100-s2026102348.mps.gz` is the ARC model of this instance exported by `irbp_exact`. Its Gurobi model fingerprint, 0xed4b31dc, equals that of the model built in memory during the benchmark runs (553,278 rows, 260,960 columns, 1,983,928 nonzeros). Fixing every variable to the values in `witness_solution.json` gives a feasible solution of cost 12,405,275.34.
- `runs.csv` and `logs/` contain the benchmark run (default parameters) and the runs that change one Gurobi setting at a time. The wrong claim occurs in every run with default parameters. It does not occur when `Aggregate`, `Presolve`, `Symmetry`, or `NumericFocus` is changed, when all cuts are disabled, or when any one of the eight cut classes in `runs.csv` is disabled; these runs end at their time limit with a bound below 12,405,275.34.

Reproduce the default run with eight threads and the benchmark parameters:

```sh
./build/exact/irbp_exact instances/small/n100-s2026102348.json arc 7200 result.json 8 0
gurobi_cl Threads=8 Seed=1 Cutoff=46075811.467817314 FeasibilityTol=1e-8 IntFeasTol=1e-8 MIPGap=0 MIPGapAbs=1e-7 OptimalityTol=1e-9 BarConvTol=1e-12 Crossover=0 TimeLimit=7200 n100-s2026102348.mps.gz
```

The parameter changes of the other runs were applied through a `gurobi.env` file in the working directory, for example `Aggregate 0`. The runs used Gurobi 12.0.3 on Windows with an AMD EPYC 9654 processor.
