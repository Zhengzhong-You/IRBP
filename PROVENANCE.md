# Provenance

The computational study of the paper ran several builds of two code trees. This repository merges them into one tree without changing any optimization code path that the experiments used.

- `heuristic/src/local_search.cpp` (SHA-256 `3b6411a6...`) is the DG-CGM kernel. It is the production kernel (`9c98556e...`) plus the component switches and counters of the generated-instance harness (`d9b27da7...`) and the two pool-only switches (`benchmark_pool_only_joint`, `benchmark_pool_only_local`, both off by default). Every experiment ran one of these four kernels, and under its configuration each of them executes the same search as this file.
- `exact/` is the source of the DW LP benchmark build. `exact/src/arc.cpp` adds the opt-in incumbent-trajectory observer (`IRBP_ARC_TRAJECTORY`) and is identical to the source of the ARC MIP benchmark build. The ARC LP build differs from it only in output fields.
- `third_party/routeopt` keeps one copy of the RouteOpt solver interface, the newer of the two versions used, which only adds functions to the older one.
- Files that lacked a copyright header received one. Test paths were adapted to the repository layout. The component test now checks the experiment semantics of `NoJoint`, which skips the whole enrichment and joint-repair call. Two unit tests of `exact/` that also fail on the original source were removed (`master_test`, `model_test`).
- `generator/instance_generator.py` differs from the generator used for the instances (`a67dba31...`) only in its docstring and the default `--reference` path.
- Every configuration used in the paper is reproduced by `experiments/configs/variants.json`.
