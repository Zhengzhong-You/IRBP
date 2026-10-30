# Contributing to IRBP

Thank you for helping improve IRBP. Questions, bug reports, documentation corrections, and algorithmic contributions are welcome through [Issues](https://github.com/Zhengzhong-You/IRBP/issues/new/choose) and pull requests.

## Report a reproducible issue

Include the commit, operating system, compiler, Gurobi version, exact command, configuration, and a small instance that reproduces the behavior. Describe the expected result and what happened. For numerical discrepancies, provide the feasible objective, solver status, relevant bounds, and validation output.

Use a generated or synthetic example when the original input cannot be shared. Do not upload solver license files, credentials, or restricted competition data.

## Prepare a change

1. Start from the current `main` branch and keep the change focused on one issue.
2. Follow the surrounding C++17 or Python style. Preserve existing copyright and third-party notices.
3. For an algorithm or modeling change, add a small regression case that demonstrates the changed behavior. Keep optimizer calls behind the RouteOpt solver interface.
4. Document any changed command, option, constraint, or result field in the relevant guide.

For algorithm comparisons, use matching inputs, initial solutions, candidate pools, thread counts, and time limits where applicable. Report both improvements and regressions. Keep a new experiment's results separate from the published CSV files; explain its setup and validation in the pull request.

## Validate locally

These repository checks need Python 3.11 or later and no Gurobi license:

```sh
python3 scripts/check_repository.py
cmake -DSOURCE_ROOT=heuristic -P cmake/solver_boundary_test.cmake
cmake -DSOURCE_ROOT=exact -P cmake/solver_boundary_test.cmake
```

For solver changes, build and test with your separately licensed Gurobi installation:

```sh
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release -j 8
ctest --test-dir build-release --output-on-failure
```

With Visual Studio, add `--config Release` when building and `-C Release` when running CTest. See the [README](README.md#build-and-test) for dependencies.

In the pull request, state which checks ran and their results. If you could not run a solver-dependent check, state that explicitly. Repository integrity checks do not substitute for feasibility validation or benchmark comparisons.

## Data, results, and licensing

The [instance manifest](instances/manifest.csv) records the released benchmark hashes. Do not silently replace those inputs or edit historical outcomes to match new code. Document intentional dataset changes and update their manifests together.

Changes to this project follow its existing [GPL-3.0 license](LICENSE). Preserve the licenses and attribution of bundled third-party components, as described in [NOTICE](NOTICE). The authors and order of the research citation are specified separately in [CITATION.cff](CITATION.cff).
