# Benchmarking Code Push Guide

This repository contains both source code and local experiment artefacts. Only
the source code, report material, robot bringup configuration, and benchmark
scripts should be pushed to the main repository.

## Push

- `scripts/benchmarking`: trajectory evaluation, GT alignment, report figure generation.
- `scripts/orbslam3`, `scripts/rtabmap`, `scripts/slam_toolbox`, `scripts/cartographer`, `scripts/swarmslam`, `scripts/covins`, `scripts/ptam`: benchmark runners and analysis wrappers.
- `scripts/logging`, `scripts/scenarios`, `scripts/diagnostics`: robot-side collection, scenario, and diagnostic helpers.
- `agv_ws/src/agv_bringup`: robot bringup, logging launch files, calibration metadata, and sensor configuration used by the dataset.
- `Report`: report source, release-facing calibration document, references, and curated figures.
- `docs` and `configs`: project documentation and reusable local configuration.

## Do Not Push

- Raw robot bags: `downloaded_bags/`, `agv_data/`, `*.bag`.
- Public datasets or downloaded validation data: `external_datasets/`.
- Generated benchmark outputs: `report_results/`, `fleet_logs/`, result databases, maps, and large archives.
- External algorithm checkouts: `Algorithms/`, `Swarm-SLAM/`, `Distributed_SLAM/`, `external/`.
- Compiled helpers and build outputs: `build/`, `devel/`, `install/`, `scripts/orbslam3/bin/`.

Those paths are ignored by `.gitignore` so that a normal `git add` does not
accidentally include large local artefacts.

## Recommended Staging Flow

Preview what would be staged:

```bash
bash scripts/benchmarking/stage_benchmark_code.sh
```

Stage the benchmark/report code:

```bash
bash scripts/benchmarking/stage_benchmark_code.sh --stage
```

Then inspect the diff before committing:

```bash
git status --short
git diff --cached --stat
git diff --cached
```

Commit when the staged diff contains only the intended code and report files:

```bash
git commit -m "Add ORKAR benchmarking pipeline"
```

## Existing Dirty State

This cleanup does not revert existing tracked edits or deletions. In
particular, tracked `offline_swarmslam/` deletions and robot bringup changes
should be reviewed deliberately before committing.
