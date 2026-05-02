# Distributed SLAM Dataset Publishability Checklist

## Purpose

This checklist defines the minimum bar for turning the current AGV collection system into a publishable distributed SLAM dataset.

It is written for this project specifically:

- multiple AGVs
- ROS Melodic
- wheel odometry
- 2D LiDAR
- RGB-D camera with IMU
- optional UWB / inter-robot ranging

This document is not a guarantee of publication. It is a quality-control framework to reduce the most common reasons datasets are rejected or ignored.

## Release Goal

The dataset should be:

- technically correct
- reproducible
- well-calibrated
- well-documented
- diverse in scenarios
- useful for benchmarking distributed and collaborative SLAM

## 1. Sensor Data Requirements

## Mandatory

- wheel odometry
- 2D LiDAR scans
- TF tree
- robot command stream such as `cmd_vel`
- RGB images
- depth images
- IMU

## Strongly Recommended

- UWB ranging
- communication logs between robots
- diagnostics topics
- battery / motor / health status

## Essential For High-Impact Publication

- reliable ground truth trajectories
- calibration files for every sensor
- synchronization documentation
- dataset metadata per run

## Pass Criteria

- every public run contains the same mandatory topic set
- message definitions are included or reproducible from source
- each sensor topic has documented units, frames, and expected rate

## 2. Dataset Organization

Use a strict, machine-readable release layout.

```text
dataset/
  scenarios/
    <scenario_id>/
      description.md
      map_sketch.png
      environment.json
      runs/
        <run_id>/
          metadata.json
          qa_report.json
          robots/
            <robot_id>/
              rosbag/
              calibration/
              notes.md
```

## Required Metadata Per Run

- run ID
- scenario ID
- robot ID
- date and time
- operator
- environment type
- sensor list
- topic list
- firmware / software commit or version
- calibration version
- clock sync method
- known issues

## Pass Criteria

- no ambiguous file names
- no manual renaming needed after collection
- every bag is traceable to one robot, one run, and one scenario

## 3. Calibration Requirements

## Required Calibration Artifacts

- camera intrinsics
- depth-to-color alignment parameters
- LiDAR-to-base extrinsics
- camera-to-base extrinsics
- IMU-to-base extrinsics
- temporal offsets between sensors
- UWB anchor calibration if used

## Required Process

- calibrate before the main collection campaign
- repeat calibration after hardware changes
- version all calibration files
- record calibration date and operator
- store calibration residual / fit quality

## Pass Criteria

- all transforms used in TF match stored calibration files
- calibration files are shipped with the dataset
- there is a documented procedure to reproduce calibration

## 4. Sensor Quality Targets

These are practical targets for publishable quality, not immutable laws.

## LiDAR

- stable rate, typically `10-15 Hz` for 2D LiDAR
- low dropout rate
- correct frame assignment
- no persistent clipping from bad mounting

## RGB

- stable `15-30 FPS`
- low frame drops
- controlled exposure where possible
- motion blur acceptable only if documented

## Depth

- stable `15-30 FPS`
- valid depth in the intended operating range
- synchronized with RGB well enough for fusion or clearly documented if not

## IMU

- stable `100-200 Hz` minimum
- timestamp monotonicity
- no large burst gaps
- bias stability measured or at least characterized

## Odometry

- stable and documented update rate
- consistent frame conventions
- no sign or axis ambiguity

## TF

- complete tree for every run
- static transforms documented
- no broken links during collection

## UWB

- documented ranging rate
- packet loss measured
- anchor layout documented
- range noise characterized

## Global Timing

- ideal: hardware synchronization
- acceptable: measured and documented software synchronization with known error bounds

## Pass Criteria

- per-run QA report includes topic frequencies, message counts, and dropout summaries
- bad runs are flagged and excluded from the benchmark split

## 5. Ground Truth Requirements

Without strong ground truth, your dataset will be much less valuable.

## Acceptable Ground Truth Options

- indoor motion capture
- total station
- AprilTag / fiducial ground truth in calibrated spaces
- RTK-GPS for outdoor scenarios
- carefully validated hybrid setup if no single method covers all cases

## Requirements

- define ground truth accuracy bounds
- define failure conditions
- define coordinate frames and alignment
- release the alignment method

## Pass Criteria

- every benchmark split has an associated ground truth source
- any sequence without trustworthy ground truth is labeled clearly as qualitative-only

## 6. Scenario Design Requirements

Do not collect random runs. Use a scenario matrix.

## Required Scenario Types

- high-overlap collaboration
- low-overlap collaboration
- delayed rendezvous
- re-entry after separation
- loop closure rich routes
- long corridors
- multi-room indoor spaces
- dynamic obstacles / people
- texture-poor areas
- communication degradation or partial loss
- asynchronous robot start times
- robot dropout or failure cases

## Coverage Requirements

- short runs
- medium runs
- long runs
- repeated runs of the same scenario
- both easy and hard variants

## Pass Criteria

- scenarios are named, described, and intentionally designed
- each scenario tests a distinct algorithmic capability

## 7. Duration and Volume Targets

## Per Run

- pilot tests: `2-5 min`
- standard benchmark runs: `5-10 min`
- long-term drift or persistence runs: `15-30 min`

## Overall Dataset

- enough runs to support training, validation, and benchmark splits
- enough repeated runs for statistical comparison
- enough diversity to avoid all sequences looking like minor variants of the same path

## Practical Target

- at least `30-50` meaningful runs if you want the dataset to be taken seriously
- several hours of usable data after QA filtering

## 8. Benchmark Utility Requirements

The dataset should not just exist. It must support evaluation.

## Required Benchmark Tasks

- trajectory estimation
- map consistency
- loop closure detection
- inter-robot place recognition
- collaborative localization
- performance under low overlap
- performance under limited communication

## Required Metrics

- trajectory error against ground truth
- map quality or consistency metrics
- loop closure precision / recall if annotated
- inter-robot localization success rate
- communication cost if studying distributed methods
- runtime / bandwidth / compute cost where relevant

## Pass Criteria

- baseline methods can be run on released splits
- evaluation scripts and metrics are documented

## 9. Communication and Distributed SLAM Requirements

For a distributed SLAM dataset, communication is not optional context.

## Collect If Possible

- inter-robot message logs
- bandwidth usage
- packet delays
- packet loss
- availability windows

## Pass Criteria

- the dataset either includes communication logs or states clearly that it is only a multi-robot sensing dataset, not a full communication benchmark

## 10. Documentation Requirements

## Must Ship

- dataset paper or technical report
- sensor specification table
- calibration guide
- collection guide
- folder structure guide
- topic dictionary
- frame diagram
- known limitations
- license and citation instructions

## Pass Criteria

- another lab can understand the dataset without reverse-engineering your code

## 11. Quality Assurance Workflow

Every run should pass a QA pipeline.

## Required Checks

- bag opens correctly
- all required topics exist
- frequencies are within expected range
- timestamps are monotonic
- TF tree is valid
- calibration files match the run configuration
- run notes are present
- ground truth availability is recorded

## Outputs

- `qa_report.json`
- pass / fail flag
- issue list

## 12. Project-Specific Next Actions

For this project, the most important next actions are:

1. Finalize the real sensor suite on the AGV.
2. Make `agv_ws/src/agv_bringup/launch/logging.launch` the single source of truth for collection.
3. Add camera, IMU, UWB, and diagnostics topics to logging once verified.
4. Define a metadata schema and write it on every run.
5. Establish a calibration and synchronization SOP.
6. Build a scenario matrix before large-scale collection starts.
7. Add QA scripts before recording a large number of runs.
8. Define benchmark splits and baseline methods early.

## Publication Readiness Gate

Do not call the dataset release-ready until all of the following are true:

- sensing stack is stable
- calibration is versioned and documented
- synchronization error is measured
- ground truth is available for benchmark runs
- scenario matrix is complete
- QA pipeline is running
- metadata is written per run
- benchmark tasks and metrics are defined
- release documentation is complete

## Bottom Line

If you meet this checklist, your dataset will be technically serious and scientifically defensible.

If you miss synchronization, calibration, ground truth, or QA, the dataset may still be usable internally, but it will not look like a strong publishable benchmark.
