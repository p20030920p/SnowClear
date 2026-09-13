# Changelog

All notable changes to this project are documented here.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added

- **`tools/render_hero.py`** — the before/after pair that now opens both READMEs: the raw scan
  with the points SnowClear removes marked in red, and the de-snowed result, each as a full
  scene plus a zoom. The zoom window is chosen by scoring whole windows for a moderate removed
  fraction, so it lands where snow lies on visible structure (the densest cluster is a red cloud
  with nothing underneath); marker size and window size are set so the red survives GitHub's
  downscaling to README width.
- **`tools/render_qualitative.py`** — renders the qualitative comparison figure a paper needs:
  (a) raw scan, (b) annotated snow, (c) the detection split into TP / FN / FP with the per-frame
  precision / recall / F1 called out in the panel and leader lines at the densest error clusters,
  (d) the de-snowed cloud. An optional second detection turns it into a method comparison.
  Every number printed in a panel is recomputed from the index files and matches what the
  pipeline reports for the same frame.
- **`tools/pcd_common.py`** — one reader for PCD files, index files and the confusion matrix,
  shared by the audit tool and the renderer, so a figure cannot disagree with a table.
- **`tools/eval_baselines.sh`** — runs SnowClear and the compiled-in baselines (DROR / DSOR / SOR /
  ROR) over the reported set - the 16 of 19 mirrored scenes that are not 14, 16 and 76, i.e. the
  same 1 620 frames Table 1 reports - through `--mode eval_folders`, one CSV row per scene. The
  evaluated set is staged as a symlink farm, because `eval_folders` walks whatever directories it
  finds under `pcd_root`.
- **`tools/gen_baseline_fig.py`** — Table 2 and Fig. 3 from those CSVs: precision / recall / F1 as
  the macro average over scenes, with the latency under each method name. It prints the table as
  markdown, so the README cannot drift from the measurement.
- **`tools/render_baseline_clouds.py`** — Fig. 12: one frame, five detectors, one camera. Every
  panel goes through `render_hero3d.draw_scene`, and the framing comes from the ground truth and
  the ROI structure rather than from a method's own output, so no panel can flatter itself by
  zooming somewhere convenient.
- **`tools/make_derived_frames.py`**, **`tools/measure_portability.sh`** and
  **`tools/gen_portability_fig.py`** — Fig. 7, the portability claim measured instead of asserted.
  The derived mirror re-emits every cloud with `z += -0.9 m` (a rigid translation with a flat
  ground: a simulated higher mount) and carries the ground-truth indices over unchanged, because
  the point order does not move. A +0.9 m mount costs the released constants **24.25 pp** of macro
  F1 (92.82 → 68.57; the same 68.6 the original audit reported, which this protocol therefore
  reproduces to 0.01 pp) while precision holds and recall halves; the self-calibrated switches hold
  92.59, i.e. they recover all but 0.06 pp, at 2.4x the frame time. The CADC row is left as an
  explicit gap rather than a number this repository cannot regenerate.
- **`tools/measure_timing.sh`** and **`tools/gen_runtime_fig.py`** — Fig. 6, the frame-time
  budget. The figure judges each equivalence-preserving switch on the stage it can touch and
  pairs the runs round by round, because on this shared machine a single pass can be 40 % off the
  next one: the `α(r)` LUT is worth a paired +1.53 ms (3/3 runs) and the elevation fast path is
  below the coarse timers' resolution. All configurations print the same F1, which the script
  checks before drawing an equivalence claim.
- **`tools/audit_high_intensity.py`** — the bright-band measurement behind OPTIMIZATION §3: the
  `I >= 2` pool is 6.24 % of all ground truth, non-snow outnumbers it 19:1 in the same band, and
  the best geometric cut buys +0.02 pp of recall for −0.10 pp of F1. Verdict: no opt-in switch.
- **`--mode roi_variants` in `tools/audit_error_budget.py`** and **Fig. 8**
  (`tools/gen_ceiling_fig.py`) — the ground-truth budget per scene, and the measurement that closes
  the ROI direction: widening the gate to `z <= 4 m`, `r <= 25 m` and `-30 deg` recovers at most
  0.18 pp of ground truth while making 8-137 extra non-snow points reachable per frame. The veto
  predicate is now a KD-tree lookup instead of a per-point Python loop, which is what makes a
  seven-variant sweep of the same frames affordable; the scene-35 budget still reads 97.07 %
  reachable, unchanged.
- **`tools/gen_per_scene_fig.py`** — Fig. 1: per-scene precision / recall / F1 over the 19 mirrored
  scenes with the 16-scene reported set shaded. It reproduces every documented per-scene number and
  flags drift; the same run is what surfaced the mixed weighting in the all-19 row (below).
- **`tools/gen_ablation_fig.py`** — Fig. 4: macro-F1 delta per switch on the 4-scene subset, with
  the released configuration and the `ROI + I = 0` rule as reference lines. Every delta reproduces
  `OPTIMIZATION.md` §6 to within 0.05 pp; the grid-search optimiser is confirmed as a no-op in F1
  but not in time (+0.0012 pp for 4.2× the frame time).
- **`tools/gen_acceptance_fig.py`** / **`tools/gen_threshold_fig.py`** — Fig. 5 and Fig. 9, analytic:
  the acceptance region of the released decision function in the `(I/T, h_ag)` plane with the
  intensity ceiling it implies, and `T(r, I)` with the `α(r)` weight and the per-frame
  `Tg = clamp(0.8·Q1, 2.5, 8.0)` behind it. No dataset and no arguments — they evaluate the shipped
  equations in closed form, and mark the measured facts (69.7 % of frames on the clamp floor,
  largest detected `I = 1`) on the curves.
- **Published figures**: `fig2_qualitative[_zh].png` (scene 35, reference frame),
  `fig10_qualitative_hard[_zh].png` (scene 16, recall-limited — most misses are ground truth
  beyond the ROI) and `fig11_comparison[_zh].png` (SnowClear against SOR on one frame), all
  referenced live from both READMEs.

### Added

- **CRFOR in the comparison.** `tools/eval_crfor.py` runs the upstream implementation (Wang et al.,
  RA-L 2023) on our frames with our ground truth and our metric, at its published parameters, and
  writes indices in the same format as everything else. Over the same 101 frames of scene 35:
  SnowClear F1 96.90, CRFOR 96.41, SOR 56.68, DSOR 12.73, DROR 12.48, ROR 3.63 - at 9 ms per frame
  against CRFOR's 17 132 ms. Its preprocessing is its own, so the table says so.
- **`tools/bev_panel.py`** - the bird's-eye panel rendering shared by the animated hero and the new
  comparison board, so every panel on the page is drawn by one piece of code.

### Changed

- **The comparison board replaced the six-panel 3D figure** (`tools/render_comparison_board.py`):
  ground truth plus seven methods in the same bird's-eye grammar as the hero, each panel carrying
  its own in-ROI P / R / F1, with a scoreboard cell so the picture and the numbers cannot drift.
- **The results page is three figures instead of fourteen.** The board, one ground-truth budget
  figure (`tools/gen_budget_fig.py`) and the hero carry the story; per-scene, ablation, frame-time,
  closed-form and intensity figures stay published and indexed in `docs/figures/README.md`.

- **The animated hero is built for comparison, not decoration.** Six panels per frame — **SnowClear**
  and **ground truth** on separate rows — where each column answers one question: *Raw scan* is the
  input, *Removed* is what came out with ours coloured by whether the annotation agrees, *De-snowed*
  is what stayed in with our misses marked in blue. Because both rows share the columns, the
  comparison is direct: short green means under-removal, red means over-removal, and a clean-looking
  cloud cannot pass for a correct one. Contrast was raised at the same time (mid-grey cloud, near
  black strong returns, larger class markers), and the labels are English in both READMEs, because a
  figure regenerated per language drifts.
- **Two analysis figures, and every figure now carries its conclusion.** Fig. 13 is one scene frame by
  frame — F1 98.0 across 101 frames with the detector removing about as many points as are annotated —
  and Fig. 14 is the weak-return test in the data: 99.6 % of annotated returns on scene 35 are exactly
  `I = 0` while 98 % of everything else is brighter, and the ceiling that follows hides 0.19 % of the
  annotations there against 27.20 % on scene 16. That last number is the honest form of a claim the
  README previously made with a single pooled figure.

- **Every figure is drawn on white.** The 3D hero, the five-detector comparison, the before/after
  card and the animated hero were dark-themed, which reads badly next to the light charts and worse
  once GitHub scales them down. They now use the same light palette as everything else - structure
  dark-on-light, TP green, FP red, FN blue - and were regenerated by the same tools with the same
  commands, so every caption and every command in `docs/figures/README.md` still holds.
- **Both READMEs carry every published figure again**, laid out by what the figure is: charts run
  full width so their axis labels survive GitHub's scaling, while qualitative and comparison panels
  sit two-up in tables with their captions. Nothing was cut to make room - the long-form text still
  lives in the local `LOCAL_NOTES.md`.

- **The READMEs are front pages now.** `README.md` went from 603 to ~200 lines and `README_CN.md`
  from 554 to ~200: usage first, a five-line method, one compact result table and four figures that
  carry the evidence, with the long-form material moved verbatim to a local `LOCAL_NOTES.md`
  (git-ignored, never pushed) and the reference documents left where they were.
- **An animated hero**, `docs/figures/detect_scene35.gif` (+ `_zh`): 26 consecutive frames of scene
  35 in one fixed view — raw scan, detections coloured by outcome, de-snowed cloud — rendered by
  `tools/render_detection_gif.py` from the same per-frame indices the evaluation uses, so the
  animation cannot disagree with the tables.

- **The all-19 row no longer mixes weightings**: its recall was the frame-weighted mean while its
  precision and F1 were macro-over-scenes. Corrected to 86.1053 in `README.md`, `README_CN.md` and
  `DATASET.md`; the 16-scene reported-set row is unaffected.
- **Table 2 is measured, not `TODO`**: the four compiled-in baselines now run over the reported set
  (16 scenes / 1 620 frames — the same frames as Table 1) instead of a 4-scene subset, so Table 2,
  Fig. 3 and Fig. 12 all come out of one `tools/eval_baselines.sh` run. SnowClear keeps 92.82
  macro F1 against 37.93 for the best baseline (SOR), 7.36 for DSOR, 6.80 for DROR and 1.76 for
  ROR; the density filters earn their precision by returning almost nothing.
- **The hero shows the detection outcome, not just the clouds**: four RViz displays — grey ROI
  structure, green true positives, blue misses, red false positives — named so the Displays tree is
  the legend, with `FlatColor` replacing RViz's rainbow intensity ramp and a low orbit camera
  (distance 22, pitch 0.20) for an actual perspective. `tools/rviz_feed.py` derives the four classes
  from the released reference output and the ground truth.
- **The README hero is a real RViz screenshot** of the shipped layout: grey de-snowed cloud,
  the removed points in red, every display reporting `Status: Ok`. `tools/capture_rviz_screenshot.sh`
  reproduces it — RViz first, then a light publisher that sends the three clouds straight from the
  PCD and the released reference indices (`tools/rviz_feed.py`), then the X11 client window is
  captured with `xwd` and decoded by `tools/xwd_to_png.py`. Three things had to be right for the
  still to read: a static `map -> lidar` transform (without a TF tree RViz warns that the fixed
  frame does not exist even though it renders), the dimmed raw overlay off (it shares coordinates
  with the removed points and washes the red out), and a flat grey de-snowed cloud instead of
  RViz's rainbow intensity ramp. All are asset-only settings on a copy of `snowclear.rviz`.
- The earlier matplotlib hero render is gone; `tools/render_hero.py --banner` still produces the
  flat bird's-eye pair for print.
- **The READMEs open like a library homepage now.** The paper-style title is condensed to a name
  plus the method acronym (RITS), and the first screen is: title, one hero card (raw zoom with the
  removed returns in red → cleaned zoom), three sentences, a five-row fact table, then the install
  commands. Requirements and Quick start moved above Method and Results.
- The 2 500-character Summary/Highlights pair is gone; its content is the fact table and the
  sections below. Tables 4–7 (per-scene detail, frame time, cross-sensor robustness, error budget)
  are collapsed into one `<details>` block, and the seven commented-out figure slots moved out of
  the READMEs into `docs/figures/README.md`, which was already their index.

- **`docs/OPTIMIZATION.md`** — a measured review of the released method's approach: the error
  budget (which stage makes each ground-truth point undetectable), the intensity bimodality that
  makes every threshold knob inert, the threshold that is pinned at its clamp in 70 % of frames,
  baseline and single-switch ablation comparisons on a 406-frame subset, and a prioritised
  roadmap with the experiments that would validate each item.
- **`tools/audit_error_budget.py`** — the audit tooling [`MIGRATION_ROS1.md`](docs/MIGRATION_ROS1.md)
  §5 recorded as never ported. Three modes (`budget`, `intensities`, `separability`) recompute
  the shipped equations from the data without running the pipeline, so they measure the
  *configuration's* structure rather than one execution of it.
- **`tools/gen_algorithm_fig.py`** and `docs/figures/algorithm1_{en,zh}.{png,svg}` — Algorithm 1
  rendered as a figure. A fenced code block inherits whatever monospace font the reader's browser
  picks, and a CJK annotation on the same line destroys the column grid, so the algorithm is now
  an image with a collapsed plain-text fallback.

### Changed

- README Table 2 and Table 3 now point at the measured 4-scene subset (baselines and
  single-switch ablations) in `docs/OPTIMIZATION.md` §6–§7, and Results gained Table 7, the
  measured error budget.
- `docs/figures/README.md` indexes the rendered algorithm alongside the nine result-figure slots.

### Fixed

Documentation, comments and tooling only — no behaviour change. Both byte-exact gates and all
15 unit tests still pass, and the generated parameter views are unchanged
(`gen_param_map.py --check` and `gen_ros2_params.py --check` both clean).

- `tools/verify.sh` hard-coded an author-local dataset path. It now defaults to the documented
  `./data` layout (still overridable via `SNOWCLEAR_DATA`) and exits with the layout reference
  when the frame is missing, instead of reporting a confusing reference mismatch.
- Six comments and one documentation section pointed at `docs/AUDIT_REPORT.md`, which is not
  distributed with this repository. They now point at `docs/MIGRATION_ROS1.md` §5.
- Comments in ten files still used the pre-port header names (`system_config.h`,
  `ablation_switches.h`, `evaluator.h`, `sensor_calibration.h`). The headers are `.hpp` under
  `include/snowclear/`.
- `src/preprocessor.cpp`'s parameter table quoted stale defaults (`height_threshold` 2.3,
  `xy_threshold` 13.0) instead of the released 2.6 / 17.0.
- The two divergent point-count gates behind `enable_pre_downsampling` (>100 k points in the
  detector stage, >400 k in `Preprocessor::run`) are now stated at both sites and in the switch
  declaration, explicitly marked as deliberately unchanged: unifying them would alter behaviour
  when the switch is enabled.

## [1.0.0] — 2026-09-10

First release of SnowClear: a ROS 2 port of the ROS 1 `clustering` catkin package,
restructured so that the algorithm carries no middleware dependency.

### Added

- **`snowclear_core`** — the detection algorithm as a ROS-free library
  (PCL + OpenMP + TBB only), with a generated typed parameter registry.
- **`snowclear_ros`** — an rclcpp node subscribing to `sensor_msgs/PointCloud2` and
  publishing the de-snowed cloud, the removed points and their indices; a Python launch file;
  an RViz layout; and `scan_to_cloud.py` for replaying PCD files without a sensor.
- **`CloudOperations::process_cloud()`** — a single, file-I/O-free entry point for one frame,
  now shared by the offline CLI, the experiment runner and the ROS 2 node. The ROS 1 build had
  two near-duplicate per-frame paths.
- **Two byte-exact gates**: an offline regression against the released reference output
  (`--mode all_checks`) and a live publish/subscribe check (`test/live_check.py`) that asserts
  the ROS 2 path emits the same indices as the offline build.
- **Generated configuration views** — `tools/gen_param_map.py` derives `to_map()` and the
  parameter registry from the loaders; `tools/gen_ros2_params.py` derives the ROS 2 params file
  from the flat YAML. Both support `--check`.
- Documentation: `docs/ARCHITECTURE.md`, `docs/METHOD.md`, `docs/ROS2.md`,
  `docs/DATASET.md`, `docs/MIGRATION_ROS1.md`.
- Bilingual `README.md` / `README_CN.md`, `CONTRIBUTING.md`, `LICENSE` and `CITATION.cff`
  (the last two as explicit placeholders).

### Changed

- **Packages split by dependency.** The ROS 1 single catkin package became two ament packages:
  an algorithm core with no ROS dependency, and a thin ROS 2 adapter containing no algorithm.
- **Configuration flows through `snowclear::ParamSource`.** `SystemConfig::load_from_ros` and
  `AblationSwitches::load_from_ros` became `load(const ParamSource&)`, with `RosParamSource`
  for the node and `MapParamSource` (YAML + `key:=value`) for the offline tools.
- **Logging flows through `snowclear::LogSink`.** `ROS_*` became `SC_*` with identical
  printf formatting and identical message strings; the node forwards to `RCLCPP_*`.
- **Timing attribution corrected.** The bucket labelled "parameter optimisation" contained
  ground-truth I/O and feature analysis as well as the optimiser. It is now split, and
  ground-truth I/O is charged to `io_time`. Under the released configuration the optimiser
  itself contributes 0.0001 ms per frame.
- Headers moved to `include/snowclear/*.hpp` and everything placed in `namespace snowclear`.
- The single-frame path now accounts for disk I/O separately, matching the multi-frame path.
- The launch files no longer hard-code an absolute dataset path; a missing input fails loudly.

### Fixed

Carried over from the ROS 1 sources, all behaviour-preserving for the released configuration
(the byte-exact regression passes):

- Stage durations printed as `ms` while computed in microseconds — a 1000× mislabel.
- `ror_radius` drove two different members, so the preprocessor's outlier radius and the ROR
  baseline's radius could not be set independently. Split into `ror_radius` and
  `dynamic_ror_radius`; both default to 0.8.
- The grid-search score function computed an F-beta and discarded it while its comment claimed
  otherwise. Dead computation removed and the comment corrected.
- `point_idx >= 0` compared a `size_t`, so it was always true.
- The shipped configuration contained a `use_fast_search` key that no code read; `param_check`
  now reports any unread key.
- The parameter-consistency guard skipped values it could not parse instead of failing, and
  compared floats with a tolerance tighter than `float` storage.

### Verified

- Offline: `[OK] 配置一致 (104 个键, 校验 104 个, 生效参数 109 个)` and
  `[OK] 检测输出与参考逐字节一致 ("reference_042126.txt", 6957 行)`.
- Live: `[OK] ROS 2 在线路径与参考逐位一致（6957 个索引）`.
- Unit tests: 11 in `snowclear_core` (config, parameter plumbing, decision-function
  invariants) and 4 in `snowclear_ros` (parameter declaration, defaults, round-trip).
- Released metrics unchanged: macro P / R / F1 = 96.6934 / 89.9765 / 92.8229 over the
  16-scene, 1 620-frame evaluation set; pooled 96.8927 / 88.7047 / 92.6181.
