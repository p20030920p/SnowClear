# Contributing

This repository accompanies a paper, so the first priority is that the published numbers stay
reproducible. Everything else is secondary to that.

---

## The rule

> Any change that can affect detection behaviour must move the byte-exact regression from
> passing to failing — loudly.

There are two such gates, and both must pass:

```bash
# offline: the core reproduces the released reference output byte for byte
ros2 run snowclear_core snowclear_runner --mode all_checks \
  --params src/snowclear_ros/config/snowclear_params.yaml \
  pcd_file:=<frame.pcd> reference_file:=testdata/reference_042126.txt \
  output_dir:=/tmp/regression

# live: the ROS 2 path emits the same indices
ros2 run snowclear_ros snowclear_node &
python3 src/snowclear_ros/test/live_check.py \
  --pcd <frame.pcd> --reference testdata/reference_042126.txt
```

If your change is *meant* to alter behaviour (a new feature, a better threshold), then:

1. put it behind a new switch in `include/snowclear/ablation_switches.hpp`, **default off**;
2. keep the released configuration bit-identical;
3. regenerate the reference output only in a dedicated commit that says so in its message.

---

## Getting set up

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Always build `Release`; `-O0` is roughly 10× slower and makes any timing claim meaningless.
`live_check.py` and the node's own launch file are the only places that set thread counts, and
they pin `OMP_NUM_THREADS` deliberately — keep that if you add tooling.

---

## Where code goes

| You are adding… | Put it in |
|---|---|
| an algorithm change | `snowclear_core/src/` + `include/snowclear/` |
| a new tunable | the loader in `system_config.cpp` **and** the shipped YAML, then run `python3 tools/gen_param_map.py` |
| a new on/off feature | `include/snowclear/ablation_switches.hpp`, then regenerate |
| ROS 2 plumbing | `snowclear_ros/src/` — no algorithm, ever |
| an offline tool | `snowclear_core/apps/` |
| a test | `snowclear_core/test/` for algorithm behaviour, `snowclear_ros/test/` for wiring |

**Never put algorithm code in `snowclear_ros`.** The whole point of the split is that the
detector can be built, tested and reasoned about without a middleware. A `#include <rclcpp/…>`
in `snowclear_core` is the one change that would undo the architecture.

---

## Generated files

`to_map()`, `param_registry()` and the ROS 2 params file are generated:

```bash
python3 tools/gen_param_map.py     # or --check
python3 tools/gen_ros2_params.py   # or --check
```

Edit the source it generates from, then regenerate. Do not hand-edit the generated blocks;
`--check` exists to catch exactly that.

---

## Style

The C++ sources use a deliberate, consistent layout. Please keep it:

- **A large block comment at the top of every file** stating its responsibility, the
  parameters it consumes, and the switches that gate its behaviour. This is the project's main
  defence against configuration drift — do not strip it, and update it when you change the file.
- Comments and log strings in Chinese, matching the surrounding code.
- Every non-obvious constant carries the *measurement* that justifies it (e.g. "1.140 ms →
  0.179 ms on a 54 k-point ROI frame, single-threaded -O3") rather than a restatement of what
  the code does.
- Known defects deliberately left in place are marked `【已知问题，故意未改】` with the reason.
- 4-space indent; `lower_snake_case` for functions and members with a trailing `_` for private
  members; `PascalCase` for types; everything inside `namespace snowclear`.
- Headers are `.hpp` under `include/snowclear/`.

Python tooling is analysis code, not a library: keep each file runnable as a script, and keep
`eval_folders`-style configuration single-sourced rather than copied.

---

## Submitting

1. Branch from `main`.
2. Run before pushing:
   ```bash
   colcon test --packages-select snowclear_core snowclear_ros
   python3 tools/gen_param_map.py --check && python3 tools/gen_ros2_params.py --check
   ros2 run snowclear_core snowclear_runner --mode all_checks ...
   ```
3. In the pull request, state what the change does, whether it changes detection behaviour
   (and if so the new macro P / R / F1), and the exact command you ran to verify.
4. Add an entry to `CHANGELOG.md` under `Unreleased`.

---

## Reporting a bug

Include:

- the output of `--mode all_checks`;
- your `OMP_NUM_THREADS`, and whether the machine is bare metal or virtualised;
- for ROS 2 issues, `ros2 param dump /snowclear` and whether `ROS_LOCALHOST_ONLY` was set;
- the command you ran, with real paths.

Reproduction failures are treated as real bugs. If a documented command does not work on a
clean clone, that is a defect in this repository, not in your setup.
