# Where the next improvement comes from

A review of the released method's *idea*, grounded in measurements rather than in the paper's
narrative. Everything below was produced by the commands in [§11](#11-reproducing-these-numbers)
on a 406-frame subset — scenes **35, 11, 14, 16** of the mirror, chosen because they span the
easy case (35, recall 97 %) and the two documented recall-limited scenes (14, 16). Nothing here
is extrapolated from a claim; where a number is a derived quantity it says so.

> **Read this together with [`METHOD.md`](METHOD.md).** That document says what the released
> configuration computes; this one says where its remaining error is, and what is worth trying
> next.

---

## 1. Headline: the detector already sits at its structural recall ceiling

`tools/audit_error_budget.py --mode budget` recomputes the shipped equations in exact arithmetic
and charges every ground-truth point to the **first** stage of Algorithm 1 that makes it
undetectable:

| Stage | Share of GT (per-frame macro) |
|---|---:|
| A. removed by the ROI gate | 24.35 % |
| B. above the intensity ceiling (`s > 0.75` ⟹ `I < 0.2132·T`) | 6.67 % |
| C. vetoed as attached to a bright surface | 0.41 % |
| **D. reachable by the shipped rule** | **68.57 %** |

Measured macro recall over the same 406 frames: **68.60 %**.

| Scene | Measured recall | Computed ceiling |
|---|---:|---:|
| 35 | 97.07 | 97.1 |
| 11 | 79.74 | 79.7 |
| 14 | 53.47 | 53.5 |
| 16 | 44.13 | 44.1 |

The two agree to within 0.05 pp on every scene — the residual is dominated by the 1 cm `α(r)`
LUT, which the audit recomputes in exact arithmetic instead of modelling. The conclusion is
uncomfortable but it is the whole story of this document:

> **Every ground-truth point the shipped rule can accept is already being found.**
> Re-tuning weights, thresholds or the threshold *shape* cannot raise recall: they change which
> points are accepted only through `T`, and the current operating point already sits inside an
> empty region of the intensity distribution (§3).

The corollary matters for how the work is described: what the published numbers measure is the
**reachability structure** of the configuration, not the discrimination ability of the
classifier. On the reference frame (scene 35) the rule is, in effect, "ROI ∧ `I = 0` ∧ not
attached to a surface" — which [`METHOD.md`](METHOD.md) §2 already notes reproduces 90.12 macro
F1 on its own, against 92.82 for the full pipeline.

---

## 2. Why recall is capped: the fused score has almost no usable range

With the released switches, `C = 0.7·s + 0.15·hag` while `θ ∈ {0.675, 0.75, 0.90}`:

- `C` can only reach **0.85** (`s = 1`, `hag = 1`), and `θ = 0.75` is the middle branch — so the
  pass region is the top ~12 % of the score's range.
- Two of the three `θ` branches are **unreachable by construction**: `s ≤ 0.4 ⟹ C ≤ 0.43 < 0.90`
  and `0.4 < s ≤ 0.7 ⟹ C ≤ 0.64 < 0.75`. Only the `s > 0.7` branch can ever fire.
- The height term saturates at 1.5 m above the local ground and contributes at most 0.15 — it
  cannot rescue an otherwise marginal point.
- Therefore detection requires `s > 0.75` ⟹ `I < 0.2132·T`, and since `T ≤ 0.8·Tg ≤ 6.4`, the
  acceptance band is `I < 1.36` at best.

This is a **score-design** problem, not a tuning problem. Measured (4-scene macro F1):

| `score_threshold` | P | R | F1 | Δ F1 |
|---|---:|---:|---:|---:|
| 0.55 | 94.38 | 68.62 | 77.06 | −0.50 |
| 0.65 | 95.19 | 68.61 | 77.43 | −0.13 |
| **0.75 (released)** | **95.47** | **68.60** | **77.56** | — |
| 0.85 | 91.77 | 11.21 | 19.74 | −57.82 |

| `idsor_scale` | P | R | F1 | Δ F1 |
|---|---:|---:|---:|---:|
| 1.0 | 95.34 | 68.60 | 77.50 | −0.06 |
| **0.8 (released)** | **95.47** | **68.60** | **77.56** | — |
| 1.2 | 95.22 | 68.61 | 77.45 | −0.11 |

Relaxing the threshold over a 0.55–0.75 range moves recall by **0.02 pp**; the F1 change is
precision-driven. That is the signature of a decision boundary sitting in a gap (§3), not of an
optimal operating point.

---

## 3. The ground truth is bimodal in intensity — so no threshold in the gap can matter

`tools/audit_error_budget.py --mode intensities`, over 5 276 594 ground-truth points inside the
ROI:

| `I = 0` | `0 < I < 1` | `1 ≤ I < 2` | `I ≥ 2` |
|---:|---:|---:|---:|
| **85.59 %** | **0.00 %** | 0.76 % | **13.65 %** |

Per scene, the `I ≥ 2` share is 0.18 % (35), 5.92 % (11), 13.20 % (14), 27.10 % (16).

The acceptance band found in §2 is `I < 0.43` for a typical frame (`Tg = 2.5 ⟹ T ≈ 2.0`). The
data has essentially **nothing between 0.43 and 2**. So:

- Every `θ` that yields an acceptance limit anywhere in `(0.43, 2)` produces *identical*
  detections. This is why the parameter sweeps in §2 and in `METHOD.md` report "≤ 0.005 pp": the
  knobs are being turned inside an empty interval.
- The 13.65 % of ground truth at `I ≥ 2` is unreachable by **any** setting of the current
  parameters, because `s > 0.75` is structural (it comes from the weight normalisation and the
  height cap, not from a tunable).
- Scene 35 has essentially no `I ≥ 2` ground truth (0.18 %), which is exactly why it scores 97 %
  recall while scenes 14 and 16 score 53 % and 44 %.

**Consequence.** If the paper's contribution is the feature-fusion machinery, this dataset does
not exercise it: on 85.6 % of the ground truth the decision reduces to `I = 0` plus geometry, and
the remaining 13.7 % is out of reach by construction. Two honest ways forward: evaluate on data
where the intensity gap is populated, or re-aim the method at the `I ≥ 2` points (§8, item 1).

---

### Widening the gate does not buy that recall back (measured)

The 24.35 % charged to the ROI gate above is not a pool the decision rule is failing to reach.
`python3 tools/audit_error_budget.py --mode roi_variants --scenes 35 11 14 16` re-scores every
frame with wider gates and counts what becomes reachable under the same proxy (ceiling plus veto):

| Wider gate | Δ GT reachable | Δ non-snow reachable | FP per recovered GT |
|---|---:|---:|---:|
| `z <= 4.0 m` | +0.06 pp | +8 / frame | 136 |
| `r <= 25 m` | +0.09 pp | +71 / frame | 767 |
| elevation `>= -30 deg` | +0.00 pp | +47 / frame | — |
| `z <= 4.0` and `r <= 25` | +0.18 pp | +89 / frame | 497 |
| all three widened | +0.18 pp | +137 / frame | 760 |

The annotated points outside the gate are also the ones the rest of the rule rejects — far, weak
and surface-attached — so widening it admits mostly non-snow: at best 0.18 pp of ground truth for
89 extra reachable non-snow points per frame. **Do not spend recall budget here.** The reachable
ceiling is a property of the whole rule, not of the gate alone, which is why item A above cannot be
recovered on its own and why the only lever with real headroom is the ceiling (roadmap item 1).

## 4. The "adaptive" intensity threshold is pinned at its lower clamp

Measured over 406 frames (`Tg = clamp(0.8·Q1, 2.5, 8.0)`, the value the CLI logs as
"强度阈值"):

| Quantity | Value |
|---|---|
| frames with `Q1 = 0` | 191 / 406 (47.0 %) |
| frames with `Tg = 2.5` (the clamp floor) | 283 / 406 (69.7 %) |
| distinct `Tg` values observed | **7** |
| `Tg` min / median / max | 2.50 / 2.50 / 7.20 |

In ~70 % of frames the "scene-adaptive" part of the threshold is a **constant**, and the only
adaptivity left is the `α(r)` shape. The cause is structural: `Q1` is computed over the
*post-ROI* cloud whose intensity distribution is dominated by snow (`I = 0` in 85.6 % of ground
truth; ~43 % of all ROI points), so `Q1 → 0` and the floor absorbs it.

`METHOD.md` §3 already records that the quantile-sentinel defect changes the final threshold on
**0** frames because the clamp masks it. This quantifies the masking: the clamp is doing the
work, in 70 % of frames, and the adaptive mechanism is decorative there.

---

## 5. Widening the ROI does not buy recall (measured)

| Variant | P | R | F1 | Δ F1 |
|---|---:|---:|---:|---:|
| **`xy_threshold` 17 (released)** | **95.47** | **68.60** | **77.56** | — |
| `xy_threshold` 25 | 94.90 | 68.70 | 77.43 | −0.13 |
| `xy_threshold` 40 | 94.15 | 68.77 | 77.20 | −0.36 |
| `height_threshold` 5.0 | 95.33 | 68.68 | 77.55 | −0.00 |

Tripling the radial bound recovers **0.17 pp** of recall and costs 1.3 pp of precision. The
reason is §2: the points the ROI rejects are far away, and at that range `α(r) → 0` so
`T → 0.8·Tg ≈ 2.0`, which admits only `I < 0.43`. **The ROI bound and the intensity ceiling are
the same wall.** Recovering stage A's 24.35 % therefore requires first making brighter points
admissible — which is why stage A must not be read as an independently recoverable 24 pp.

---

## 6. Ablations: which modules earn their place (measured)

4-scene macro average, one switch changed at a time:

| Configuration | P | R | F1 | Δ F1 | ms/frame |
|---|---:|---:|---:|---:|---:|
| **Released** | **95.47** | **68.60** | **77.56** | — | **8.1** |
| zero-intensity surface suppression **off** | 87.64 | 69.01 | 74.94 | **−2.62** | 5.1 |
| + `enable_planarity_calculation` | 94.93 | 51.77 | 65.17 | **−12.39** | 22.7 |
| + `enable_density_calculation` | 96.26 | 41.02 | 56.38 | **−21.18** | 21.1 |
| + `enable_feature_entropy` | 94.99 | 68.61 | 77.34 | −0.22 | 21.1 |

Reading:

- The **surface veto is the one module that pays for itself**: +7.8 pp precision for −0.4 pp
  recall. The documented ≈2.2 pp F1 cost on the 16-scene set reproduces as 2.62 pp here.
- Planarity and density are not merely neutral — they are **strongly harmful** (−12.4 and
  −21.2 pp) and 2.6× slower, because enabling either one changes the weight normalisation and
  lets the geometry term dominate the intensity evidence. Disabling them was the single best
  decision in the released configuration.
- Entropy costs 0.22 pp and 2.6× the time for nothing.

---

## 7. Baselines

Two measurements of this repository's own re-implementations, both sharing the identical ROI gate,
index mapping and evaluation path — so the comparison isolates the decision rule, not the plumbing.

**On the reported set** (16 scenes / 1 620 frames, the frames Table 1 uses):

| Method | P | R | F1 | ms/frame |
|---|---:|---:|---:|---:|
| DROR (Charron et al., 2018) | 85.54 | 3.57 | 6.80 | 1041 |
| DSOR (Kurup & Bos, 2021) | 85.02 | 3.88 | 7.36 | 70 |
| SOR (Rusu et al., 2008) | 89.24 | 25.56 | 37.93 | 110 |
| ROR (Rusu, 2009) | 79.70 | 0.89 | 1.76 | 1013 |
| **SnowClear (released)** | **96.69** | **89.97** | **92.82** | **≈ 10** |

Reproduce with `bash tools/eval_baselines.sh <outdir>` (one CSV per method, one row per scene);
`python3 tools/gen_baseline_fig.py --csv-dir <outdir> --latency-csv-dir <single-scene run>` prints
this table as markdown and draws README Fig. 3, `tools/render_baseline_clouds.py` draws Fig. 12.
Latencies are a single run on scene 35 (101 frames) at `OMP_NUM_THREADS=2`, the same protocol as
Table 5; they move with machine load, F1 does not.

**On the 4-scene subset** used by the rest of this document (scenes 35, 11, 14, 16 — it
deliberately includes the two recall-limited scenes):

| Method | P | R | F1 | ms/frame |
|---|---:|---:|---:|---:|
| DROR (Charron et al., 2018) | 80.70 | 3.76 | 7.10 | 606.7 |
| DSOR (Kurup & Bos, 2021) | 79.80 | 3.68 | 6.93 | 49.5 |
| SOR (Rusu et al., 2008) | 83.21 | 25.05 | 36.36 | 74.1 |
| ROR (Rusu, 2009) | 74.80 | 0.98 | 1.92 | 619.7 |
| **SnowClear (released)** | **95.47** | **68.60** | **77.56** | **8.1** |

The ordering is unambiguous on both sets: 2.1× the F1 of the best baseline on the hard subset,
2.4× on the reported set. The pure-geometry filters fail in an informative way — their precision
looks respectable *because* they remove almost nothing (recall < 4 % for DROR/DSOR/ROR), which is
the expected behaviour of density-based outlier removal against volumetric snowfall. Our own
number drops on the subset because scenes 14 and 16 are limited by the intensity ceiling of §3,
a failure mode the baselines do not share — and they still lose by more than 2×.

---

## 8. Prioritised roadmap

| # | Change | Why it is worth it | Expected | Cost | Verify with |
|---|---|---|---|---|---|
| **1** | **Re-parameterise the decision rule so `I ≥ 2` points are admissible** — e.g. drop the three-branch `θ` and the hard `s > 0.3`, and calibrate a monotone score (or a small logistic model) on `(s, hag, r, local geometry)` | It is the **only** lever on 13.65 % of the ground truth, and it is also the gate that makes the ROI's 24.35 % unreachable (§2, §3, §5) | recall ↑, precision ↓; F1 is an open question and must be measured, not assumed | M–L | scenes 14/16 first (they hold 40 % of the `I ≥ 2` GT), then the 16-scene set; watch the P/R curve, not one F1 |
| **2** | Fix or drop the threshold's adaptivity: compute `Q1` over `{I > 0}` (or a higher quantile) instead of the whole ROI cloud | 70 % of frames currently sit on the clamp floor, so the "scene-adaptive" claim is unsupported (§4) | F1 ≈ unchanged on WADS; removes a false claim; gives real headroom on other scenes | S | `--mode param_check` plus the 4-scene sweep |
| **3** | Keep the veto, and revisit its **absolute** support floor `I > 1.0` with the existing (`default-off`) self-calibration switch on a non-8-bit sensor | The veto is the only module with a positive contribution (§6); its floor is the documented portability failure (CADC: 90 % of ROI points classified as snow) | no WADS change; portability | S | the CADC / mounting-height experiments in `METHOD.md` §6 |
| **4** | Make the optimiser search what matters, or delete it: `score_threshold` jointly with `idsor_scale`, evaluated on the `I ≥ 2` subclass | Today it searches DCOR/RGOR/cluster parameters that the released detector never reads, and returns the defaults in 0.0001 ms/frame (`METHOD.md` §5) | currently 0; potentially the mechanism for item 1 | S–M | grid-search log lines that actually fire |
| **5** | Extend the evaluation set with frames where the intensity gap is populated | Every conclusion in §2–§3 is a statement about *this* dataset's label distribution | none directly; it makes the contribution measurable | M | report per scene, as `DATASET.md` already does for 14/16 |

Deliberately **not** on the list: multi-frame temporal consistency. `ablation_switches.hpp` records a
corrected implementation at +0.03 pp end-to-end, which does not justify a multi-frame cache and
an ICP dependency.

---

## 9. Dead ends — measured, do not spend more time here

| Idea | Measurement | Verdict |
|---|---|---|
| Weight sweeps across the fused score | `METHOD.md` §7: all six IDSOR parameters move macro F1 by ≤ 0.005 pp | inert (§3) |
| Any `score_threshold` in 0.55–0.75 | recall changes by 0.02 pp on 4 scenes | inert (§3) |
| Planarity / density / entropy terms | −12.4 / −21.2 / −0.2 pp and 2.6× slower | harmful or useless (§6) |
| Enlarging the ROI | +0.17 pp recall, −1.3 pp precision | no (§5) |
| Multi-frame occupancy | +0.03 pp end-to-end, needs a temporal cache | no (`ablation_switches.hpp` §8) |
| Pre-downsampling, isolated-point handling, grid search | off in the release; grid search is a no-op | no (`METHOD.md` §5) |

---

## 10. Threats to validity

- **Scope.** 4 scenes / 406 frames, deliberately including the two hardest scenes. The 16-scene
  reported set mixes them with easier ones, so absolute numbers here are lower
  (macro F1 77.56 vs 92.82). Every *comparison* is within this fixed subset.
- **Timing.** The sweeps use `snowclear_runner`, whose `time_ms` averages the per-frame stage
  budget; the paper's ≈10 ms excludes I/O and one 5-frame calibration window. Treat the baseline
  ratios (9×, 75×) as indicative, not calibrated.
- **Baselines.** Re-implementations in `dynamic_outlier_filters.cpp` on the shared pipeline, not
  the upstream harnesses; DROR/DSOR/SOR/ROR parameters were left at their shipped defaults and
  were **not** tuned per scene, which can only flatter the proposed method.
- **The ceiling is an upper bound, not a prediction.** It ignores the surface veto's dependence
  on neighbouring points and treats the `α(r)` LUT as exact.
- **The separability probe is not range-controlled.** `--mode separability` compares neighbour
  counts of `I ≥ 2` ground truth against other `I ≥ 2` points without matching range, and density
  falls steeply with range. Its result (medians 770/782/279/588 for GT vs
  1298/539/456/391 for non-GT — i.e. no consistent direction) is therefore evidence *against* a
  simple density feature, not proof that none exists.

---

## 11. Reproducing these numbers

```bash
export SNOWCLEAR_DATA=/path/to/wads/mirror        # layout: docs/DATASET.md

# 1. the error budget, the intensity distribution, and the separability probe
python3 tools/audit_error_budget.py --mode budget
python3 tools/audit_error_budget.py --mode intensities
python3 tools/audit_error_budget.py --mode separability

# 2. the sweeps (4 scenes, one CSV row per scene)
#    eval_folders needs the <scene>/velodyne level; symlink a subset to keep it quick
mkdir -p /tmp/sc/pcd /tmp/sc/result
for s in 35 11 14 16; do
  ln -s "$SNOWCLEAR_DATA/pcd_output/$s" /tmp/sc/pcd/$s
  ln -s "$SNOWCLEAR_DATA/result/$s"     /tmp/sc/result/$s
done
source install/setup.bash
for args in "" "detector_type:=dror" "detector_type:=dsor" "detector_type:=sor" \
            "detector_type:=ror" "score_threshold:=0.55" "score_threshold:=0.65" \
            "score_threshold:=0.85" "idsor_scale:=1.0" "idsor_scale:=1.2" \
            "xy_threshold:=25.0" "xy_threshold:=40.0" "height_threshold:=5.0" \
            "enable_zero_intensity_surface_suppression:=false" \
            "enable_planarity_calculation:=true" "enable_density_calculation:=true" \
            "enable_feature_entropy:=true"; do
  tag=$(echo "$args" | tr -c 'a-zA-Z0-9' '_'); tag=${tag:-released}
  OMP_NUM_THREADS=2 OMP_DYNAMIC=false ros2 run snowclear_core snowclear_runner \
    --mode eval_folders \
    --params src/snowclear_ros/config/snowclear_params.yaml \
    pcd_root:=/tmp/sc/pcd result_root:=/tmp/sc/result \
    csv_path:=/tmp/sc/$tag.csv $args
done
# each CSV holds one row per scene; macro-average the rows to reproduce the tables
```

Every table in this document is a macro average of those rows, except §1 (the audit tool) and
§4 (parsed from the run logs: `grep -oE '强度阈值: [0-9.]+ \(Q1: [0-9.]+'`).
