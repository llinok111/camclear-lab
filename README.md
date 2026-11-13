# camclear-lab

**Camera-path smoothing that can prove it misses the walls:** a fully smoothed (box-blurred) path drives through obstacles on all 6 sample scenes (sampled clearance **0.0000**, certified bound **−0.0094…−0.0060**), while the same core returns a *spatially adaptive* blend whose **certified lower bound is 0.1225–0.1391 ≥ 0.12** target, with **0.685–0.768 mean smoothing retained** versus **0.181–0.449** for a single global blend — a 1.67×–3.98× improvement at identical certified clearance. Every path number in this README is produced by `node bench/verify.mjs` and `node bench/bench.mjs`.

Upstream: none

## What it is

You give it waypoints for a camera fly-through and a closed triangle mesh (the sample scenes use box obstacles). It certifies the distance from the path to the mesh — **for the continuous path, not just for samples** — and repairs the path by locally reducing how much smoothing is applied, so the camera still rounds corners where there is room and tightens where there is not.

The certificate is a real lower bound: distance-to-mesh is 1-Lipschitz, so for a segment `AB` of length `L` with midpoint `M`, every point `q` on the segment satisfies `d(q) ≥ min(d(A), d(M), d(B)) − L/4`. Summing over densely sampled segments gives a bound for the whole path. `bench/verify.mjs` re-checks every certificate against an independently written triangle-distance routine and a 12× denser path sampling (gaps 0.0093–0.0123).

## Run it

```bash
node tools/gen-sample.mjs     # writes data/scenes.json + data/ref.json (deterministic)
node bench/verify.mjs         # 48 assertions, exit 0
node bench/bench.mjs          # writes bench/result.json
python3 -m http.server 4173   # then open http://localhost:4173
```

The browser page uses the same `src/core.mjs` as node and shows the node reference values from `data/ref.json` next to its own live computation; a mismatch is displayed in amber.

## Evidence (real runs, 2026-11-13)

Naive path = full smoothing, α = 1, no check. Adaptive = per-sample α field.

| scene | tris | naive sampled min | adaptive certified | adaptive mean α | uniform α | path share α>0.9 |
| --- | --- | --- | --- | --- | --- | --- |
| slalom-a | 96 | 0.0000 | 0.1225 | 0.685 | 0.313 | 38.3% |
| slalom-b | 108 | 0.0000 | 0.1242 | 0.759 | 0.455 | 44.1% |
| slalom-c | 120 | 0.0000 | 0.1263 | 0.754 | 0.449 | 50.0% |
| corner-a | 96 | 0.0000 | 0.1391 | 0.768 | 0.193 | 60.4% |
| corner-b | 120 | 0.0000 | 0.1291 | 0.713 | 0.181 | 52.1% |
| corner-c | 144 | 0.0000 | 0.1369 | 0.735 | 0.201 | 54.7% |

- Independent dense oracle on the planned path: 0.1298–0.1485; certificate gap 0.0093–0.0123; **sound** (bound never above the oracle) on all 6 scenes and on 12 random paths.
- Scale invariance 1e-6×…1e6×: **30/30** decisions and cert/target ratios unchanged (relative tolerances only; see limitations).
- Point-triangle distance vs an independently written projection/edge-clamp oracle: 300 random cases, max |Δ| = **8.88e-16**.
- Inside-mesh ray parity vs analytic box containment: **240/240**.
- Planning time (7 samples per scene after warmup): per-scene medians **103–210 ms** for the adaptive planner, **79–134 ms** for the uniform planner, cold start **215.3 ms**; pooled median 168.8 ms with **RSD 21.5%** because samples span six different scenes (per-scene RSD ≤ **2.5%**). Certifying one already-built naive path costs **6.8 ms** median. Clearance queries: **85.7k/s**.
- Generator is byte-deterministic: `node tools/gen-sample.mjs` twice gives the same `sceneHash=3643db3d`.

## 本轮创新

**Spatially adaptive smoothing budget with a certificate over the continuous path.** Existing three.js/NURBS camera tools apply one smoothing weight to the whole curve; collision-aware tools test the camera *rigid body* at discrete frames. Here the smoothing weight is a per-sample field, reduced only in a window around the current worst violation, and the reported clearance is a Lipschitz lower bound over every point of the path — the same core runs in node and in the browser, so the number on screen is re-verified by a 12× denser independent oracle. No judgement-only certificate: the planner *constructs* the path it certifies.

## How it works

1. `resamplePolyline` densifies the waypoints (64 samples/span).
2. `smoothPath` box-blurs the dense path (radius 36 samples, 3 passes), endpoints fixed — this is the aggressive smoothing that cuts corners.
3. `certifyPoints` evaluates clearance at every vertex and segment midpoint and subtracts `L/4`.
4. `planSafePathAdaptive` lowers α in a ±10-sample window around the worst violation, blurs the α field, and repeats; if the field collapses, it shrinks globally. α ≡ 0 is the raw waypoint polyline, which the sample generator guarantees is safe, so the fallback is always feasible.
5. `planSafePath` (uniform α, bisection) is the baseline that shows what spatial allocation buys.

## Known limitations

- **Closed meshes only.** Clearance uses surface distance plus an inside/outside parity test; open or non-manifold meshes make "inside" undefined. The sample data uses closed boxes on purpose.
- **Absolute `AREA`-style thresholds are avoided, but two remain:** the ray test uses a relative determinant guard `1e-13·|e1||e2|` and a relative `t > 1e-9·|e1|` guard; `vnorm` treats lengths below `1e-12` as zero. The verified range is **1e-6 … 1e6 units** with the target scaled proportionally; below ~1e-6 the vector epsilon starts to bite. Coordinates must be metres-scale or consistently scaled.
- **The certificate is conservative.** It subtracts `L/4` per segment, so the reported bound is below the true minimum (gaps 0.009–0.012 here). It never over-claims.
- **Planning cost is ~0.1–0.2 s** for 6 scenes × ~100–144 triangles; the inner loop is O(samples × triangles) with no BVH, so 10k-triangle scenes will need acceleration.
- **The α field is heuristic, not globally optimal.** It is greedy in the worst violation and may spend more smoothing than necessary near a single obstacle; the returned path is always certified, but a smaller mean-α path may exist.
- **C0 only at the waypoints** when α < 1: the blend of the raw polyline and the smoothed path is continuous but not C1 at waypoint knots. For fly-through previews the turn rate stays bounded; C1 blending is not implemented.
- The browser view is verified by `curl` (HTTP 200 + importmap) and by comparing its live numbers to `data/ref.json`; there is no GPU pixel comparison.

## Files

```
src/core.mjs          pure geometry + planner (no three, no DOM)
main.js               rendering/interaction only
index.html            three@0.169.0 importmap, fixed version
tools/gen-sample.mjs  deterministic scenes + data/ref.json
bench/verify.mjs      48 assertions incl. independent oracle
bench/bench.mjs       cold + 7 samples, medians, bench/result.json
```
