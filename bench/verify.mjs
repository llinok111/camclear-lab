// Assertions with real numbers. Exit 0 only if every check passes.
import { generateAll } from '../tools/gen-sample.mjs';
import {
  planSafePath, planSafePathAdaptive, certifyPoints, clearanceAt, nearestOnMesh,
} from '../src/core.mjs';

let passed = 0, failed = 0;
const failures = [];
function ok(cond, label, detail = '') {
  if (cond) { passed++; console.log('PASS  ' + label + (detail ? '  [' + detail + ']' : '')); }
  else { failed++; failures.push(label); console.log('FAIL  ' + label + (detail ? '  [' + detail + ']' : '')); }
}

// --- independent oracle: closest point on triangle, projection + edge clamps
function oraclePointTriangle(p, a, b, c) {
  const sub = (u, v) => [u[0] - v[0], u[1] - v[1], u[2] - v[2]];
  const dot = (u, v) => u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
  const add = (u, v) => [u[0] + v[0], u[1] + v[1], u[2] + v[2]];
  const mul = (u, s) => [u[0] * s, u[1] * s, u[2] * s];
  const dist = (u, v) => Math.hypot(u[0] - v[0], u[1] - v[1], u[2] - v[2]);
  const n = [(b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1]),
             (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2]),
             (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])];
  const nn = dot(n, n);
  let best = Infinity;
  if (nn > 0) {
    const t = dot(sub(p, a), n) / nn;
    const q = sub(p, mul(n, t));
    // barycentric test for q inside abc
    const v0 = sub(b, a), v1 = sub(c, a), v2 = sub(q, a);
    const d00 = dot(v0, v0), d01 = dot(v0, v1), d11 = dot(v1, v1), d20 = dot(v2, v0), d21 = dot(v2, v1);
    const den = d00 * d11 - d01 * d01;
    if (Math.abs(den) > 1e-18) {
      const vv = (d11 * d20 - d01 * d21) / den;
      const ww = (d00 * d21 - d01 * d20) / den;
      if (vv >= 0 && ww >= 0 && vv + ww <= 1) best = dist(p, q);
    }
  }
  for (const [u, v] of [[a, b], [b, c], [c, a]]) {
    const uv = sub(v, u);
    const L2 = dot(uv, uv);
    let s = L2 > 0 ? dot(sub(p, u), uv) / L2 : 0;
    s = Math.max(0, Math.min(1, s));
    best = Math.min(best, dist(p, add(u, mul(uv, s))));
  }
  return best;
}
function oracleClearance(p, mesh) {
  let best = Infinity;
  for (let t = 0; t < mesh.length / 9; t++) {
    const o = t * 9;
    const a = [mesh[o], mesh[o + 1], mesh[o + 2]];
    const b = [mesh[o + 3], mesh[o + 4], mesh[o + 5]];
    const c = [mesh[o + 6], mesh[o + 7], mesh[o + 8]];
    const d = oraclePointTriangle(p, a, b, c);
    if (d < best) best = d;
  }
  return best;
}
// independent point-on-path evaluator (linear interpolation by segment index)
function evalPath(path, s) {
  const x = Math.max(0, Math.min(path.length - 1, s));
  const i = Math.min(path.length - 2, Math.floor(x));
  const f = x - i;
  const a = path[i], b = path[i + 1];
  return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f];
}

// --- G7-style determinism of the generator
const g1 = generateAll();
const g2 = generateAll();
ok(g1.json === g2.json, 'generator is byte-deterministic across two runs', 'hash=' + g1.payload.scenes.length);
ok(g1.scenes.length === 6, 'six sample scenes generated');

// --- per-scene planning
const rows = [];
for (const s of g1.scenes) {
  const uniform = planSafePath(s.waypoints, s.mesh, s.target);
  const adaptive = planSafePathAdaptive(s.waypoints, s.mesh, s.target);
  rows.push({ s, uniform, adaptive });

  ok(uniform.feasible && uniform.certificate.certified >= s.target,
    s.name + ': uniform blend is certified', 'alpha=' + uniform.alpha.toFixed(3) +
    ' cert=' + uniform.certificate.certified.toFixed(4));
  ok(adaptive.feasible && adaptive.certificate.certified >= s.target,
    s.name + ': adaptive blend is certified', 'meanA=' + adaptive.meanAlpha.toFixed(3) +
    ' cert=' + adaptive.certificate.certified.toFixed(4));
  ok(adaptive.meanAlpha > uniform.alpha,
    s.name + ': adaptive keeps more smoothing than uniform',
    adaptive.meanAlpha.toFixed(3) + ' > ' + uniform.alpha.toFixed(3));
  ok(adaptive.naive.certified < s.target,
    s.name + ': fully smoothed naive path fails the target',
    'naive cert=' + adaptive.naive.certified.toFixed(4));

  // independent dense oracle over the continuous path
  const path = adaptive.path;
  let oracleMin = Infinity;
  const N = path.length * 12;
  for (let k = 0; k <= N; k++) {
    const p = evalPath(path, (k / N) * (path.length - 1));
    const d = oracleClearance(p, s.mesh);
    if (d < oracleMin) oracleMin = d;
  }
  const tightness = oracleMin - adaptive.certificate.certified;
  ok(adaptive.certificate.certified <= oracleMin + 1e-9,
    s.name + ': certificate is a lower bound (sound)', 'cert=' + adaptive.certificate.certified.toFixed(4) +
    ' oracle=' + oracleMin.toFixed(4));
  ok(oracleMin >= s.target,
    s.name + ': independent oracle confirms clearance target', 'oracleMin=' + oracleMin.toFixed(4));
  ok(tightness >= 0 && tightness < 0.05,
    s.name + ': certificate is tight', 'gap=' + tightness.toFixed(5));
  console.log('      ' + s.name + '  naive=' + adaptive.naive.sampledMin.toFixed(4) +
    '  uniform a=' + uniform.alpha.toFixed(3) + '  adaptive meanA=' + adaptive.meanAlpha.toFixed(3) +
    '  oracleMin=' + oracleMin.toFixed(4) + '  tris=' + s.mesh.length / 9);
}

// --- point-triangle oracle on random points
function rng(seed) {
  let x = seed >>> 0;
  return () => { x ^= x << 13; x >>>= 0; x ^= x >> 17; x ^= x << 5; x >>>= 0; return x / 4294967296; };
}
const r = rng(12345);
let maxDelta = 0, cases = 0;
for (let k = 0; k < 300; k++) {
  const a = [r() * 4 - 2, r() * 4 - 2, r() * 4 - 2];
  const b = [r() * 4 - 2, r() * 4 - 2, r() * 4 - 2];
  const c = [r() * 4 - 2, r() * 4 - 2, r() * 4 - 2];
  const p = [r() * 6 - 3, r() * 6 - 3, r() * 6 - 3];
  const mesh = new Float64Array([...a, ...b, ...c]);
  const exact = nearestOnMesh(p, mesh).d;
  const oracle = oracleClearance(p, mesh);
  maxDelta = Math.max(maxDelta, Math.abs(exact - oracle));
  cases++;
}
ok(maxDelta < 1e-9, 'point-triangle distance matches independent oracle on random cases',
  cases + ' cases, max delta=' + maxDelta.toExponential(2));

// --- inside test vs analytic boxes
let insideAgree = 0, insideTotal = 0;
for (const s of g1.scenes) {
  for (let k = 0; k < 40; k++) {
    const p = [r() * 9 - 0.5, r() * 5 - 0.5, r() * 9 - 0.5];
    let want = false;
    for (const b of s.boxes) {
      if (Math.abs(p[0] - b.center[0]) <= b.size[0] / 2 &&
          Math.abs(p[1] - b.center[1]) <= b.size[1] / 2 &&
          Math.abs(p[2] - b.center[2]) <= b.size[2] / 2) { want = true; break; }
    }
    const got = clearanceAt(p, s.mesh).inside;
    insideTotal++;
    if (got === want) insideAgree++;
  }
}
ok(insideAgree === insideTotal, 'inside-mesh parity agrees with analytic box test',
  insideAgree + '/' + insideTotal);

// --- scale invariance across six orders of magnitude
const scales = [1e-6, 1e-3, 1, 1e3, 1e6];
let scaleOk = 0, scaleTotal = 0;
for (const s of g1.scenes) {
  const base = planSafePathAdaptive(s.waypoints, s.mesh, s.target);
  const baseRatio = base.certificate.certified / s.target;
  for (const sc of scales) {
    const w = s.waypoints.map((p) => p.map((v) => v * sc));
    const mesh = new Float64Array(s.mesh.length);
    for (let i = 0; i < mesh.length; i++) mesh[i] = s.mesh[i] * sc;
    const p = planSafePathAdaptive(w, mesh, s.target * sc);
    const ratio = p.certificate.certified / (s.target * sc);
    scaleTotal++;
    if (p.feasible === base.feasible && Math.abs(ratio - baseRatio) <= 1e-6 * Math.abs(baseRatio)) scaleOk++;
    else console.log('      scale mismatch ' + s.name + ' x' + sc + ' ratio=' + ratio.toFixed(12) + ' base=' + baseRatio.toFixed(12));
  }
}
ok(scaleOk === scaleTotal, 'planner decision and cert/target ratio invariant 1e-6..1e6',
  scaleOk + '/' + scaleTotal);

// --- certificate soundness on random dense paths (Lipschitz bound never over-claims)
let soundOk = 0;
for (let k = 0; k < 12; k++) {
  const s = g1.scenes[k % g1.scenes.length];
  const pts = [];
  for (let i = 0; i < 40; i++) pts.push([r() * 8, r() * 4, r() * 8]);
  const cert = certifyPoints(pts, s.mesh);
  let mx = Infinity;
  for (let i = 0; i < 2000; i++) {
    const p = evalPath(pts, r() * (pts.length - 1));
    mx = Math.min(mx, clearanceAt(p, s.mesh).d);
  }
  if (cert.certified <= mx + 1e-9) soundOk++;
}
ok(soundOk === 12, 'certificate lower bound never exceeds dense sample on 12 random paths', soundOk + '/12');

console.log('\n' + passed + ' passed, ' + failed + ' failed');
if (failed > 0) { console.log('failures: ' + failures.join(' | ')); process.exit(1); }
