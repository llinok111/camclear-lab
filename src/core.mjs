// Pure geometry core: clearance-certified camera path planning.
// No three.js, no DOM, no I/O, no environment reads.
//
// A path is a Catmull-Rom spline through waypoints. The distance function to a
// triangle mesh is 1-Lipschitz, so sampling a span at m points with parameter
// speed bound V gives a *lower bound* on the clearance of the whole span:
//     min_u d(P(u)) >= min_k d(P(u_k)) - V/(2m)
// The planner inserts new waypoints at violating parameters, pushed away from
// the mesh, and re-certifies. Everything is deterministic Float64 math.

const EPS = 1e-12;

export function vadd(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
export function vsub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
export function vscale(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
export function vdot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
export function vcross(a, b) {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}
export function vlen(a) { return Math.sqrt(vdot(a, a)); }
export function vnorm(a) {
  const l = vlen(a);
  return l < EPS ? [0, 0, 0] : [a[0] / l, a[1] / l, a[2] / l];
}

// Closest point on triangle abc to p (Ericson, Real-Time Collision Detection).
export function closestPointOnTriangle(p, a, b, c) {
  const ab = vsub(b, a), ac = vsub(c, a), ap = vsub(p, a);
  const d1 = vdot(ab, ap), d2 = vdot(ac, ap);
  if (d1 <= 0 && d2 <= 0) return a;
  const bp = vsub(p, b);
  const d3 = vdot(ab, bp), d4 = vdot(ac, bp);
  if (d3 >= 0 && d4 <= d3) return b;
  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) {
    const v = d1 / (d1 - d3);
    return vadd(a, vscale(ab, v));
  }
  const cp = vsub(p, c);
  const d5 = vdot(ab, cp), d6 = vdot(ac, cp);
  if (d6 >= 0 && d5 <= d6) return c;
  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) {
    const w = d2 / (d2 - d6);
    return vadd(a, vscale(ac, w));
  }
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
    const w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return vadd(b, vscale(vsub(c, b), w));
  }
  const denom = 1 / (va + vb + vc);
  const v = vb * denom, w = vc * denom;
  return vadd(a, vadd(vscale(ab, v), vscale(ac, w)));
}

// Mesh is a flat Float64Array of 9*T numbers (three vertices per triangle).
export function triCount(mesh) { return mesh.length / 9; }

export function nearestOnMesh(p, mesh) {
  let bestD2 = Infinity, best = null, bestTri = -1;
  const n = triCount(mesh);
  for (let t = 0; t < n; t++) {
    const o = t * 9;
    const a = [mesh[o], mesh[o + 1], mesh[o + 2]];
    const b = [mesh[o + 3], mesh[o + 4], mesh[o + 5]];
    const c = [mesh[o + 6], mesh[o + 7], mesh[o + 8]];
    const q = closestPointOnTriangle(p, a, b, c);
    const dx = p[0] - q[0], dy = p[1] - q[1], dz = p[2] - q[2];
    const d2 = dx * dx + dy * dy + dz * dz;
    if (d2 < bestD2) { bestD2 = d2; best = q; bestTri = t; }
  }
  return { d: Math.sqrt(bestD2), point: best, tri: bestTri };
}

// Point-in-mesh parity test (closed triangle soup). Ray direction is fixed and
// deliberately not axis-aligned so box edges/faces are not hit degenerately.
const RAY_DIR = (() => {
  const v = [0.5731, 0.6172, 0.5387];
  const l = Math.hypot(v[0], v[1], v[2]);
  return [v[0] / l, v[1] / l, v[2] / l];
})();

export function insideMesh(p, mesh) {
  let hits = 0;
  const d = RAY_DIR;
  const n = triCount(mesh);
  for (let t = 0; t < n; t++) {
    const o = t * 9;
    const ax = mesh[o], ay = mesh[o + 1], az = mesh[o + 2];
    const e1x = mesh[o + 3] - ax, e1y = mesh[o + 4] - ay, e1z = mesh[o + 5] - az;
    const e2x = mesh[o + 6] - ax, e2y = mesh[o + 7] - ay, e2z = mesh[o + 8] - az;
    const px = d[1] * e2z - d[2] * e2y;
    const py = d[2] * e2x - d[0] * e2z;
    const pz = d[0] * e2y - d[1] * e2x;
    const det = e1x * px + e1y * py + e1z * pz;
    const ref = Math.hypot(e1x, e1y, e1z) * Math.hypot(e2x, e2y, e2z);
    if (Math.abs(det) < 1e-13 * ref || ref === 0) continue;
    const inv = 1 / det;
    const tx = p[0] - ax, ty = p[1] - ay, tz = p[2] - az;
    const u = (tx * px + ty * py + tz * pz) * inv;
    if (u < 0 || u > 1) continue;
    const qx = ty * e1z - tz * e1y;
    const qy = tz * e1x - tx * e1z;
    const qz = tx * e1y - ty * e1x;
    const v = (d[0] * qx + d[1] * qy + d[2] * qz) * inv;
    if (v < 0 || u + v > 1) continue;
    const tt = (e2x * qx + e2y * qy + e2z * qz) * inv;
    if (tt > 1e-9 * Math.hypot(e1x, e1y, e1z)) hits++;
  }
  return (hits & 1) === 1;
}

// Signed clearance: 0 when the point is inside the solid, surface distance
// otherwise. `push` always points from the point toward free space.
export function clearanceAt(p, mesh) {
  const q = nearestOnMesh(p, mesh);
  if (insideMesh(p, mesh)) {
    return { d: 0, point: q.point, inside: true, push: vnorm(vsub(q.point, p)), surfaceDist: q.d, tri: q.tri };
  }
  return { d: q.d, point: q.point, inside: false, push: vnorm(vsub(p, q.point)), surfaceDist: q.d, tri: q.tri };
}

// Dense polyline path: perSpan samples per waypoint span (segment), endpoints
// included once.
export function resamplePolyline(waypoints, perSpan = 64) {
  return waypointPolyline(waypoints, perSpan);
}

// Repeated box blur along the dense path (endpoints held fixed). A radius of
// r samples rounds corners over roughly r*spacing units, which is what makes
// the smoothed path cut far away from the raw polyline.
export function smoothPath(points, radius = 18, passes = 3) {
  let cur = points.map((p) => [p[0], p[1], p[2]]);
  const n = cur.length;
  for (let pass = 0; pass < passes; pass++) {
    const next = cur.map((p) => [p[0], p[1], p[2]]);
    for (let i = 1; i < n - 1; i++) {
      let sx = 0, sy = 0, sz = 0, cnt = 0;
      const lo = Math.max(0, i - radius), hi = Math.min(n - 1, i + radius);
      for (let j = lo; j <= hi; j++) { sx += cur[j][0]; sy += cur[j][1]; sz += cur[j][2]; cnt++; }
      next[i] = [sx / cnt, sy / cnt, sz / cnt];
    }
    cur = next;
  }
  return cur;
}

export function blendPaths(raw, smooth, alpha) {
  const out = new Array(raw.length);
  for (let i = 0; i < raw.length; i++) {
    const a = raw[i], b = smooth[i];
    out[i] = [a[0] + (b[0] - a[0]) * alpha, a[1] + (b[1] - a[1]) * alpha, a[2] + (b[2] - a[2]) * alpha];
  }
  return out;
}

/**
 * Certified clearance of a dense polyline. Each segment is bounded by its
 * endpoints; for any point q on a segment of length L, min(|q-A|,|q-B|) <=
 * L/2, and distance-to-mesh is 1-Lipschitz, so
 *     d(q) >= min(d(A), d(B)) - L/2  ... tightened to L/4 with the midpoint,
 * which is sampled too. Returned bound is a lower bound on the continuous path.
 */
export function certifyPoints(points, mesh) {
  let cert = Infinity, sampledMin = Infinity, total = 0, worst = null, maxSeg = 0;
  for (let i = 0; i < points.length; i++) {
    const p = points[i];
    const q = clearanceAt(p, mesh);
    total++;
    if (q.d < sampledMin) sampledMin = q.d;
    if (q.d < cert) { cert = q.d; worst = { i, u: 0, point: p, nearest: q.point, d: q.d, inside: q.inside, slack: 0 }; }
    if (i + 1 < points.length) {
      const a = points[i], b = points[i + 1];
      const m = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];
      const qm = clearanceAt(m, mesh);
      total++;
      if (qm.d < sampledMin) sampledMin = qm.d;
      const L = vlen(vsub(b, a));
      if (L > maxSeg) maxSeg = L;
      const slack = L / 4;
      if (qm.d - slack < cert) { cert = qm.d - slack; worst = { i, u: 0.5, point: m, nearest: qm.point, d: qm.d, inside: qm.inside, slack }; }
    }
  }
  return { certified: cert, sampledMin, samples: total, maxSeg, worst };
}

/**
 * Largest tested blend fraction that is still certified at `target`.
 * alpha=0 is the raw waypoint polyline (rejected as infeasible if it is not
 * safe), alpha=1 is fully smoothed. Bisection keeps lo certified and returns
 * lo, so the result is always backed by a passing certificate. `limiting`
 * carries the witness of the largest failing alpha: that is the obstacle that
 * decides how much smoothing the scene allows.
 */
export function planSafePath(waypoints, mesh, target, opts = {}) {
  const perSpan = opts.perSpan ?? 64;
  const iters = opts.iters ?? 36;
  const bias = opts.bias ?? 14;
  const raw = resamplePolyline(waypoints, perSpan);
  const smooth = smoothPath(raw, iters);
  const atZero = certifyPoints(raw, mesh);
  if (atZero.certified < target) {
    return { feasible: false, alpha: 0, raw, smooth, certificate: atZero, reason: 'raw waypoint polyline is not certified at target' };
  }
  const atOne = certifyPoints(smooth, mesh);
  if (atOne.certified >= target) {
    return { feasible: true, alpha: 1, raw, smooth, certificate: atOne, naive: atOne, limiting: null, iterations: 0, path: smooth };
  }
  let lo = 0, hi = 1, limiting = atOne, iterations = 0;
  for (let k = 0; k < bias; k++) {
    const mid = 0.5 * (lo + hi);
    const c = certifyPoints(blendPaths(raw, smooth, mid), mesh);
    iterations++;
    if (c.certified >= target) lo = mid; else { hi = mid; limiting = c; }
  }
  const path = blendPaths(raw, smooth, lo);
  const cert = certifyPoints(path, mesh);
  return {
    feasible: true, alpha: lo, raw, smooth, path,
    certificate: cert, naive: atOne, limiting, iterations,
  };
}

/**
 * Spatially adaptive version: instead of one global blend fraction, keep a
 * per-sample alpha field (alpha=1 fully smoothed, 0 = raw polyline). Rounds
 * lower alpha only in a window around the current worst violation, then blur
 * the field so the path does not kink. If the field ever collapses, a global
 * shrink is applied until the certificate passes, so alpha=0 (the raw
 * polyline) is the guaranteed fallback.
 */
export function planSafePathAdaptive(waypoints, mesh, target, opts = {}) {
  const perSpan = opts.perSpan ?? 64;
  const radius = opts.radius ?? 36;
  const maxRounds = opts.maxRounds ?? 60;
  const win = opts.win ?? 10;
  const raw = resamplePolyline(waypoints, perSpan);
  const smooth = smoothPath(raw, radius);
  const n = raw.length;
  const base = certifyPoints(raw, mesh);
  if (base.certified < target) {
    return { feasible: false, reason: 'raw waypoint polyline is not certified at target', certificate: base, raw, smooth };
  }
  const alpha = new Float64Array(n).fill(1);
  const blend = () => raw.map((p, i) => [
    p[0] + (smooth[i][0] - p[0]) * alpha[i],
    p[1] + (smooth[i][1] - p[1]) * alpha[i],
    p[2] + (smooth[i][2] - p[2]) * alpha[i],
  ]);
  const countPass = () => {
    let m = 0;
    for (let i = 0; i < n - 1; i++) if (Math.min(alpha[i], alpha[i + 1]) > 0.9) m++;
    return m / (n - 1);
  };
  let cert = certifyPoints(blend(), mesh);
  let rounds = 0;
  while (cert.certified < target && rounds < maxRounds) {
    rounds++;
    const i = cert.worst.i;
    for (let j = Math.max(0, i - win); j <= Math.min(n - 1, i + win); j++) {
      const w = 1 - Math.abs(j - i) / (win + 1);
      alpha[j] *= 1 - 0.35 * w;
    }
    // smooth the alpha field so the path stays visually clean
    const sm = Float64Array.from(alpha);
    for (let j = 1; j < n - 1; j++) {
      sm[j] = (alpha[j - 1] + alpha[j] + alpha[j + 1]) / 3;
    }
    alpha.set(sm);
    cert = certifyPoints(blend(), mesh);
  }
  let scale = 1;
  while (cert.certified < target && scale > 1e-6) {
    scale *= 0.7;
    for (let i = 0; i < n; i++) alpha[i] *= 0.7;
    cert = certifyPoints(blend(), mesh);
  }
  let sum = 0, mn = 1;
  for (let i = 0; i < n; i++) { sum += alpha[i]; if (alpha[i] < mn) mn = alpha[i]; }
  const path = blend();
  return {
    feasible: cert.certified >= target, alpha, path, raw, smooth, certificate: cert,
    meanAlpha: sum / n, minAlpha: mn, passFraction: countPass(),
    rounds, scale, iterations: rounds, naive: certifyPoints(smooth, mesh),
  };
}

// --- measurements ----------------------------------------------------------

export function waypointPolyline(waypoints, perSpan = 64) {
  const out = [];
  for (let i = 0; i + 1 < waypoints.length; i++) {
    const a = waypoints[i], b = waypoints[i + 1];
    for (let k = 0; k < perSpan; k++) {
      const u = k / perSpan;
      out.push([a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u]);
    }
  }
  out.push(waypoints[waypoints.length - 1]);
  return out;
}

export function pointToSegment(p, a, b) {
  const ab = vsub(b, a);
  const L2 = vdot(ab, ab);
  if (L2 < 1e-24) return vlen(vsub(p, a));
  let t = vdot(vsub(p, a), ab) / L2;
  t = Math.max(0, Math.min(1, t));
  return vlen(vsub(p, vadd(a, vscale(ab, t))));
}

export function boxTriangles(center, size) {
  const [cx, cy, cz] = center;
  const hx = size[0] / 2, hy = size[1] / 2, hz = size[2] / 2;
  const v = [
    [cx - hx, cy - hy, cz - hz], [cx + hx, cy - hy, cz - hz],
    [cx + hx, cy + hy, cz - hz], [cx - hx, cy + hy, cz - hz],
    [cx - hx, cy - hy, cz + hz], [cx + hx, cy - hy, cz + hz],
    [cx + hx, cy + hy, cz + hz], [cx - hx, cy + hy, cz + hz],
  ];
  const faces = [
    [0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6],
    [0, 4, 5], [0, 5, 1], [1, 5, 6], [1, 6, 2],
    [2, 6, 7], [2, 7, 3], [3, 7, 4], [3, 4, 0],
  ];
  const out = new Float64Array(faces.length * 9);
  faces.forEach((f, i) => {
    for (let k = 0; k < 3; k++) {
      const p = v[f[k]];
      out[i * 9 + k * 3] = p[0];
      out[i * 9 + k * 3 + 1] = p[1];
      out[i * 9 + k * 3 + 2] = p[2];
    }
  });
  return out;
}

export function meshFromBoxes(boxes) {
  const parts = boxes.map((b) => boxTriangles(b.center, b.size));
  const total = parts.reduce((s, p) => s + p.length, 0);
  const out = new Float64Array(total);
  let o = 0;
  for (const p of parts) { out.set(p, o); o += p.length; }
  return out;
}

export function fnv1a(txt) {
  let h = 0x811c9dc5;
  for (let i = 0; i < txt.length; i++) {
    h ^= txt.charCodeAt(i);
    h = Math.imul(h, 0x01000193) >>> 0;
  }
  return h.toString(16).padStart(8, '0');
}

export function meshHash(mesh) {
  let h = 0x811c9dc5;
  for (let i = 0; i < mesh.length; i++) {
    const k = Math.round(mesh[i] * 1e6);
    h ^= k & 0xff; h = Math.imul(h, 0x01000193) >>> 0;
    h ^= (k >>> 8) & 0xff; h = Math.imul(h, 0x01000193) >>> 0;
    h ^= (k >>> 16) & 0xff; h = Math.imul(h, 0x01000193) >>> 0;
  }
  return h.toString(16).padStart(8, '0');
}
