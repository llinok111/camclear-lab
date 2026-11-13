// Deterministic sample-scene generator. Two runs with the same code produce
// byte-identical data/scenes.json and data/ref.json.
//
// Usage: node tools/gen-sample.mjs
import { writeFileSync, mkdirSync } from 'node:fs';
import { meshFromBoxes, fnv1a, meshHash, planSafePath, planSafePathAdaptive, resamplePolyline, smoothPath, pointToSegment } from '../src/core.mjs';

export const TARGET = 0.12;

function mulberry32(a) {
  return function () {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const LAYOUTS = {
  slalom: [[1, 2.0, 2.0], [2.4, 2.0, 4.8], [3.9, 2.0, 2.0], [5.4, 2.0, 4.8], [7.0, 2.0, 2.0]],
  corner: [[1.2, 2.0, 1.5], [4.6, 2.0, 1.5], [6.4, 2.0, 3.2], [6.4, 2.0, 6.2]],
  diagonal: [[1.2, 1.8, 1.2], [3.2, 2.6, 3.2], [4.8, 1.8, 4.8], [6.6, 2.6, 6.6]],
};

const SCENES = [
  { name: 'slalom-a', layout: 'slalom', seed: 11, boxes: 8, region: [0.6, 7.4, 0.8, 4.0, 0.8, 6.0] },
  { name: 'slalom-b', layout: 'slalom', seed: 23, boxes: 9, region: [0.6, 7.4, 0.8, 4.0, 0.8, 6.0] },
  { name: 'slalom-c', layout: 'slalom', seed: 29, boxes: 10, region: [0.6, 7.4, 0.8, 4.0, 0.8, 6.0] },
  { name: 'corner-a', layout: 'corner', seed: 37, boxes: 8, region: [0.8, 6.8, 0.8, 4.0, 0.8, 6.8] },
  { name: 'corner-b', layout: 'corner', seed: 41, boxes: 10, region: [0.8, 6.8, 0.8, 4.0, 0.8, 6.8] },
  { name: 'corner-c', layout: 'corner', seed: 47, boxes: 12, region: [0.8, 6.8, 0.8, 4.0, 0.8, 6.8] },
];

function distToWaypoints(p, wps) {
  let m = Infinity;
  for (const q of wps) {
    const dx = p[0] - q[0], dy = p[1] - q[1], dz = p[2] - q[2];
    m = Math.min(m, Math.sqrt(dx * dx + dy * dy + dz * dz));
  }
  return m;
}

// Boxes must intrude on the smoothed spline while staying clear of the raw
// waypoint polyline: that is exactly the gap this planner is about.
// Place each obstacle centred on the aggressively smoothed path, at points the
// raw waypoint polyline stays well clear of. By construction the naive smooth
// path is obstructed while the raw polyline is safe.
function makeBoxes(rand, cfg, wps) {
  const raw = resamplePolyline(wps, 64);
  const smooth = smoothPath(raw, 36);
  const boxes = [];
  let guard = 0;
  while (boxes.length < cfg.boxes && guard++ < 20000) {
    const c0 = smooth[Math.floor(rand() * smooth.length)];
    const c = [c0[0] + (rand() - 0.5) * 0.3, c0[1] + (rand() - 0.5) * 0.3, c0[2] + (rand() - 0.5) * 0.3];
    let dRaw = Infinity;
    for (let i = 0; i + 1 < raw.length; i++) {
      const d = pointToSegment(c, raw[i], raw[i + 1]);
      if (d < dRaw) dRaw = d;
    }
    const hMax = Math.min(0.9, dRaw - 0.16);
    if (hMax < 0.2) continue;
    const sz = [0.24 + rand() * (hMax * 0.9), 0.28 + rand() * (hMax * 1.2), 0.24 + rand() * (hMax * 0.9)];
    if (0.5 * Math.hypot(sz[0], sz[1], sz[2]) > hMax) continue;
    boxes.push({ center: c.map((v) => Math.round(v * 1e4) / 1e4), size: sz.map((v) => Math.round(v * 1e4) / 1e4) });
  }
  return boxes;
}

// Build one scene; retry seeds deterministically until naive smoothing is
// actually unsafe at TARGET (that is the failure mode this tool repairs).
export function buildScene(cfg) {
  const wps = LAYOUTS[cfg.layout];
  for (let attempt = 0; attempt < 60; attempt++) {
    const rand = mulberry32(cfg.seed + attempt * 7919);
    const boxes = makeBoxes(rand, cfg, wps);
    if (boxes.length < cfg.boxes) continue;
    const mesh = meshFromBoxes(boxes);
    const plan = planSafePath(wps, mesh, TARGET);
    if (process.env.DBG) console.error(cfg.name, 'try', attempt, 'boxes', boxes.length, 'feas', plan.feasible, 'naive', plan.naive ? plan.naive.certified.toFixed(3) : '-', 'alpha', plan.alpha);
    if (plan.feasible && plan.naive.certified < TARGET) {
      return { name: cfg.name, layout: cfg.layout, seed: cfg.seed, attempt, target: TARGET, boxes, waypoints: wps, naive: plan.naive, plan, mesh };
    }
  }
  throw new Error('no unsafe scene for ' + cfg.name);
}

export function generateAll() {
  const scenes = SCENES.map(buildScene);
  const payload = {
    target: TARGET,
    scenes: scenes.map((s) => ({
      name: s.name, layout: s.layout, seed: s.seed, attempt: s.attempt,
      target: s.target, boxes: s.boxes, waypoints: s.waypoints,
      naive: {
        certified: s.naive.certified, sampledMin: s.naive.sampledMin,
        samples: s.naive.samples,
        worst: { i: s.naive.worst.i, u: s.naive.worst.u, d: s.naive.worst.d },
      },
    })),
  };
  return { scenes, payload, json: JSON.stringify(payload, null, 1) + '\n' };
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const { scenes, payload, json } = generateAll();
  mkdirSync(new URL('../data/', import.meta.url), { recursive: true });
  writeFileSync(new URL('../data/scenes.json', import.meta.url), json);
  const ref = {
    target: TARGET,
    generated: scenes.map((s) => {
      const u = planSafePath(s.waypoints, s.mesh, s.target);
      const a = planSafePathAdaptive(s.waypoints, s.mesh, s.target);
      return {
        name: s.name, boxes: s.boxes.length, tris: s.mesh.length / 9,
        meshHash: meshHash(s.mesh), naiveMin: s.naive.sampledMin,
        naiveCertified: s.naive.certified,
        adaptiveCert: a.certificate.certified, adaptiveMeanAlpha: a.meanAlpha,
        uniformAlpha: u.alpha,
      };
    }),
    sceneHash: fnv1a(json),
  };
  writeFileSync(new URL('../data/ref.json', import.meta.url), JSON.stringify(ref, null, 1) + '\n');
  console.log('wrote data/scenes.json  scenes=' + scenes.length +
    '  tris=' + scenes.reduce((a, s) => a + s.mesh.length / 9, 0) +
    '  hash=' + ref.sceneHash);
}
