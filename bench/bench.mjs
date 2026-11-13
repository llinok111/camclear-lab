// Timing harness: 1 cold run + 1 warmup + 7 measured runs per configuration.
import { writeFileSync } from 'node:fs';
import { generateAll } from '../tools/gen-sample.mjs';
import { planSafePath, planSafePathAdaptive, certifyPoints, clearanceAt, resamplePolyline, smoothPath } from '../src/core.mjs';

function stats(arr) {
  const s = [...arr].sort((a, b) => a - b);
  const n = s.length;
  const median = n % 2 ? s[(n - 1) / 2] : 0.5 * (s[n / 2 - 1] + s[n / 2]);
  const mean = arr.reduce((a, b) => a + b, 0) / n;
  const variance = arr.reduce((a, b) => a + (b - mean) * (b - mean), 0) / n;
  return {
    median, p95: s[Math.min(n - 1, Math.floor(0.95 * n))],
    min: s[0], max: s[n - 1], mean,
    stddev: Math.sqrt(variance),
    rsdPct: 100 * Math.sqrt(variance) / mean,
  };
}

const { scenes } = generateAll();
const samples = 7;

// Cold start: the very first planner call in this process.
const coldT0 = performance.now();
planSafePathAdaptive(scenes[0].waypoints, scenes[0].mesh, scenes[0].target);
const coldMs = performance.now() - coldT0;

const adaptive = [], uniform = [], naive = [];
const perScene = [];
for (const s of scenes) {
  const smooth = smoothPath(resamplePolyline(s.waypoints, 64), 36);
  planSafePathAdaptive(s.waypoints, s.mesh, s.target); // warmup
  const a = [], u = [], nv = [];
  for (let i = 0; i < samples; i++) {
    let t = performance.now();
    planSafePathAdaptive(s.waypoints, s.mesh, s.target);
    a.push(performance.now() - t);
    t = performance.now();
    planSafePath(s.waypoints, s.mesh, s.target);
    u.push(performance.now() - t);
    t = performance.now();
    certifyPoints(smooth, s.mesh);
    nv.push(performance.now() - t);
  }
  adaptive.push(...a); uniform.push(...u); naive.push(...nv);
  const sa = stats(a), su = stats(u), sn = stats(nv);
  perScene.push({
    name: s.name,
    tris: s.mesh.length / 9,
    adaptiveMs: sa.median, adaptiveRsdPct: sa.rsdPct,
    uniformMs: su.median, uniformRsdPct: su.rsdPct,
    naiveCertifyMs: sn.median,
  });
}

// Clearance query throughput.
const qScene = scenes[0];
const queries = 30000;
{
  let acc = 0;
  const t0 = performance.now();
  for (let i = 0; i < queries; i++) {
    const p = [1 + (i % 97) * 0.07, 1 + (i % 13) * 0.11, 1 + (i % 31) * 0.19];
    acc += clearanceAt(p, qScene.mesh).d;
  }
  const dt = performance.now() - t0;
  var throughput = { queries, ms: dt, qps: queries / (dt / 1000), sink: acc };
}

const perSceneMaxRsd = Math.max(...perScene.map((p) => p.adaptiveRsdPct));
const out = {
  tool: 'camclear-lab',
  target: scenes[0].target,
  samplesPerConfig: samples,
  coldMs,
  aggregate: {
    adaptive: stats(adaptive),
    uniform: stats(uniform),
    naivePathCertifyMs: stats(naive),
  },
  perScene,
  maxPerSceneAdaptiveRsdPct: perSceneMaxRsd,
  clearanceQuery: throughput,
  note: 'Times are wall-clock from node ' + process.version + ' on 4 vCPU; medians over ' +
    samples + ' runs per scene after one warmup.',
};
writeFileSync(new URL('./result.json', import.meta.url), JSON.stringify(out, null, 1) + '\n');
console.log(JSON.stringify(out, null, 1));
