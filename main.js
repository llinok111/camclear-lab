// Rendering + interaction only. Every number shown comes from src/core.mjs.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import {
  planSafePath, planSafePathAdaptive, meshFromBoxes, clearanceAt,
} from './src/core.mjs';

const app = document.getElementById('app');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
app.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0f14);
scene.fog = new THREE.Fog(0x0b0f14, 22, 46);

const camera = new THREE.PerspectiveCamera(50, innerWidth / innerHeight, 0.1, 200);
camera.position.set(9.5, 8.5, 11);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(3.8, 1.8, 3.6);
controls.enableDamping = true;

scene.add(new THREE.HemisphereLight(0x9fc4ff, 0x1b2430, 1.15));
const key = new THREE.DirectionalLight(0xffffff, 1.5);
key.position.set(6, 12, 8);
scene.add(key);
const grid = new THREE.GridHelper(30, 30, 0x223040, 0x141c26);
grid.position.y = -0.01;
scene.add(grid);

const obstacles = new THREE.Group();
scene.add(obstacles);

const matNaive = new THREE.LineDashedMaterial({ color: 0xff5f5f, dashSize: 0.16, gapSize: 0.12 });
const matRaw = new THREE.LineBasicMaterial({ color: 0x5b6b7c, transparent: true, opacity: 0.7 });
const matPlan = new THREE.MeshStandardMaterial({ color: 0x5ee0a0, emissive: 0x123a2a, roughness: 0.35, metalness: 0.15 });

const naiveLine = new THREE.Line(new THREE.BufferGeometry(), matNaive);
const rawLine = new THREE.Line(new THREE.BufferGeometry(), matRaw);
let planMesh = new THREE.Mesh(new THREE.BufferGeometry(), matPlan);
scene.add(naiveLine, rawLine, planMesh);

const el = (id) => document.getElementById(id);
let scenes = [];
let refs = new Map();
let current = 0;
let mesh = null;
let target = 0.12;

function rebuildObstacles(s) {
  obstacles.clear();
  const geo = new THREE.BoxGeometry(1, 1, 1);
  for (const b of s.boxes) {
    const m = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
      color: 0x33546e, roughness: 0.75, metalness: 0.1,
    }));
    m.position.set(b.center[0], b.center[1], b.center[2]);
    m.scale.set(b.size[0], b.size[1], b.size[2]);
    obstacles.add(m);
  }
  const edgeGeo = new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1));
  for (const b of s.boxes) {
    const e = new THREE.LineSegments(edgeGeo, new THREE.LineBasicMaterial({ color: 0x69a4cc, transparent: true, opacity: 0.45 }));
    e.position.copy(obstacles.children[obstacles.children.length - 1].position);
    e.scale.copy(obstacles.children[obstacles.children.length - 1].scale);
    obstacles.add(e);
  }
}

// Dense independent sampling of the planned polyline, same evaluation the
// node oracle uses, at a lower rate so the UI stays interactive.
function oracleMin(path, m, samples = 600) {
  let best = Infinity;
  const last = path.length - 1;
  for (let k = 0; k <= samples; k++) {
    const x = (k / samples) * last;
    const i = Math.min(last - 1, Math.floor(x));
    const f = x - i;
    const a = path[i], b = path[i + 1];
    const p = [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f];
    const d = clearanceAt(p, m).d;
    if (d < best) best = d;
  }
  return best;
}

function recompute() {
  const s = scenes[current];
  const t0 = performance.now();
  const uniform = planSafePath(s.waypoints, mesh, target);
  const adaptive = planSafePathAdaptive(s.waypoints, mesh, target);
  const ms = performance.now() - t0;

  naiveLine.geometry.dispose();
  naiveLine.geometry = new THREE.BufferGeometry().setFromPoints(
    adaptive.smooth.map((p) => new THREE.Vector3(p[0], p[1], p[2])));
  naiveLine.computeLineDistances();

  rawLine.geometry.dispose();
  rawLine.geometry = new THREE.BufferGeometry().setFromPoints(
    s.waypoints.map((p) => new THREE.Vector3(p[0], p[1], p[2])));

  const curve = new THREE.CatmullRomCurve3(
    adaptive.path.map((p) => new THREE.Vector3(p[0], p[1], p[2])), false, 'catmullrom', 0.5);
  const tube = new THREE.TubeGeometry(curve, 420, 0.045, 7, false);
  planMesh.geometry.dispose();
  planMesh.geometry = tube;

  const oracle = oracleMin(adaptive.path, mesh, 600);
  const ref = refs.get(s.name);
  const nodeOk = ref && Math.abs(ref.adaptiveCert - adaptive.certificate.certified) < 1e-9
    && Math.abs(ref.adaptiveMeanAlpha - adaptive.meanAlpha) < 1e-9;

  el('tv').textContent = target.toFixed(2);
  el('naive').innerHTML = '<span class="bad">' + adaptive.naive.sampledMin.toFixed(3) +
    '</span> <span class="k">/ cert ' + adaptive.naive.certified.toFixed(3) + '</span>';
  el('cert').innerHTML = '<span class="' + (adaptive.feasible ? 'good' : 'bad') + '">' +
    adaptive.certificate.certified.toFixed(4) + '</span> <span class="k">≥ ' + target.toFixed(2) + '</span>';
  el('oracle').textContent = oracle.toFixed(4);
  el('alpha').textContent = adaptive.meanAlpha.toFixed(3) + ' (min ' + adaptive.minAlpha.toFixed(2) + ')';
  el('ualpha').textContent = uniform.alpha.toFixed(3);
  el('ms').textContent = ms.toFixed(1) + ' ms';
  el('abar').style.width = (100 * adaptive.meanAlpha).toFixed(1) + '%';
  el('node').innerHTML = nodeOk
    ? '<span class="good">matches data/ref.json</span>'
    : '<span class="warn">ref mismatch</span>';
}

function show(i) {
  current = i;
  const s = scenes[i];
  mesh = meshFromBoxes(s.boxes);
  rebuildObstacles(s);
  document.querySelectorAll('#scenes button').forEach((b, k) => b.classList.toggle('on', k === i));
  recompute();
}

async function boot() {
  const [sj, rj] = await Promise.all([
    fetch('./data/scenes.json').then((r) => r.json()),
    fetch('./data/ref.json').then((r) => r.json()),
  ]);
  scenes = sj.scenes;
  target = sj.target;
  el('target').value = target;
  for (const g of rj.generated) refs.set(g.name, g);
  const row = el('scenes');
  scenes.forEach((s, i) => {
    const b = document.createElement('button');
    b.textContent = s.name;
    b.onclick = () => show(i);
    row.appendChild(b);
  });
  el('target').addEventListener('input', (e) => {
    target = parseFloat(e.target.value);
    recompute();
  });
  show(0);
}

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});

renderer.setAnimationLoop(() => {
  controls.update();
  renderer.render(scene, camera);
});

boot();
