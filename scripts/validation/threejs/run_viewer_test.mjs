// run_viewer_test.mjs -- SCRIPT_VERSION 1.0
// Purpose: load robot_vNN.glb in real Three.js (GLTFLoader + AnimationMixer + WebGL, headless Chromium) and compare it with Blender.
// Usage:   node run_viewer_test.mjs GLB REFERENCE.json OUT_DIR [REPORT.md]      (exit 0 = all checks pass)
// Needs:   npm install (three, pinned) in this folder; Playwright from the global install.
import http from "node:http";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { createRequire } from "node:module";
const require = createRequire(import.meta.url);
const globalRoot = process.env.NODE_PATH || "/opt/node22/lib/node_modules";
const { chromium } = require(path.join(globalRoot, "playwright"));

const here = path.dirname(fileURLToPath(import.meta.url));
const [glb, refPath, outDir, reportPath] = process.argv.slice(2);
const ref = JSON.parse(fs.readFileSync(refPath, "utf8"));
fs.mkdirSync(outDir, { recursive: true });

const types = { ".html": "text/html", ".js": "text/javascript", ".mjs": "text/javascript", ".json": "application/json", ".glb": "model/gltf-binary" };
const server = http.createServer((req, res) => {
  const u = decodeURIComponent(req.url.split("?")[0]);
  let f = u === "/" || u === "/viewer_test.html" ? path.join(here, "viewer_test.html")
        : u.startsWith("/three/") ? path.join(here, "node_modules", u)
        : u === "/robot.glb" ? path.resolve(glb) : null;
  if (u.startsWith("/three/")) f = path.join(here, "node_modules", "three", u.slice(7));
  if (!f || !fs.existsSync(f)) { res.writeHead(404); res.end(); return; }
  res.writeHead(200, { "content-type": types[path.extname(f)] || "application/octet-stream" }); fs.createReadStream(f).pipe(res);
});
await new Promise(r => server.listen(0, "127.0.0.1", r));
const base = `http://127.0.0.1:${server.address().port}`;

const browser = await chromium.launch({ args: ["--use-angle=swiftshader", "--use-gl=angle", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"] });
const page = await browser.newPage({ viewport: { width: 640, height: 900 } });
const errors = [];
page.on("pageerror", e => errors.push(String(e)));
page.on("console", m => { if (m.type() === "error") errors.push(m.text()); });
await page.goto(`${base}/viewer_test.html?glb=/robot.glb`);
await page.waitForFunction("window.__ready === true", null, { timeout: 120000 });
const gpu = await page.evaluate(() => { const c = document.createElement("canvas"); const g = c.getContext("webgl2"); const e = g && g.getExtension("WEBGL_debug_renderer_info"); return g ? (e ? g.getParameter(e.UNMASKED_RENDERER_WEBGL) : "webgl2") : "none"; });
await page.evaluate(f => window.api.setFps(f), ref.fps);

const info = await page.evaluate(() => window.api.info());
const bones = await page.evaluate(r => window.api.boneErrors(r), ref);
const boxes = await page.evaluate(r => window.api.meshBoxes(r), ref);
const band = ref.waist_band ? await page.evaluate(r => window.api.waistBand(r), ref) : null;
const size = await page.evaluate(() => window.api.robotHeight());
const stats0 = await page.evaluate(() => window.api.pixelStats(0));
await page.screenshot({ path: path.join(outDir, "threejs_frame00.png") });
const stats10 = await page.evaluate(() => window.api.pixelStats(10));
await page.screenshot({ path: path.join(outDir, "threejs_frame10.png") });
const changed = await page.evaluate(() => window.api.diff(0, 10));
await browser.close(); server.close();

const checks = [];
const chk = (name, ok, detail) => checks.push({ name, ok, detail });
chk("GLB loads in GLTFLoader without page errors", errors.length === 0, errors.length ? errors.join(" | ").slice(0, 200) : "no errors");
chk("WebGL2 context available", gpu !== "none", gpu);
chk("56 skeleton joints, all DEF_", info.joints.length === 56 && info.nonDefJoints.length === 0, `${info.joints.length} joints, non-DEF ${info.nonDefJoints.length}`);
chk("every mesh is skinned (SkinnedMesh) with skin attributes", info.skinnedMeshes >= 100 && info.missingSkinAttrs.length === 0, `${info.skinnedMeshes} skinned meshes`);
chk("no NaN vertex positions; every vertex has unit skin weight", info.nanPositions === 0 && info.weightVertsNotUnit === 0, `${info.weightVertsChecked} vertices, ${info.weightVertsNotUnit} not unit, ${info.nanPositions} NaN`);
chk("one animation clip driving 56 nodes", info.clips === 1 && info.trackedNodes === 56, `${info.clipName}: ${info.tracks} tracks, ${info.trackedNodes} nodes, ${info.duration.toFixed(3)} s`);
const expectDur = (ref.frame_range[1] - ref.frame_range[0]) / ref.fps;
chk("clip duration equals the Blender action length", Math.abs(info.duration - expectDur) < 0.05, `${info.duration.toFixed(3)} s vs ${expectDur.toFixed(3)} s`);
chk("bone world positions match Blender (all DEF bones, every 2nd frame)", bones.worst < 1e-4, `${bones.compared} comparisons, worst ${bones.worst.toExponential(2)} m at ${bones.worstWhere}`);
chk("skinned mesh bounding boxes match Blender (torso, head, hands, thigh, shin, pelvis, feet, visor)", boxes.worst < 2e-3 && boxes.missing.length === 0, `${boxes.count} boxes, worst ${boxes.worst.toExponential(2)} m at ${boxes.where}, missing ${boxes.missing.length}`);
if (band) chk("torso waist band (z 1.05-1.17) matches Blender: skinned vertex extents", band.err < 1e-3 && band.n > 100, `${band.n} vertices, worst ${band.err.toExponential(2)} m (waist half-width ${band.max[0].toFixed(4)} m)`);
chk("robot height is plausible (2.2-2.5 m)", size.height > 2.2 && size.height < 2.5, `${size.height.toFixed(3)} m`);
chk("WebGL render shows the robot", stats0.foreground > 0.02 * stats0.total && stats0.foreground < 0.5 * stats0.total, `${(100 * stats0.foreground / stats0.total).toFixed(1)} % foreground pixels`);
chk("pose changes between frame 0 and frame 10 in the rendered image", changed > 200, `${changed} pixels changed`);

const lines = [`# Three.js viewer test`, ``, `three 0.170.0, headless Chromium, renderer: ${gpu}`, ``, `| check | result | detail |`, `|---|---|---|`,
  ...checks.map(c => `| ${c.name} | ${c.ok ? "PASS" : "**FAIL**"} | ${c.detail} |`), ``,
  `Screenshots: threejs_frame00.png (rest), threejs_frame10.png (hello_sequence peak).`, ``,
  `**${checks.every(c => c.ok) ? "ALL VIEWER CHECKS PASSED" : "FAILED: " + checks.filter(c => !c.ok).map(c => c.name).join("; ")}**`];
console.log(lines.join("\n"));
if (reportPath) fs.writeFileSync(reportPath, lines.join("\n") + "\n");
process.exit(checks.every(c => c.ok) ? 0 : 1);
