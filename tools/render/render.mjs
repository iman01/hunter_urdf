// Render PNG previews of the HUNTeR URDF with three.js in headless Chromium.
//
//   cd tools/render && npm install three@0.160.0 urdf-loader@0.12.3 playwright
//   xacro ../../hunter_description/urdf/hunter.urdf.xacro > /tmp/robot.urdf
//   node render.mjs /tmp/robot.urdf ../../hunter_description ../../docs/images  [views.json]
//
// views.json: [{"name": "iso", "view": "35,20,9,0,0,1", "joints": "front_left_steer_joint:0.3"}, ...]
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const here = path.dirname(fileURLToPath(import.meta.url));
const [urdfPath, descDir, outDir, viewsFile] = process.argv.slice(2);
if (!urdfPath || !descDir || !outDir) {
  console.error('usage: node render.mjs <robot.urdf> <hunter_description dir> <out dir> [views.json]');
  process.exit(2);
}
const nodeModules = process.env.NODE_MODULES || path.join(here, 'node_modules');
const views = viewsFile ? JSON.parse(fs.readFileSync(viewsFile, 'utf8')) : [
  { name: 'hunter_iso', view: '35,18,10.5,0,0,0.9' },
];
fs.mkdirSync(outDir, { recursive: true });

const types = { '.html': 'text/html', '.js': 'text/javascript', '.mjs': 'text/javascript', '.stl': 'model/stl',
                '.urdf': 'text/xml', '.dae': 'model/vnd.collada+xml' };
const server = http.createServer((req, res) => {
  const u = decodeURIComponent(new URL(req.url, 'http://x').pathname);
  let f;
  if (u === '/' || u === '/viewer.html') f = path.join(here, 'viewer.html');
  else if (u === '/robot.urdf') f = urdfPath;
  else if (u.startsWith('/node_modules/')) f = path.join(nodeModules, u.slice('/node_modules/'.length));
  else if (u.startsWith('/hunter_description/')) f = path.join(descDir, u.slice('/hunter_description/'.length));
  if (!f || !fs.existsSync(f)) { res.writeHead(404); res.end(); return; }
  res.writeHead(200, { 'Content-Type': types[path.extname(f)] || 'application/octet-stream' });
  fs.createReadStream(f).pipe(res);
});
await new Promise(r => server.listen(0, '127.0.0.1', r));
const port = server.address().port;

const browser = await chromium.launch({ args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const page = await browser.newPage({ viewport: { width: 1600, height: 1000 } });
page.on('console', m => { if (m.type() === 'error') console.error('page:', m.text()); });
for (const v of views) {
  const qs = new URLSearchParams({ view: v.view, joints: v.joints || '', w: v.w || 1600, h: v.h || 1000,
                                   fov: v.fov || 30, grid: v.grid ?? '1', z: v.z || 0,
                                   hide: v.hide || '', ground: v.ground ?? '1' });
  await page.setViewportSize({ width: +(v.w || 1600), height: +(v.h || 1000) });
  await page.goto(`http://127.0.0.1:${port}/viewer.html?${qs}`);
  await page.waitForFunction(() => window.__ready === true, null, { timeout: 120000 });
  const out = path.join(outDir, `${v.name}.png`);
  await page.locator('canvas').screenshot({ path: out });
  console.log('wrote', out, await page.title());
}
await browser.close();
server.close();
