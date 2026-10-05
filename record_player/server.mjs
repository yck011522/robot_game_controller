/** Local read-only viewer server. Run: npm start | node server.mjs --port 4318 --open
 * Optional environment: RECORD_PLAYER_PYTHON=/path/to/venv/python. Binds loopback only.
 */
import http from 'node:http';
import {readFile, stat} from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {spawn} from 'node:child_process';
import {gzipSync} from 'node:zlib';

const here = path.dirname(fileURLToPath(import.meta.url)); // App root, independent of working directory.
const python = process.env.RECORD_PLAYER_PYTHON || path.join(here, '.venv', process.platform === 'win32' ? 'Scripts/python.exe' : 'bin/python'); // Isolated adapter interpreter.
const portIndex = process.argv.indexOf('--port'); // Optional CLI port argument position.
const port = Number(portIndex >= 0 ? process.argv[portIndex+1] : 4317); // Local server port.
const cache = new Map(); // Bounded cache for expensive game/scene conversions; restart to refresh sources.
const inflight = new Map(); // Coalesce duplicate requests for the same game.
const types = {'.html':'text/html', '.js':'text/javascript', '.mjs':'text/javascript', '.css':'text/css', '.json':'application/json'}; // Static MIME types.

/** Run the Python adapter without a shell; called by API handlers and shared across duplicate requests. */
function convert(action, id = '') {
  const key = `${action}:${id}`; // Conversion cache key.
  if (action !== 'index' && cache.has(key)) return Promise.resolve(cache.get(key));
  if (inflight.has(key)) return inflight.get(key);
  const pending = new Promise((resolve, reject) => {
    const child = spawn(python, [path.join(here, 'data.py'), action, ...(id ? [id] : [])], {cwd:here, windowsHide:true}); // Read-only conversion child.
    const chunks = []; // UTF-8 stdout chunks.
    let errorText = ''; // Bounded diagnostic stderr returned on failures.
    const timer = setTimeout(() => { child.kill(); reject(new Error('Recording conversion timed out after 120 seconds')); }, 120000); // Conversion watchdog.
    child.stdout.on('data', chunk => chunks.push(chunk));
    child.stderr.on('data', chunk => { errorText = (errorText + chunk).slice(-6000); });
    child.on('error', error => { clearTimeout(timer); reject(error); });
    child.on('close', code => {
      clearTimeout(timer);
      if (code !== 0) return reject(new Error(errorText || `Adapter exited ${code}`));
      const body = Buffer.concat(chunks); // Serialized conversion output.
      if (action !== 'index') {
        if (cache.size >= 4) cache.delete(cache.keys().next().value);
        cache.set(key, body);
      }
      resolve(body);
    });
  }).finally(() => inflight.delete(key)); // Allow retries after failures.
  inflight.set(key, pending);
  return pending;
}

/** Serve one local request; expose only public assets and three.js, never arbitrary repository files. */
async function handle(request, response) {
  try {
    if (request.method !== 'GET') { response.writeHead(405); response.end(); return; }
    const url = new URL(request.url, `http://127.0.0.1:${port}`); // Parsed request URL.
    let body; // Buffer sent to the browser.
    let mime = 'application/json'; // API default content type.
    if (url.pathname === '/api/index') body = await convert('index');
    else if (url.pathname === '/api/scene') body = await convert('scene');
    else if (url.pathname === '/api/game') {
      const id = url.searchParams.get('id') || ''; // Date/time recording ID, not a filesystem path.
      if (!/^\d{4}-\d{2}-\d{2}\/\d{2}-\d{2}-\d{2}(?:_\d+)?$/.test(id)) { response.writeHead(400); response.end('Invalid recording ID'); return; }
      body = await convert('game', id);
    } else {
      const vendor = url.pathname.startsWith('/vendor/'); // Restrict vendor exposure to Three.js.
      const root = path.join(here, vendor ? 'node_modules/three' : 'public'); // Allowed static root.
      const relative = decodeURIComponent(vendor ? url.pathname.slice(8) : url.pathname === '/' ? 'index.html' : url.pathname.slice(1)); // Relative static path.
      const file = path.resolve(root, relative); // Canonical static file path.
      if (!file.startsWith(root + path.sep) || !(await stat(file)).isFile()) { response.writeHead(404); response.end('Not found'); return; }
      body = await readFile(file);
      mime = types[path.extname(file)] || 'application/octet-stream';
    }
    const zipped = body.length > 2048 && /gzip/.test(request.headers['accept-encoding'] || ''); // Compress large trace payloads.
    response.writeHead(200, {'Content-Type':mime+'; charset=utf-8', 'Cache-Control':'no-cache', 'X-Content-Type-Options':'nosniff', ...(zipped ? {'Content-Encoding':'gzip'} : {})});
    response.end(zipped ? gzipSync(body) : body);
  } catch (error) {
    response.writeHead(error.code === 'ENOENT' ? 404 : 500, {'Content-Type':'application/json'});
    response.end(JSON.stringify({error:error.message}));
  }
}

if (!Number.isInteger(port) || port < 1024 || port > 65535) throw new Error('Port must be an integer from 1024 to 65535');
const server = http.createServer(handle); // Loopback-only offline viewer.
server.on('error', error => { console.error(error.message); process.exitCode = 1; });
server.listen(port, '127.0.0.1', () => {
  const address = `http://127.0.0.1:${port}`; // Browser entry point.
  console.log(`Record Player ready: ${address}\nPress Ctrl+C to stop.`);
  if (process.argv.includes('--open') && process.platform === 'win32') spawn('rundll32.exe', ['url.dll,FileProtocolHandler', address], {windowsHide:true});
});
