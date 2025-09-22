// cancel_file.js: Offline comparator using AECM WASM
// Usage: node cancel_file.js <render.wav> <capture.wav> [--no-linear] [--no-nonlinear]

const fs = require('fs');
const assert = require('assert');

const AECMModule = require('./dist/aecm_wasm.js');

const kSr = 16000;
const kBlock = 64;

function rd32le(buf, off) { return buf[off] | (buf[off+1]<<8) | (buf[off+2]<<16) | (buf[off+3]<<24); }
function rd16le(buf, off) { return buf[off] | (buf[off+1]<<8); }

function readWavPcm16Mono16k(path) {
  const buf = fs.readFileSync(path);
  if (buf.length < 44) throw new Error('Too small WAV');
  if (buf.toString('utf8', 0, 4) !== 'RIFF' || buf.toString('utf8', 8, 12) !== 'WAVE') throw new Error('Not RIFF/WAVE');
  let pos = 12; let sr = 0, ch = 0, bps = 0; let dataOff = 0, dataSize = 0;
  while (pos + 8 <= buf.length) {
    const id = rd32le(buf, pos); pos += 4; const sz = rd32le(buf, pos); pos += 4; const start = pos;
    if (id === 0x20746d66) { // 'fmt '
      const fmt = rd16le(buf, start+0); ch = rd16le(buf, start+2); sr = rd32le(buf, start+4); bps = rd16le(buf, start+14);
      if (fmt !== 1 || bps !== 16) throw new Error('Expected PCM16');
    } else if (id === 0x61746164) { // 'data'
      dataOff = start; dataSize = sz; break;
    }
    pos = start + sz;
  }
  if (!dataOff || !dataSize) throw new Error('No data chunk');
  if (sr !== kSr || ch !== 1) throw new Error('Expected 16k mono wav');
  const ns = Math.floor(dataSize / 2);
  const arr = new Int16Array(ns);
  for (let i = 0; i < ns; i++) arr[i] = buf.readInt16LE(dataOff + i*2);
  return arr;
}

function writeWavPcm16Mono16k(path, samples) {
  const ch = 1, bps = 16, sr = kSr;
  const dataBytes = samples.length * 2;
  const byteRate = sr * ch * (bps/8);
  const blockAlign = ch * (bps/8);
  const out = Buffer.alloc(44 + dataBytes);
  out.write('RIFF', 0);
  out.writeUInt32LE(36 + dataBytes, 4);
  out.write('WAVE', 8);
  out.write('fmt ', 12);
  out.writeUInt32LE(16, 16); // fmt size
  out.writeUInt16LE(1, 20);  // PCM
  out.writeUInt16LE(ch, 22);
  out.writeUInt32LE(sr, 24);
  out.writeUInt32LE(byteRate, 28);
  out.writeUInt16LE(blockAlign, 32);
  out.writeUInt16LE(bps, 34);
  out.write('data', 36);
  out.writeUInt32LE(dataBytes, 40);
  for (let i = 0; i < samples.length; i++) out.writeInt16LE(samples[i], 44 + i*2);
  fs.writeFileSync(path, out);
}

(async () => {
  const args = process.argv.slice(2);
  if (args.length < 2) {
    console.error('Usage: node cancel_file.js <render.wav> <capture.wav> [--no-linear] [--no-nonlinear]');
    process.exit(1);
  }
  const renderPath = args[0];
  const capturePath = args[1];
  let enableLinear = true, enableNonlinear = true;
  for (const a of args.slice(2)) {
    if (a === '--no-linear') enableLinear = false;
    else if (a === '--no-nonlinear') enableNonlinear = false;
  }

  const x = readWavPcm16Mono16k(renderPath);
  const y = readWavPcm16Mono16k(capturePath);
  const N = Math.min(Math.floor(x.length / kBlock), Math.floor(y.length / kBlock));

  const mod = await AECMModule();
  const h = mod._aecm_create();
  mod._aecm_set_bypass_supmask(h, enableLinear ? 0 : 1);
  mod._aecm_set_bypass_nlp(h, enableNonlinear ? 0 : 1);

  const bytes = kBlock * 2;
  const pRef = mod._malloc(bytes);
  const pCap = mod._malloc(bytes);
  const pOut = mod._malloc(bytes);

  const processed = new Int16Array(N * kBlock);

  for (let n = 0; n < N; n++) {
    const ref = x.subarray(n*kBlock, (n+1)*kBlock);
    const cap = y.subarray(n*kBlock, (n+1)*kBlock);
    mod.HEAP16.set(ref, pRef >> 1);
    mod.HEAP16.set(cap, pCap >> 1);
    const status = mod._aecm_process(h, pRef, pCap, pOut);
    let outView;
    if (status === 0) {
      outView = mod.HEAP16.subarray(pOut >> 1, (pOut >> 1) + kBlock);
    } else {
      outView = cap;
    }
    processed.set(outView, n*kBlock);

    // Simple metrics similar in spirit to C++ version
    let y2 = 0, e2 = 0;
    for (let i = 0; i < kBlock; i++) { const yy = cap[i]; const ee = outView[i]; y2 += yy*yy; e2 += ee*ee; }
    const ratio = y2 > 0 ? (e2 / y2) : 0;
    const dblk = mod._aecm_get_last_delay_blocks(h);
    const dms = dblk >= 0 ? (dblk * (1000.0 * kBlock / kSr)) : -1;
    console.log(`block=${n} y2=${y2.toExponential()} e2=${e2.toExponential()} e2_over_y2=${ratio.toExponential()} est_delay_blocks=${dblk} est_delay_ms=${dms.toFixed(3)}`);
  }

  writeWavPcm16Mono16k('processed.wav', processed);

  mod._free(pRef); mod._free(pCap); mod._free(pOut);
  mod._aecm_destroy(h);
})();
