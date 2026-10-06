// Connects like the viewer, subscribes to preview, and reports the stream.
//   node scripts/probe.mjs [ws://localhost:8080/ws] [fps] [seconds]
const url = process.argv[2] ?? "ws://localhost:8080/ws";
const fps = Number(process.argv[3] ?? 30);
const seconds = Number(process.argv[4] ?? 5);

const ws = new WebSocket(url);
ws.binaryType = "arraybuffer";
let frames = 0, bytes = 0, first = null, last = null, gaps = 0, lags = [];
const t0 = performance.now();

ws.onopen = () => ws.send(JSON.stringify({ cmd: "preview", enable: true, fps }));
ws.onmessage = (ev) => {
  if (typeof ev.data === "string") {
    if (ev.data.includes('"preview"')) console.log("ack:", ev.data);
    return;
  }
  const buf = ev.data;
  const len = new DataView(buf).getUint32(0, true);
  const h = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 4, len)));
  const jpeg = new Uint8Array(buf, 4 + len);
  if (jpeg[0] !== 0xff || jpeg[1] !== 0xd8) console.log("bad JPEG magic");
  if (last && h.seq !== last.seq + 1) gaps += h.seq - last.seq - 1;
  const et = h.overlay.eye_tracking;
  if (et?.valid) lags.push(h.frame_id - et.frame_id);
  frames++;
  bytes += buf.byteLength;
  first ??= h;
  last = h;
};
setTimeout(() => {
  const dt = (performance.now() - t0) / 1000;
  const { overlay, ...rest } = last ?? {};
  console.log(JSON.stringify(rest));
  console.log("overlay keys:", Object.keys(overlay ?? {}));
  console.log(JSON.stringify(overlay));
  lags.sort((a, b) => a - b);
  if (!lags.length) lags.push(NaN);
  console.log(
    `frames ${frames} in ${dt.toFixed(1)}s = ${(frames / dt).toFixed(1)} fps, ` +
      `${(bytes / 1024 / dt).toFixed(0)} KB/s, avg ${(bytes / frames / 1024).toFixed(1)} KB/frame, ` +
      `seq gaps ${gaps}, lag median ${lags[lags.length >> 1]} max ${lags[lags.length - 1]}`,
  );
  ws.close();
  process.exit(0);
}, seconds * 1000);
