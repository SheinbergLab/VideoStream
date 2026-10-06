// A client that subscribes to preview and then stops reading its socket, to
// exercise the server's backpressure path.
//   node scripts/stall.mjs [host] [port] [seconds]
import net from "node:net";
import crypto from "node:crypto";

const host = process.argv[2] ?? "localhost";
const port = Number(process.argv[3] ?? 8080);
const seconds = Number(process.argv[4] ?? 15);

const sock = net.connect(port, host, () => {
  const key = crypto.randomBytes(16).toString("base64");
  sock.write(
    `GET /ws HTTP/1.1\r\nHost: ${host}:${port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n` +
      `Sec-WebSocket-Key: ${key}\r\nSec-WebSocket-Version: 13\r\n\r\n`,
  );
});

let upgraded = false;
let received = 0;
sock.on("data", (d) => {
  received += d.length;
  if (upgraded) return;
  upgraded = true;
  const payload = Buffer.from(JSON.stringify({ cmd: "preview", enable: true, fps: 60 }));
  const mask = crypto.randomBytes(4);
  const masked = Buffer.from(payload.map((b, i) => b ^ mask[i % 4]));
  sock.write(Buffer.concat([Buffer.from([0x81, 0x80 | payload.length]), mask, masked]));
  // Let a little arrive, then stop reading entirely.
  setTimeout(() => {
    sock.pause();
    console.log(`paused after ${(received / 1024).toFixed(0)} KB`);
  }, 500);
});
sock.on("close", () => console.log("server closed the stalled connection"));
setTimeout(() => {
  console.log(`still open after ${seconds}s: ${!sock.destroyed}`);
  sock.destroy();
  process.exit(0);
}, seconds * 1000);
