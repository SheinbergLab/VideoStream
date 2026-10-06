import { defineConfig } from "vite";
import { fileURLToPath } from "node:url";

// Each browser app is a page in its own folder, served by VideoStream at
// /<app>/ (e.g. /eyetracker/); add a new app by adding its folder here.
// Shared assets/ and public/ files sit at the root. In dev
// (http://localhost:5173/eyetracker/), /ws goes to VideoStream (in WSL,
// reachable on Windows localhost); override with VS_TARGET.
const target = process.env.VS_TARGET ?? "ws://127.0.0.1:8080";

const apps = ["eyetracker"];

export default defineConfig({
  base: "/",
  server: {
    proxy: {
      "/ws": { target, ws: true },
    },
  },
  build: {
    outDir: "dist",
    emptyOutDir: true,
    sourcemap: true,
    rollupOptions: {
      input: Object.fromEntries(
        apps.map((a) => [a, fileURLToPath(new URL(`${a}/index.html`, import.meta.url))]),
      ),
    },
  },
});
