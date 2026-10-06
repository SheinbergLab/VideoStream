import { defineConfig } from "vite";

// VideoStream serves the build at /app/. In dev, /ws goes to VideoStream
// (in WSL, reachable on Windows localhost); override with VS_TARGET.
const target = process.env.VS_TARGET ?? "ws://127.0.0.1:8080";

export default defineConfig({
  base: "/app/",
  server: {
    proxy: {
      "/ws": { target, ws: true },
    },
  },
  build: {
    outDir: "dist",
    emptyOutDir: true,
    sourcemap: true,
  },
});
