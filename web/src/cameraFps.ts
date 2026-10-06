import type { Connection } from "./connection";

// FLIR reports the achieved rate as AcquisitionResultingFrameRate; Lucid has
// no such node, and its AcquisitionFrameRate (enabled) is the running rate.
const FPS_NODES = ["AcquisitionResultingFrameRate", "AcquisitionFrameRate"];

/** Reads the camera's running frame rate, remembering which node it has. Call reset() when the camera changes. */
export function cameraFpsReader(conn: Connection) {
  let index = 0;
  return {
    reset(): void {
      index = 0;
    },
    async read(): Promise<number> {
      while (index < FPS_NODES.length) {
        try {
          return Number(await conn.sendEvalAsync(`camera::node ${FPS_NODES[index]}`));
        } catch (e) {
          // Only a missing node moves on to the next name.
          if (!String(e).includes("no such node")) throw e;
          index++;
        }
      }
      throw new Error("camera reports no frame rate");
    },
  };
}
