// Wire format shared with VideoStream (WebPreview.cpp, EyeTrackingPlugin
// getOverlayJSON). Preview frames are binary WebSocket messages:
//   [uint32 little-endian header_len][header JSON][JPEG bytes]

export interface Point {
  x: number;
  y: number;
}

export interface EyeTrackingOverlay {
  mode: "full" | "pupil_p1" | "pupil_only";
  focus_mode: number;
  valid: boolean;
  roi?: { x: number; y: number; w: number; h: number };
  // Ring-buffer slot and source frame id of the frame these results came
  // from; FrameHeader.frame_id minus frame_id is the overlay lag. Absent
  // when !valid.
  analysis_frame?: number;
  frame_id?: number;
  abs_frame_id?: number;
  in_blink?: boolean;
  tracking_lost?: boolean;
  roi_violation?: boolean;
  pupil?: {
    detected: boolean;
    x?: number;
    y?: number;
    r?: number;
    /** fitEllipse center (full frame); falls back to x,y if absent. */
    ex?: number;
    ey?: number;
    /** Semi-axes size.width/2 and size.height/2 from OpenCV RotatedRect. */
    a?: number;
    b?: number;
    /** fitEllipse angle (degrees), width axis from horizontal. */
    angle?: number;
  };
  p1?: { detected: boolean; x?: number; y?: number; intensity?: number };
  p4?: { detected: boolean; x?: number; y?: number };
  // Centre and size of the P4 search window.
  p4_predicted?: { x: number; y: number; w: number; h: number };
  p4_model?: { initialized: boolean; frozen: boolean; samples: number };
  // Recorded detections for this frame from a loaded session .db.
  reference?: {
    /** a/b = stored semi-major/minor axes; angle = minor-axis direction (deg). */
    pupil?: Point & { r: number; a?: number; b?: number; angle?: number };
    p1?: Point;
    p4?: Point;
    blink: boolean;
  };
}

export interface PreviewSource {
  type: string;
  label: string;
  tooltip?: string;
  /** Playback file path, from the picker or the "Recorded file" tooltip line. */
  file?: string;
  /** File playback rate (0.25–2). */
  speed?: number;
  /** Stable id for FLIR/Lucid cameras (persist gain per device). */
  camera_key?: string;
}

export interface SourceCapabilities {
  playback: boolean;
  webcam: boolean;
  flir: boolean;
  lucid: boolean;
}

export interface ProbedCamera {
  vendor: string;
  id: number;
  label: string;
  model?: string;
  serial?: string;
}

export interface SourcesResponse {
  type: "sources";
  status: string;
  capabilities: SourceCapabilities;
  cameras: ProbedCamera[];
  source_active?: boolean;
  requestId?: string;
  error?: string;
}

export interface BrowseEntry {
  name: string;
  path: string;
  dir: boolean;
  size?: number;
  /** Unix seconds. */
  mtime?: number;
}

export interface BrowsePlace {
  label: string;
  path: string;
}

export interface BrowseResponse {
  type: "browse";
  status: string;
  path?: string;
  parent?: string | null;
  entries?: BrowseEntry[];
  places?: BrowsePlace[];
  error?: string;
  requestId?: string;
}

export interface FrameHeader {
  type: "frame";
  seq: number;
  video_frame: number;
  frame_id: number;
  ring_index: number;
  ring_size: number;
  ts_us: number;
  width: number;
  height: number;
  channels: number;
  src_fps: number;
  /** Incomplete frames discarded between the camera and the server. */
  incomplete_frames?: number;
  /** VideoStream CPU, 100 = one core. Absent until a second of samples exists. */
  proc_cpu?: number;
  /** Whole machine busy percent. */
  host_cpu?: number;
  rss_kb?: number;
  mem_avail_kb?: number;
  in_obs: boolean;
  encode_ms: number;
  source?: PreviewSource;
  // Keyed by registered plugin name.
  overlay: { eye_tracking?: EyeTrackingOverlay; [plugin: string]: unknown };
}

export interface PreviewCommand {
  cmd: "preview";
  enable: boolean;
  fps?: number;
  quality?: number;
}

export interface FrameMessage {
  header: FrameHeader;
  jpeg: Uint8Array;
  bytes: number;
}

const decoder = new TextDecoder();

export function parseFrameMessage(buf: ArrayBuffer): FrameMessage | null {
  if (buf.byteLength < 4) return null;
  const headerLen = new DataView(buf).getUint32(0, true);
  if (4 + headerLen > buf.byteLength) return null;
  let header: FrameHeader;
  try {
    header = JSON.parse(decoder.decode(new Uint8Array(buf, 4, headerLen)));
  } catch {
    return null;
  }
  if (header.type !== "frame") return null;
  return { header, jpeg: new Uint8Array(buf, 4 + headerLen), bytes: buf.byteLength };
}
