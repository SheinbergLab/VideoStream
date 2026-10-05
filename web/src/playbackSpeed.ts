export const PLAYBACK_SPEEDS = [0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75, 2] as const;

export type PlaybackSpeed = (typeof PLAYBACK_SPEEDS)[number];

export const PLAYBACK_SPEED_KEY = "vs.viewer.playbackSpeed";

export function loadPlaybackSpeedPref(): number {
  try {
    const raw = localStorage.getItem(PLAYBACK_SPEED_KEY);
    if (raw == null) return 1;
    const n = Number(raw);
    return PLAYBACK_SPEEDS.includes(n as PlaybackSpeed) ? n : 1;
  } catch {
    return 1;
  }
}

export function savePlaybackSpeedPref(speed: number): void {
  localStorage.setItem(PLAYBACK_SPEED_KEY, String(speed));
}

/** Button label suffix, e.g. "0.25x", "1x". */
export function formatPlaybackSpeed(speed: number): string {
  const s = speed.toFixed(2).replace(/\.?0+$/, "");
  return `${s}x`;
}

export function nearestPlaybackSpeed(speed: number): PlaybackSpeed {
  let best: PlaybackSpeed = 1;
  let d = Infinity;
  for (const v of PLAYBACK_SPEEDS) {
    const diff = Math.abs(v - speed);
    if (diff < d) {
      d = diff;
      best = v;
    }
  }
  return best;
}
