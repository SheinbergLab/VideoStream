import type { Connection } from "./connection";
import type { FilePicker } from "./filePicker";
import type { RefPicker } from "./refPicker";
import type { PreviewSource, ProbedCamera } from "./protocol";
import { tclDoubleQuoted, tclListArg } from "./tcl";

export { tclDoubleQuoted, tclListArg };

export interface SourceMenuOptions {
  conn: Connection;
  filePicker: FilePicker;
  refPicker: RefPicker;
  wrap: HTMLElement;
  menu: HTMLElement;
  anchorBtn: HTMLButtonElement;
  cameraList: HTMLElement;
  refreshBtn: HTMLButtonElement;
  currentFileBtn: HTMLButtonElement;
  openFileBtn: HTMLElement;
  savedWrap: HTMLElement;
  savedToggle: HTMLButtonElement;
  savedPanel: HTMLElement;
  errorEl: HTMLElement;
  getPlaybackSpeed: () => number;
  getCurrentSource: () => PreviewSource | undefined;
  onSwitchStart?: () => void;
  onSwitchDone?: (source: PreviewSource) => void;
  onSwitchError?: (msg: string) => void;
}

export function attachSourceMenu(opts: SourceMenuOptions): {
  open: () => void;
  close: () => void;
  toggle: () => void;
  isOpen: () => boolean;
  syncPlayback: () => void;
  refreshCameras: () => Promise<void>;
} {
  const showError = (msg: string) => {
    opts.errorEl.textContent = msg;
    opts.errorEl.hidden = !msg;
  };

  const closeSaved = () => {
    opts.savedPanel.hidden = true;
    opts.savedToggle.setAttribute("aria-expanded", "false");
  };

  const openSaved = () => {
    opts.savedPanel.hidden = false;
    opts.savedToggle.setAttribute("aria-expanded", "true");
    const r = opts.savedToggle.getBoundingClientRect();
    opts.savedPanel.style.position = "fixed";
    opts.savedPanel.style.left = `${Math.round(r.right + 6)}px`;
    opts.savedPanel.style.top = `${Math.round(r.top)}px`;
  };

  const close = () => {
    closeSaved();
    opts.menu.hidden = true;
    opts.wrap.classList.remove("open");
    opts.anchorBtn.setAttribute("aria-expanded", "false");
    showError("");
  };

  const syncPlayback = () => {
    const src = opts.getCurrentSource();
    const file = src?.type === "playback" ? src.file : undefined;
    if (!file) {
      opts.currentFileBtn.hidden = true;
      opts.currentFileBtn.classList.remove("selected");
      opts.openFileBtn.textContent = "Open video file\u2026";
    } else {
      opts.currentFileBtn.hidden = false;
      opts.currentFileBtn.textContent = baseName(file);
      opts.currentFileBtn.title = file;
      opts.currentFileBtn.classList.toggle("selected", true);
      opts.openFileBtn.textContent = "Open a different video\u2026";
    }
    markSelectedCameras();
  };

  const open = () => {
    opts.menu.hidden = false;
    opts.wrap.classList.add("open");
    opts.anchorBtn.setAttribute("aria-expanded", "true");
    showError("");
    setListDisabled(false);
    if (cameras) renderCameras(cameras);
    else if (!scan) opts.cameraList.textContent = "Looking for cameras\u2026";
    syncPlayback();
    void opts.refPicker.refresh();
  };

  const toggle = () => {
    if (opts.menu.hidden) open();
    else close();
  };

  const isOpen = () => !opts.menu.hidden || opts.filePicker.isOpen();

  opts.currentFileBtn.addEventListener("click", () => close());

  opts.openFileBtn.addEventListener("click", async () => {
    const path = await opts.filePicker.pick();
    if (path) await openFile(path);
  });

  opts.savedToggle.addEventListener("click", (ev) => {
    ev.stopPropagation();
    if (opts.savedPanel.hidden) openSaved();
    else closeSaved();
  });

  opts.wrap.addEventListener("keydown", (ev) => {
    if (ev.key !== "Escape") return;
    if (!opts.savedPanel.hidden) {
      closeSaved();
      ev.stopPropagation();
      return;
    }
    if (!opts.menu.hidden) {
      close();
      ev.stopPropagation();
    }
  });

  document.addEventListener("click", (ev) => {
    if (opts.menu.hidden) return;
    const t = ev.target as Node;
    if (opts.wrap.contains(t) || opts.filePicker.isOpen()) return;
    close();
  });

  let cameras: ProbedCamera[] | null = null;
  let scan: Promise<void> | null = null;

  function cameraSelected(cam: ProbedCamera): boolean {
    const cur = opts.getCurrentSource();
    if (!cur || cur.type === "playback" || cur.type !== cam.vendor) return false;
    const key = cur.camera_key ?? "";
    const prefix = `${cam.vendor}:`;
    const suffix = key.startsWith(prefix) ? key.slice(prefix.length) : "";
    if (suffix) {
      if (cam.serial && suffix === cam.serial) return true;
      if (suffix === String(cam.id)) return true;
    }
    if (cam.serial && cur.tooltip?.includes(cam.serial)) return true;
    const indexLine = cur.tooltip?.match(/Camera index:\s*(\d+)/);
    if (indexLine && Number(indexLine[1]) === cam.id) return true;
    const idLine = cur.tooltip?.match(/(?:^|\n)id:\s*(\d+)/);
    if (idLine && Number(idLine[1]) === cam.id) return true;
    return cur.label === cam.label;
  }

  function markSelectedCameras() {
    const buttons = opts.cameraList.querySelectorAll<HTMLButtonElement>(".source-menu-item");
    buttons.forEach((btn, i) => {
      btn.classList.toggle("selected", cameras?.[i] ? cameraSelected(cameras[i]) : false);
    });
  }

  async function refreshCameras() {
    if (scan) return scan;
    if (!cameras) opts.cameraList.textContent = "Looking for cameras\u2026";
    opts.refreshBtn.disabled = true;
    scan = (async () => {
      try {
        const sources = await opts.conn.fetchSources();
        cameras = sources.cameras;
        renderCameras(cameras);
        showError("");
      } catch (e) {
        opts.cameraList.textContent = "Could not list cameras.";
        showError(e instanceof Error ? e.message : String(e));
      } finally {
        scan = null;
        opts.refreshBtn.disabled = false;
      }
    })();
    return scan;
  }

  opts.refreshBtn.addEventListener("click", (ev) => {
    ev.stopPropagation();
    void refreshCameras();
  });

  function renderCameras(cameras: ProbedCamera[]) {
    opts.cameraList.replaceChildren();
    if (!cameras.length) {
      const p = document.createElement("p");
      p.className = "source-menu-muted";
      p.textContent = "No cameras detected.";
      opts.cameraList.append(p);
      return;
    }
    for (const cam of cameras) {
      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = "source-menu-item";
      btn.textContent = cam.label;
      if (cameraSelected(cam)) btn.classList.add("selected");
      btn.addEventListener("click", () => void switchCamera(cam, btn));
      opts.cameraList.append(btn);
    }
  }

  function setListDisabled(disabled: boolean) {
    for (const btn of opts.menu.querySelectorAll<HTMLButtonElement>(".source-menu-item")) {
      btn.disabled = disabled;
    }
    (opts.openFileBtn as HTMLButtonElement).disabled = disabled;
  }

  async function switchCamera(cam: ProbedCamera, btn: HTMLButtonElement) {
    let script: string;
    if (cam.vendor === "webcam") {
      script = `switch_to_camera webcam ${cam.id}`;
    } else if (cam.serial) {
      script = `switch_to_camera ${cam.vendor} ${cam.id} ${tclDoubleQuoted(cam.serial)}`;
    } else {
      script = `switch_to_camera ${cam.vendor} ${cam.id}`;
    }
    await runSwitch(script, { type: cam.vendor, label: cam.label }, btn);
  }

  async function openFile(path: string) {
    const spd = opts.getPlaybackSpeed();
    await runSwitch(
      `set_playback_speed ${spd}; switch_to_playback ${tclListArg(path)}`,
      { type: "playback", label: baseName(path), file: path },
      opts.openFileBtn,
    );
  }

  async function runSwitch(script: string, source: PreviewSource, activeBtn: HTMLElement) {
    showError("");
    const previous = activeBtn.textContent;
    activeBtn.textContent = "Connecting\u2026";
    setListDisabled(true);
    opts.onSwitchStart?.();
    try {
      await opts.conn.sendEvalAsync(script);
      activeBtn.textContent = previous;
      setListDisabled(false);
      close();
      opts.onSwitchDone?.(source);
    } catch (e) {
      activeBtn.textContent = previous;
      setListDisabled(false);
      const msg = e instanceof Error ? e.message : String(e);
      open();
      showError(msg);
      opts.onSwitchError?.(msg);
    }
  }

  return { open, close, toggle, isOpen, syncPlayback, refreshCameras };
}

function baseName(path: string): string {
  const i = Math.max(path.lastIndexOf("/"), path.lastIndexOf("\\"));
  return i >= 0 ? path.slice(i + 1) : path;
}
