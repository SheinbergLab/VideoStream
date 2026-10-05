import type { Connection } from "./connection";
import type { Badge } from "./overlay/eyetracking";
import { parseTclDictNested, parseTclList, tclListArg } from "./tcl";

/** Matches serve.tcl ::S::run_state. */
export type RunState = "idle" | "running" | "closing" | "done";

export const REF_NONE = "none";
export const REF_CREATE = "__create__";
const RUN_PREVIEW_FPS = 10;
const POLL_MS = 500;

export interface RefPickerOptions {
  conn: Connection;
  row: HTMLElement;
  refSection: HTMLElement;
  refList: HTMLElement;
  choiceEl: HTMLElement;
  reprocessBtn: HTMLButtonElement;
  saveBtn: HTMLButtonElement;
  showError: (msg: string) => void;
  /** Called whenever run state or progress changes (badges, step lock). */
  onChange?: () => void;
  /** Re-render reference items in the source menu when open. */
  onMenuUpdate?: () => void;
  /** Close the source menu after picking a stored reference file. */
  onRefPicked?: () => void;
}

export interface RefPicker {
  refresh: () => Promise<void>;
  runState: () => RunState;
  badge: () => Badge | null;
  renderMenu: () => void;
  selection: () => string;
  choiceLabel: () => string;
}

function baseName(path: string): string {
  const i = path.lastIndexOf("/");
  return i >= 0 ? path.slice(i + 1) : path;
}

/**
 * Reference .db choices for the current pre-recorded video: stored files,
 * None, and Create new (Re-process the video, then Save).
 */
export function attachRefPicker(opts: RefPickerOptions): RefPicker {
  let state: RunState = "idle";
  let frame = 0;
  let total = 0;
  let nextName = "";
  let lastValue = REF_NONE;
  let pollTimer: number | undefined;
  let busy = false;
  const pct = () => (total > 0 ? Math.min(100, Math.round((100 * frame) / total)) : 0);

  function setState(next: RunState) {
    const was = state;
    state = next;
    const active = next === "running" || next === "closing";
    if (active && pollTimer === undefined) {
      opts.conn.setPreviewFps(RUN_PREVIEW_FPS);
      pollTimer = window.setInterval(() => void poll(), POLL_MS);
    } else if (!active && pollTimer !== undefined) {
      window.clearInterval(pollTimer);
      pollTimer = undefined;
      opts.conn.setPreviewFps(null);
    }
    if (was !== next) opts.onChange?.();
  }

  function statusText(): string {
    if (state === "running") return `Re-processing\u2026 ${pct()}% (frame ${frame} of ${total})`;
    if (state === "closing") return "Finishing re-process\u2026";
    if (state === "done") return `Re-process finished. Save to keep it as ${nextName}.`;
    if (lastValue === REF_CREATE) return `Re-process will write ${nextName}.`;
    return "Saved tracking shown on the picture.";
  }

  function choiceLabel(): string {
    if (lastValue === REF_CREATE) return "Create new\u2026";
    if (lastValue === REF_NONE || !lastValue) return "None";
    return baseName(lastValue);
  }

  function renderButtons() {
    const creating = lastValue === REF_CREATE;
    opts.reprocessBtn.hidden = !creating;
    opts.saveBtn.hidden = !(creating && state === "done");
    opts.reprocessBtn.textContent = state === "running" ? "Restart" : "Re-process";
    opts.reprocessBtn.title =
      "Run the whole video from the start through the current settings" +
      (nextName ? ` (saves as ${nextName})` : "");
    opts.saveBtn.title = nextName ? `Save as ${nextName}` : "Save";
    opts.refSection.title = statusText();
    opts.choiceEl.textContent = choiceLabel();
  }

  function renderMenu() {
    const selected = lastValue;
    for (const btn of opts.refList.querySelectorAll<HTMLButtonElement>(".source-menu-item")) {
      btn.classList.toggle("selected", btn.dataset.ref === selected);
    }
  }

  function populate(files: string[], current: string) {
    opts.refList.replaceChildren();
    for (const f of files) {
      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = "source-menu-item";
      btn.dataset.ref = f;
      btn.textContent = baseName(f);
      btn.addEventListener("click", () => void pickRef(f));
      opts.refList.append(btn);
    }
    const none = document.createElement("button");
    none.type = "button";
    none.className = "source-menu-item";
    none.dataset.ref = REF_NONE;
    none.textContent = "None";
    none.addEventListener("click", () => void pickRef(REF_NONE));
    opts.refList.append(none);
    const create = document.createElement("button");
    create.type = "button";
    create.className = "source-menu-item";
    create.dataset.ref = REF_CREATE;
    create.textContent = "Create new\u2026";
    create.addEventListener("click", () => void pickRef(REF_CREATE));
    opts.refList.append(create);

    if (state !== "idle" || lastValue === REF_CREATE) lastValue = REF_CREATE;
    else if (current && files.includes(current)) lastValue = current;
    else lastValue = REF_NONE;

    renderMenu();
    renderButtons();
  }

  async function pickRef(value: string) {
    if (lastValue === REF_CREATE && value !== REF_CREATE && state !== "idle") {
      if (!window.confirm("Discard the unsaved re-process run?")) return;
      const nextRef = value === REF_CREATE ? "keep" : value;
      await run(`run_discard ${tclListArg(nextRef)}`);
      lastValue = value;
      await refresh();
      return;
    }
    lastValue = value;
    if (value === REF_CREATE) {
      renderMenu();
      renderButtons();
      opts.onMenuUpdate?.();
      return;
    }
    await run(`ref_select ${tclListArg(value)}`);
    renderMenu();
    renderButtons();
    opts.onRefPicked?.();
  }

  async function refresh() {
    let raw = "";
    try {
      raw = await opts.conn.sendEvalAsync("ref_list");
    } catch {
      raw = "";
    }
    if (!raw.trim()) {
      opts.refSection.hidden = true;
      const panel = opts.refSection.querySelector(".source-submenu-panel") as HTMLElement | null;
      if (panel) panel.hidden = true;
      opts.refSection.querySelector(".source-submenu-toggle")?.setAttribute("aria-expanded", "false");
      opts.row.hidden = true;
      if (state === "idle") lastValue = REF_NONE;
      renderButtons();
      return;
    }
    const d = parseTclDictNested(raw);
    nextName = d.next ?? "";
    setState((d.state as RunState) || "idle");
    opts.refSection.hidden = false;
    opts.row.hidden = false;
    populate(parseTclList(d.files ?? ""), d.current ?? "");
    opts.onMenuUpdate?.();
  }

  async function poll() {
    try {
      const d = parseTclDictNested(await opts.conn.sendEvalAsync("run_status"));
      frame = Number(d.frame) || 0;
      total = Number(d.total) || 0;
      const next = (d.state as RunState) || "idle";
      if (next !== state && (next === "done" || next === "idle")) {
        setState(next);
        await refresh();
        return;
      }
      setState(next);
      renderButtons();
      renderMenu();
      opts.onChange?.();
    } catch {
      // transient (busy main loop); try again next tick
    }
  }

  async function run(script: string): Promise<string | null> {
    if (busy) return null;
    busy = true;
    opts.showError("");
    try {
      return await opts.conn.sendEvalAsync(script);
    } catch (e) {
      opts.showError(e instanceof Error ? e.message : String(e));
      return null;
    } finally {
      busy = false;
    }
  }

  opts.reprocessBtn.addEventListener("click", async () => {
    if (state === "done" && !window.confirm("Replace the unsaved re-process run?")) return;
    const res = await run("run_start");
    if (res === null) return;
    frame = 0;
    setState(res === "closing" ? "closing" : "running");
    await poll();
  });

  opts.saveBtn.addEventListener("click", async () => {
    const res = await run("run_save");
    if (res === null) return;
    lastValue = res;
    setState("idle");
    await refresh();
  });

  function badge(): Badge | null {
    if (state === "running") return { text: `re-processing ${pct()}%`, kind: "info" };
    if (state === "closing") return { text: "re-process finishing", kind: "info" };
    if (state === "done") {
      return {
        text: "unsaved run",
        kind: "warn",
        title: "Open the source menu and Save to keep this tracking pass.",
      };
    }
    return null;
  }

  return {
    refresh,
    runState: () => state,
    badge,
    renderMenu,
    selection: () => lastValue,
    choiceLabel,
  };
}
