import type { Connection } from "./connection";
import type { BrowseEntry, BrowsePlace } from "./protocol";
import { settings } from "./settings";

const VIDEO_FILTER = "Video files (.mp4, .avi, .mkv, .mov, .m4v, .webm, .mpg)";

const FOLDER_ICON =
  '<svg viewBox="0 0 16 16" aria-hidden="true"><path d="M1.5 3.5h4l1.5 1.5h7.5v8h-13z" fill="#d9a441" stroke="#b5832a"/></svg>';
const VIDEO_ICON =
  '<svg viewBox="0 0 16 16" aria-hidden="true"><rect x="2" y="2.5" width="12" height="11" rx="1.5" fill="#3d5a80" stroke="#6b8fbf"/><path d="M6.5 5.5v5l4-2.5z" fill="#dfe8f5"/></svg>';

export interface FilePicker {
  /** Resolves with the chosen file path, or null if cancelled. */
  pick: () => Promise<string | null>;
  isOpen: () => boolean;
}

type SortKey = "name" | "size" | "mtime";
const SORT_SETTING = "ui.pickerSort"; // kept by the server, the same in every browser

// Folders stay above files; the chosen column orders each group.
function loadSort(): { key: SortKey; asc: boolean } {
  const v = settings.json<{ key?: string; asc?: unknown } | null>(SORT_SETTING, null);
  if ((v?.key === "name" || v?.key === "size" || v?.key === "mtime") && typeof v.asc === "boolean") {
    return { key: v.key, asc: v.asc };
  }
  return { key: "name", asc: true };
}

const byName = (a: BrowseEntry, b: BrowseEntry) =>
  a.name.localeCompare(b.name, undefined, { numeric: true, sensitivity: "base" });

function sortEntries(list: BrowseEntry[], key: SortKey, asc: boolean): BrowseEntry[] {
  const dir = asc ? 1 : -1;
  const value = (e: BrowseEntry) => (key === "size" ? (e.size ?? 0) : (e.mtime ?? 0));
  return [...list].sort((a, b) => {
    if (a.dir !== b.dir) return a.dir ? -1 : 1;
    // Folders have no size, so size order falls back to name for them.
    if (key === "name" || (key === "size" && a.dir)) return dir * byName(a, b);
    return dir * (value(a) - value(b)) || byName(a, b);
  });
}

export function attachFilePicker(conn: Connection): FilePicker {
  const backdrop = el("div", "modal-backdrop picker-backdrop");
  backdrop.hidden = true;
  const dlg = el("div", "modal picker");
  dlg.setAttribute("role", "dialog");
  dlg.setAttribute("aria-label", "Open video");
  dlg.hidden = true;
  dlg.innerHTML = `
    <header class="modal-header">
      <h2>Open video</h2>
      <button type="button" class="modal-close" data-act="cancel" aria-label="Close">×</button>
    </header>
    <div class="picker-toolbar">
      <button type="button" class="picker-nav" data-act="back" title="Back (Alt+Left)" aria-label="Back">&#8592;</button>
      <button type="button" class="picker-nav" data-act="up" title="Up one level (Backspace)" aria-label="Up">&#8593;</button>
      <nav class="picker-crumbs" aria-label="Current folder"></nav>
    </div>
    <div class="picker-body">
      <aside class="picker-places" aria-label="Places"></aside>
      <div class="picker-main">
        <div class="picker-head" role="row">
          <button type="button" class="picker-sort" data-sort="name">Name<span class="picker-sort-arrow" aria-hidden="true"></span></button>
          <button type="button" class="picker-sort picker-sort-size" data-sort="size">Size<span class="picker-sort-arrow" aria-hidden="true"></span></button>
          <button type="button" class="picker-sort" data-sort="mtime">Modified<span class="picker-sort-arrow" aria-hidden="true"></span></button>
        </div>
        <div class="picker-list" role="listbox" tabindex="0"></div>
      </div>
    </div>
    <p class="modal-error picker-error" hidden></p>
    <footer class="picker-footer">
      <label class="picker-name">
        <span>File name</span>
        <input type="text" readonly />
      </label>
      <span class="picker-filter">${VIDEO_FILTER}</span>
      <div class="picker-actions">
        <button type="button" class="picker-btn" data-act="cancel">Cancel</button>
        <button type="button" class="picker-btn picker-btn-primary" data-act="open" disabled>Open</button>
      </div>
    </footer>`;
  document.body.append(backdrop, dlg);

  const q = <T extends HTMLElement>(sel: string) => dlg.querySelector(sel) as T;
  const crumbs = q<HTMLElement>(".picker-crumbs");
  const placesEl = q<HTMLElement>(".picker-places");
  const list = q<HTMLElement>(".picker-list");
  const errorEl = q<HTMLElement>(".picker-error");
  const nameInput = q<HTMLInputElement>(".picker-name input");
  const openBtn = q<HTMLButtonElement>('[data-act="open"]');
  const backBtn = q<HTMLButtonElement>('[data-act="back"]');
  const upBtn = q<HTMLButtonElement>('[data-act="up"]');

  let cwd = "";
  let parent: string | null = null;
  let entries: BrowseEntry[] = [];
  let selected = -1;
  let history: string[] = [];
  let resolver: ((v: string | null) => void) | null = null;
  let loadSeq = 0;
  let sort = loadSort();
  let typeAhead = "";
  let typeAheadTimer: number | undefined;

  const isOpen = () => !dlg.hidden;

  function finish(result: string | null) {
    dlg.hidden = true;
    backdrop.hidden = true;
    const r = resolver;
    resolver = null;
    r?.(result);
  }

  function showError(msg: string) {
    errorEl.textContent = msg;
    errorEl.hidden = !msg;
  }

  async function navigate(path?: string, pushHistory = true) {
    const seq = ++loadSeq;
    showError("");
    list.replaceChildren(muted("Loading…"));
    try {
      const res = path !== undefined ? await conn.browsePath(path) : await conn.browsePath();
      if (seq !== loadSeq) return;
      if (res.status === "error") throw new Error(res.error ?? "Could not open folder");
      if (pushHistory && cwd && res.path !== cwd) history.push(cwd);
      cwd = res.path ?? "";
      parent = res.parent ?? null;
      entries = sortEntries(res.entries ?? [], sort.key, sort.asc);
      selected = -1;
      renderCrumbs();
      renderPlaces(res.places ?? []);
      renderList();
      renderSortHeader();
      updateFooter();
      list.focus();
    } catch (e) {
      if (seq !== loadSeq) return;
      list.replaceChildren(muted("Could not read this folder."));
      showError(e instanceof Error ? e.message : String(e));
    }
    backBtn.disabled = history.length === 0;
    upBtn.disabled = !parent;
  }

  function renderCrumbs() {
    crumbs.replaceChildren();
    const parts = cwd.split("/").filter(Boolean);
    const segs: { label: string; path: string }[] = [{ label: "/", path: "/" }];
    let acc = "";
    for (const p of parts) {
      acc += "/" + p;
      segs.push({ label: p, path: acc });
    }
    segs.forEach((s, i) => {
      if (i > 1) crumbs.append(el("span", "picker-crumb-sep", "›"));
      const b = el("button", "picker-crumb", s.label) as HTMLButtonElement;
      b.type = "button";
      if (i === segs.length - 1) b.classList.add("current");
      b.addEventListener("click", () => void navigate(s.path));
      crumbs.append(b);
    });
    crumbs.scrollLeft = crumbs.scrollWidth;
  }

  function renderPlaces(places: BrowsePlace[]) {
    placesEl.replaceChildren(el("div", "picker-places-title", "Places"));
    for (const p of places) {
      const b = el("button", "picker-place", p.label) as HTMLButtonElement;
      b.type = "button";
      b.title = p.path;
      if (p.path === cwd) b.classList.add("current");
      b.addEventListener("click", () => void navigate(p.path));
      placesEl.append(b);
    }
  }

  function renderList() {
    list.replaceChildren();
    if (!entries.length) {
      list.append(muted("This folder has no subfolders or video files."));
      return;
    }
    entries.forEach((ent, i) => {
      const row = el("div", "picker-row" + (ent.dir ? " is-dir" : ""));
      row.setAttribute("role", "option");
      row.dataset.idx = String(i);
      const name = el("span", "picker-cell-name");
      name.innerHTML = ent.dir ? FOLDER_ICON : VIDEO_ICON;
      name.append(el("span", "picker-label", ent.name));
      row.append(
        name,
        el("span", "picker-cell-size", ent.dir ? "" : formatSize(ent.size)),
        el("span", "picker-cell-date", formatDate(ent.mtime)),
      );
      row.title = ent.name;
      row.addEventListener("click", () => select(i));
      row.addEventListener("dblclick", () => activate(i));
      list.append(row);
    });
  }

  function renderSortHeader() {
    for (const b of dlg.querySelectorAll<HTMLElement>("[data-sort]")) {
      const on = b.dataset.sort === sort.key;
      b.classList.toggle("active", on);
      b.setAttribute("aria-sort", on ? (sort.asc ? "ascending" : "descending") : "none");
      const arrow = b.querySelector(".picker-sort-arrow");
      if (arrow) arrow.textContent = on ? (sort.asc ? "\u25B2" : "\u25BC") : "";
    }
  }

  // Click the active column to flip direction; a new column starts ascending.
  function setSort(key: SortKey) {
    sort = key === sort.key ? { key, asc: !sort.asc } : { key, asc: true };
    settings.set(SORT_SETTING, JSON.stringify(sort));
    const keep = entries[selected]?.path;
    entries = sortEntries(entries, sort.key, sort.asc);
    selected = keep === undefined ? -1 : entries.findIndex((e) => e.path === keep);
    renderList();
    renderSortHeader();
    if (selected >= 0) select(selected);
    else updateFooter();
  }

  // Another browser changed the sort order (or this one just loaded the server's).
  settings.on(SORT_SETTING, () => {
    sort = loadSort();
    const keep = entries[selected]?.path;
    entries = sortEntries(entries, sort.key, sort.asc);
    selected = keep === undefined ? -1 : entries.findIndex((e) => e.path === keep);
    if (!dlg.hidden) {
      renderList();
      if (selected >= 0) select(selected);
    }
    renderSortHeader();
  });

  function select(i: number) {
    if (i < 0 || i >= entries.length) return;
    selected = i;
    for (const r of list.querySelectorAll<HTMLElement>(".picker-row")) {
      const on = Number(r.dataset.idx) === i;
      r.classList.toggle("selected", on);
      r.setAttribute("aria-selected", String(on));
      if (on) r.scrollIntoView({ block: "nearest" });
    }
    updateFooter();
  }

  function activate(i: number) {
    const ent = entries[i];
    if (!ent) return;
    if (ent.dir) void navigate(ent.path);
    else finish(ent.path);
  }

  function updateFooter() {
    const ent = entries[selected];
    nameInput.value = ent && !ent.dir ? ent.name : "";
    openBtn.disabled = !ent;
    openBtn.textContent = ent?.dir ? "Open folder" : "Open";
  }

  function goUp() {
    if (parent) void navigate(parent);
  }

  function goBack() {
    const prev = history.pop();
    if (prev !== undefined) void navigate(prev, false);
  }

  dlg.addEventListener("click", (ev) => {
    const sortKey = (ev.target as HTMLElement).closest<HTMLElement>("[data-sort]")?.dataset.sort;
    if (sortKey) {
      setSort(sortKey as SortKey);
      return;
    }
    const act = (ev.target as HTMLElement).closest<HTMLElement>("[data-act]")?.dataset.act;
    if (act === "cancel") finish(null);
    else if (act === "open") activate(selected);
    else if (act === "up") goUp();
    else if (act === "back") goBack();
  });
  backdrop.addEventListener("click", () => finish(null));

  dlg.addEventListener("keydown", (ev) => {
    if (ev.key === "Escape") {
      ev.preventDefault();
      ev.stopPropagation();
      finish(null);
      return;
    }
    if ((ev.target as HTMLElement).tagName === "BUTTON" && ev.key === "Enter") return;
    switch (ev.key) {
      case "ArrowDown":
        ev.preventDefault();
        select(Math.min(entries.length - 1, selected + 1));
        return;
      case "ArrowUp":
        ev.preventDefault();
        if (ev.altKey) goUp();
        else select(Math.max(0, selected - 1));
        return;
      case "ArrowLeft":
        if (ev.altKey) {
          ev.preventDefault();
          goBack();
        }
        return;
      case "Home":
        ev.preventDefault();
        select(0);
        return;
      case "End":
        ev.preventDefault();
        select(entries.length - 1);
        return;
      case "Enter":
        ev.preventDefault();
        activate(selected);
        return;
      case "Backspace":
        ev.preventDefault();
        goUp();
        return;
    }
    if (ev.key.length === 1 && !ev.ctrlKey && !ev.metaKey && !ev.altKey) {
      window.clearTimeout(typeAheadTimer);
      typeAhead += ev.key.toLowerCase();
      typeAheadTimer = window.setTimeout(() => (typeAhead = ""), 700);
      const hit = entries.findIndex((e) => e.name.toLowerCase().startsWith(typeAhead));
      if (hit >= 0) select(hit);
    }
  });

  function pick(): Promise<string | null> {
    if (resolver) finish(null);
    dlg.hidden = false;
    backdrop.hidden = false;
    history = [];
    void navigate(cwd || undefined, false);
    return new Promise((resolve) => {
      resolver = resolve;
    });
  }

  return { pick, isOpen };
}

function el(tag: string, cls: string, text?: string): HTMLElement {
  const e = document.createElement(tag);
  e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
}

function muted(text: string): HTMLElement {
  return el("p", "picker-empty", text);
}

function formatSize(bytes?: number): string {
  if (bytes === undefined) return "";
  const units = ["B", "KB", "MB", "GB", "TB"];
  let v = bytes;
  let u = 0;
  while (v >= 1024 && u < units.length - 1) {
    v /= 1024;
    u++;
  }
  return `${v < 10 && u > 0 ? v.toFixed(1) : Math.round(v)} ${units[u]}`;
}

function formatDate(unix?: number): string {
  if (!unix) return "";
  return new Date(unix * 1000).toLocaleString(undefined, {
    year: "numeric",
    month: "short",
    day: "numeric",
    hour: "numeric",
    minute: "2-digit",
  });
}
