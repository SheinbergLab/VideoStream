import type { Connection, LogLine } from "./connection";

// Lines kept on the page; the server keeps its own, longer-running ring.
const MAX_LINES = 2000;
// Stay pinned to the newest line while the view is within this many pixels of the bottom.
const STICK_PX = 24;

export interface ConsolePanel {
  append: (lines: LogLine[]) => void;
  /** Forget everything shown (the server restarted). */
  reset: () => void;
}

/**
 * The server's terminal output, shown in a panel above the status bar and
 * opened with a button in it. The stream only runs while the panel is open;
 * reopening picks up the lines that came in meanwhile.
 */
export function attachConsolePanel(conn: Connection, toggle: HTMLButtonElement, bar: HTMLElement): ConsolePanel {
  const panel = document.createElement("section");
  panel.className = "console-panel";
  panel.setAttribute("aria-label", "Server console");
  panel.hidden = true;
  panel.innerHTML = `
    <header class="console-head">
      <span class="console-title">Server console</span>
      <button type="button" class="console-btn" data-act="clear" title="Clear what is shown here">Clear</button>
      <button type="button" class="console-btn" data-act="close" aria-label="Close" title="Close">×</button>
    </header>
    <div class="console-body" role="log" tabindex="0"></div>`;
  document.body.append(panel);
  const body = panel.querySelector(".console-body") as HTMLElement;

  const place = () => {
    panel.style.bottom = `${bar.offsetHeight + 8}px`;
  };

  function open() {
    place();
    panel.hidden = false;
    toggle.setAttribute("aria-expanded", "true");
    toggle.classList.add("active");
    conn.setLogs(true);
    body.scrollTop = body.scrollHeight;
  }

  function close() {
    panel.hidden = true;
    toggle.setAttribute("aria-expanded", "false");
    toggle.classList.remove("active");
    conn.setLogs(false);
  }

  toggle.addEventListener("click", () => (panel.hidden ? open() : close()));
  window.addEventListener("resize", () => {
    if (!panel.hidden) place();
  });
  panel.addEventListener("click", (ev) => {
    const act = (ev.target as HTMLElement).closest<HTMLElement>("[data-act]")?.dataset.act;
    if (act === "close") close();
    else if (act === "clear") body.replaceChildren();
  });
  panel.addEventListener("keydown", (ev) => {
    if (ev.key === "Escape") close();
  });

  return {
    append(lines) {
      if (!lines.length) return;
      const stick = body.scrollHeight - body.scrollTop - body.clientHeight < STICK_PX;
      const frag = document.createDocumentFragment();
      for (const l of lines) {
        const row = document.createElement("div");
        row.className = l.err ? "console-line err" : "console-line";
        row.textContent = l.text;
        frag.append(row);
      }
      body.append(frag);
      while (body.childElementCount > MAX_LINES) body.firstElementChild?.remove();
      if (stick) body.scrollTop = body.scrollHeight;
    },
    reset() {
      body.replaceChildren();
    },
  };
}
