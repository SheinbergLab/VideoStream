/** Double-quoted Tcl word (safe inside `[list "…"]`). */
export function tclDoubleQuoted(value: string): string {
  const s = value.replace(/\r/g, "");
  return (
    '"' +
    s
      .replace(/\\/g, "\\\\")
      .replace(/"/g, '\\"')
      .replace(/\$/g, "\\$")
      .replace(/\[/g, "\\[")
      .replace(/\]/g, "\\]") +
    '"'
  );
}

/** Tcl list → elements; handles {braced} and "quoted" elements and backslashes. */
export function parseTclList(raw: string): string[] {
  const out: string[] = [];
  const s = raw;
  let i = 0;
  const isSpace = (ch: string) => ch === " " || ch === "\t" || ch === "\n" || ch === "\r";
  while (i < s.length) {
    while (i < s.length && isSpace(s[i])) i++;
    if (i >= s.length) break;
    if (s[i] === "{") {
      let depth = 1;
      let j = i + 1;
      while (j < s.length && depth > 0) {
        if (s[j] === "\\") j++;
        else if (s[j] === "{") depth++;
        else if (s[j] === "}") depth--;
        j++;
      }
      out.push(s.slice(i + 1, j - 1));
      i = j;
    } else {
      const quoted = s[i] === '"';
      let j = quoted ? i + 1 : i;
      let word = "";
      while (j < s.length && (quoted ? s[j] !== '"' : !isSpace(s[j]))) {
        if (s[j] === "\\" && j + 1 < s.length) j++;
        word += s[j];
        j++;
      }
      out.push(word);
      i = quoted ? j + 1 : j;
    }
  }
  return out;
}

/** Tcl dict whose values may be lists or nested dicts (left as raw strings). */
export function parseTclDictNested(raw: string): Record<string, string> {
  const parts = parseTclList(raw);
  const out: Record<string, string> = {};
  for (let i = 0; i + 1 < parts.length; i += 2) out[parts[i]] = parts[i + 1];
  return out;
}
