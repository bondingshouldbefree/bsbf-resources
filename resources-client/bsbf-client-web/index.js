/**
 * ============================================================================
 * BSBF client dashboard — overview
 * ============================================================================
 *
 * This script drives the single-page dashboard that shows one "card" per WAN
 * interface (e.g. eth1, wwan0, ...). Each card displays:
 *   - a live RX/TX speed graph (canvas),
 *   - the current download/upload rate caps (editable, applied via CGI),
 *   - MPTCP / connectivity status badges,
 *   - a "Backup" checkbox for MPTCP subflow priority.
 *
 * Data flow, once per second (POLL_INTERVAL):
 *   1. `runClientPollCycle` fetches a single plain-text snapshot from
 *      `/cgi-bin/bsbf-client-web?poll` (see "POLL SNAPSHOT FETCHING & PARSING"
 *      below for the text format).
 *   2. Every registered "poll hook" (see `registerClientPollHook`) gets the
 *      raw text and updates its part of the UI from it. The main hook,
 *      `coreClientPollHook`, updates the interface list, graphs, rate labels
 *      and tc (traffic-control) derived cap values. The bonding toggle
 *      registers its own hook for the bonding enabled/disabled status.
 *
 * The user can also:
 *   - Drag-reorder cards (top card = "primary" link, gets unlimited caps;
 *     the rest get tiered caps based on the primary's observed peak speed).
 *   - Edit Download/Upload Mbps caps per card and click "Apply", which pushes
 *     the new limits to the router via `?limit&<iface>&<dl>&<ul>`.
 *   - Toggle MPTCP bonding on/off, and force an interface into MPTCP
 *     "backup" (lower priority) role.
 *
 * State for interface `X` is spread across a few parallel maps, all keyed by
 * interface name:
 *   - `ifaceData[X]`   — live RX/TX history + peaks, for the graph.
 *   - `ifaceFlags[X]`  — UI flags: connectivity/MPTCP badges, backup config.
 *   - `tcInfo[X]`      — last known caps as reported by `tc class show`
 *                        (reference only — does NOT track unsaved edits).
 *   - `sortCap[X]`     — the caps used for sorting + what gets sent to the
 *                        router; this is what the input fields reflect.
 *   - `origIfaceIndex[X]` — tie-breaker reflecting original discovery order.
 *
 * `ifaceOrder` is the single source of truth for "which interfaces exist and
 * in what order they're displayed" — it drives both the DOM order and which
 * interfaces get polled for stats / caps.
 */

// ----------------------------------------------------------------------------
// Tunable constants
// ----------------------------------------------------------------------------

/** How many samples each speed graph keeps (one per poll, ~1 per second). */
const HISTORY_LEN = 60;
/** How often we poll `/cgi-bin/bsbf-client-web?poll` for fresh data, in ms. */
const POLL_INTERVAL = 1000;
/** Duration of the card-reorder slide animation, in ms. */
const REORDER_ANIM_MS = 340;
/** Easing curve for the card-reorder slide animation. */
const REORDER_ANIM_EASING = "cubic-bezier(0.22, 1, 0.36, 1)";
/**
 * After the user applies a cap change or toggles bonding/backup, the next
 * poll(s) within this window only update the speed graphs — they skip
 * re-fetching/re-applying caps and re-syncing the bonding UI. This avoids the
 * UI snapping the just-edited inputs back to stale values before the router
 * has actually applied the change.
 */
const CHART_ONLY_MS = 2000;

// ----------------------------------------------------------------------------
// Global state
// ----------------------------------------------------------------------------

/** Timestamp (ms) until which poll cycles should only update the speed charts. */
let chartOnlyUntil = 0;

/** Per-interface live stats (RX/TX history + peaks) — see `initIfaceData`. */
const ifaceData = {};
/** Per-interface UI flags: connectivity/MPTCP badges + backup config — see `applyMptcpBadgesForIfaceList`. */
const ifaceFlags = {};

/** Order of interfaces (top to bottom); updated when user reorders cards or caps change. */
let ifaceOrder = [];
/** Name of the interface currently being drag-reordered, or null. */
let draggedIface = null;

/** Original discovery order — tie-break when caps are equal. */
const origIfaceIndex = {};

/** Parsed from `tc class show` (reference only; does not change when user edits cap). */
const tcInfo = {};

/**
 * Editable per-path limits (Mbps) + per-direction unlimited flags.
 * Order: both unlimited first, then by download Mbps, then upload Mbps.
 */
const sortCap = {};

/** Monotonic tie-break for newly discovered interfaces (after initial load). */
let nextOrigIfaceIndex = 0;

/** Only one panel drag at a time (avoids multi-touch / multiple simultaneous drags). */
let panelDragActive = false;

// ============================================================================
// Formatting helpers
// ============================================================================

/** Format a byte/s rate for display on the graph axis and rate labels, e.g. "12.34 Mbps". */
function formatRate(bytesPerSec) {
  if (bytesPerSec < 0) return "---";
  const bitsPerSec = bytesPerSec * 8;
  if (bitsPerSec >= 1e9) return (bitsPerSec / 1e9).toFixed(2) + " Gbps";
  if (bitsPerSec >= 1e6) return (bitsPerSec / 1e6).toFixed(2) + " Mbps";
  if (bitsPerSec >= 1e3) return (bitsPerSec / 1e3).toFixed(2) + " Kbps";
  return bitsPerSec.toFixed(0) + " bps";
}

/**
 * Format a Mbps value for the cap input fields: up to 3 decimal places,
 * trailing zeros trimmed (e.g. 12 -> "12", 12.5 -> "12.5", 12.000001 -> "12").
 */
function formatMbpsShort(mbps) {
  if (!isFinite(mbps)) return "0";
  const r = Math.round(mbps * 1000) / 1000;
  if (Math.abs(r - Math.round(r)) < 1e-9) return String(Math.round(r));
  const s = r.toFixed(3).replace(/\.?0+$/, "");
  return s;
}

/** Convert a peak byte/s rate (as stored in `ifaceData[x].peakTx/peakRx`) to Mbps. */
function bytesPerSecPeakToMbps(bps) {
  return (bps * 8) / 1e6;
}

// ============================================================================
// Cap (download/upload limit) state <-> card UI sync
// ============================================================================

/** Push sortCap to Download / Upload inputs for one card. */
function syncCapUIFromSortCap(iface) {
  const card = document.getElementById("card-" + iface);
  if (!card) return;
  const c = sortCap[iface];
  if (!c) return;
  const inpDl = card.querySelector(".cap-dl-input");
  const inpUl = card.querySelector(".cap-ul-input");
  const cbDl = card.querySelector(".cap-dl-unl");
  const cbUl = card.querySelector(".cap-ul-unl");
  if (!inpDl || !inpUl || !cbDl || !cbUl) return;
  cbDl.checked = c.dlUnl;
  cbUl.checked = c.ulUnl;
  inpDl.disabled = c.dlUnl;
  inpUl.disabled = c.ulUnl;
  inpDl.value = c.dlUnl ? "" : formatMbpsShort(c.dlMbps);
  inpUl.value = c.ulUnl ? "" : formatMbpsShort(c.ulMbps);
  updateCapApplyButtonState(iface);
}

/**
 * Read the Download/Upload cap currently shown in a card's inputs, as a
 * `{ dlUnl, ulUnl, dlMbps, ulMbps }` object (same shape as `sortCap[iface]`).
 * Returns `null` if the card/inputs are missing, or if a non-"Unlimited"
 * field doesn't contain a valid non-negative number (i.e. the input is
 * invalid and shouldn't be applied).
 */
function readCapFromInputsForIface(iface) {
  const card = document.getElementById("card-" + iface);
  if (!card) return null;
  const inpDl = card.querySelector(".cap-dl-input");
  const inpUl = card.querySelector(".cap-ul-input");
  const cbDl = card.querySelector(".cap-dl-unl");
  const cbUl = card.querySelector(".cap-ul-unl");
  if (!inpDl || !inpUl || !cbDl || !cbUl) return null;
  const c = sortCap[iface];
  if (!c) return null;
  const dlUnl = cbDl.checked;
  const ulUnl = cbUl.checked;
  let dlMbps = c.dlMbps;
  let ulMbps = c.ulMbps;
  if (!dlUnl) {
    const v = parseFloat(String(inpDl.value).trim());
    if (!isFinite(v) || v < 0) return null;
    dlMbps = v;
  }
  if (!ulUnl) {
    const v = parseFloat(String(inpUl.value).trim());
    if (!isFinite(v) || v < 0) return null;
    ulMbps = v;
  }
  return { dlUnl: dlUnl, ulUnl: ulUnl, dlMbps: dlMbps, ulMbps: ulMbps };
}

/** True if two cap objects (`{ dlUnl, ulUnl, dlMbps, ulMbps }`) represent the same limits. */
function capsEqual(a, b) {
  if (!a || !b) return false;
  if (a.dlUnl !== b.dlUnl || a.ulUnl !== b.ulUnl) return false;
  const eps = 1e-6;
  if (!a.dlUnl && Math.abs(a.dlMbps - b.dlMbps) > eps) return false;
  if (!a.ulUnl && Math.abs(a.ulMbps - b.ulMbps) > eps) return false;
  return true;
}

/**
 * True if a card's inputs (cap fields and/or the "Backup" checkbox) differ
 * from the last-known-applied state (`sortCap` / `ifaceFlags`). Used to:
 *  - enable/disable the per-card "Apply" button, and
 *  - decide whether a poll cycle is allowed to overwrite this card's inputs
 *    with fresh server data (it must not clobber unsaved user edits).
 */
function ifaceHasPendingChanges(iface) {
  const c = sortCap[iface];
  if (!c) return false;
  const read = readCapFromInputsForIface(iface);
  const capDirty = read === null ? true : !capsEqual(read, c);
  const card = document.getElementById("card-" + iface);
  const cbFb = card && card.querySelector(".cap-force-backup");
  const wantFb = !!(cbFb && cbFb.checked);
  const savedFb = !!(ifaceFlags[iface] && ifaceFlags[iface].forceBackupConfig);
  return capDirty || wantFb !== savedFb;
}

/** Enable the "Apply" button for a card only if it has unsaved cap/backup edits. */
function updateCapApplyButtonState(iface) {
  const card = document.getElementById("card-" + iface);
  if (!card) return;
  const btn = card.querySelector(".btn-cap-apply");
  if (!btn) return;
  btn.disabled = !ifaceHasPendingChanges(iface);
}

/**
 * Show/clear the spinner + disabled state on a card's "Apply" button while a
 * `limit&` CGI call for that interface is in flight. Used both by the
 * button's own click handler and by `reorderInterfaces`, whose drag-triggered
 * `applyRateLimits()` call covers whichever cards' caps actually changed.
 */
function setCapApplyButtonLoading(iface, loading) {
  const card = document.getElementById("card-" + iface);
  if (!card) return;
  const btn = card.querySelector(".btn-cap-apply");
  if (!btn) return;
  if (loading) {
    btn.disabled = true;
    btn.classList.add("is-loading");
    btn.setAttribute("aria-busy", "true");
  } else {
    btn.classList.remove("is-loading");
    btn.removeAttribute("aria-busy");
    updateCapApplyButtonState(iface);
  }
}

// ============================================================================
// Tiered caps after drag-reorder
// ============================================================================

/** All non-top interfaces use this fraction of the top (index 0) session peaks — always interface 1, never each other. */
const TIER_NON_TOP_FRAC_OF_TOP = 0.8;

/**
 * After drag reorder: slot 0 = unlimited. Every other slot uses 80% of the **top**
 * interface’s session peak RX/TX (Mbps) only (same reference for rows 2, 3, …).
 */
function applyTieredCapsFromTopPeaks() {
  if (ifaceOrder.length === 0) return;
  const topName = ifaceOrder[0];
  const topData = ifaceData[topName];
  const topRxMbps = topData ? bytesPerSecPeakToMbps(topData.peakRx) : 0;
  const topTxMbps = topData ? bytesPerSecPeakToMbps(topData.peakTx) : 0;

  for (let i = 0; i < ifaceOrder.length; i++) {
    const name = ifaceOrder[i];
    if (i === 0) {
      sortCap[name] = {
        dlUnl: true,
        ulUnl: true,
        dlMbps: 0,
        ulMbps: 0,
      };
    } else {
      let dlRounded = Math.round(
        topRxMbps * TIER_NON_TOP_FRAC_OF_TOP
      );
      let ulRounded = Math.round(
        topTxMbps * TIER_NON_TOP_FRAC_OF_TOP
      );
      if (dlRounded === 0) dlRounded = 1;
      if (ulRounded === 0) ulRounded = 1;
      sortCap[name] = {
        dlUnl: false,
        ulUnl: false,
        dlMbps: dlRounded,
        ulMbps: ulRounded,
      };
    }
  }
}

/** Re-sync every card's cap inputs from `sortCap` (e.g. after a reorder retiers caps). */
function syncAllCapUIFromSortCap() {
  for (let i = 0; i < ifaceOrder.length; i++) {
    syncCapUIFromSortCap(ifaceOrder[i]);
  }
}

// ============================================================================
// `tc class show` output parsing
// ============================================================================
//
// The router reports current rate limits by running `tc class show` for each
// interface's download (IFB) and upload (egress) classes. We parse that
// human-readable `tc` output ourselves (this used to be done by a CGI awk
// script) to recover the configured Mbps caps for each direction.

/**
 * Find the highest `rate`/`ceil` value (in Mbps) across all `tc class show`
 * lines in `block`. Returns `NaN` if no `rate`/`ceil ...bit` token was found
 * at all (i.e. this class has no configured limit).
 *
 * `tc` prints values like "100Mbit", "1.5Gbit", "500Kbit", or a bare
 * "<number>bit" with no unit (bits/sec) — `parseTok` below converts all of
 * these to Mbps.
 */
function parseTcClassMaxMbps(block) {
  const lines = String(block || "").split(/\r?\n/);
  let max = 0;
  let found = false;

  // Convert one whitespace-separated token, e.g. "100Mbit" or "500Kbit", to Mbps.
  // Returns NaN for tokens that aren't a `<number><unit>bit` rate (e.g. "burst").
  function parseTok(tok) {
    if (!/bit$/.test(tok)) return NaN;
    let s = tok.replace(/bit$/, "");
    // Walk back from the end over the unit letters (e.g. "G", "M", "k") to
    // find where the numeric part ends.
    let i = s.length;
    let u = "";
    while (i > 0) {
      const c = s.charAt(i - 1);
      if (/[0-9.]/.test(c)) break;
      u = c + u;
      i--;
    }
    const v = parseFloat(s.slice(0, i));
    if (!isFinite(v)) return NaN;
    if (u === "G" || u === "g") return v * 1000;
    if (u === "M" || u === "m") return v;
    if (u === "k" || u === "K") return v / 1000;
    if (u === "") return v / 1000000; // bare bits/sec -> Mbps
    return NaN;
  }

  for (let li = 0; li < lines.length; li++) {
    const parts = lines[li].trim().split(/\s+/);
    for (let i = 0; i < parts.length; i++) {
      if (parts[i] === "rate" || parts[i] === "ceil") {
        if (i + 1 < parts.length) {
          const m = parseTok(parts[i + 1]);
          if (isFinite(m) && m >= 0) {
            found = true;
            if (m > max) max = m;
          }
        }
      }
    }
  }
  return found ? max : NaN;
}

/**
 * Turn the raw `tc class show` text for one interface's download (`dlText`)
 * and upload (`ulText`) classes into a normalized cap row (see `normalizeTcJson`).
 * Empty text on both sides, or no parseable rate on either side, means
 * "unlimited" for that direction (or both).
 */
function tcBlocksToRow(dlText, ulText) {
  const blank =
    !String(dlText || "").replace(/\s/g, "").length &&
    !String(ulText || "").replace(/\s/g, "").length;
  if (blank) return normalizeTcJson({ unlimited: true });
  const dlVal = parseTcClassMaxMbps(dlText);
  const ulVal = parseTcClassMaxMbps(ulText);
  const dlNA = !isFinite(dlVal);
  const ulNA = !isFinite(ulVal);
  if (dlNA && ulNA) return normalizeTcJson({ unlimited: true });
  return normalizeTcJson({
    dl_unlimited: dlNA,
    ul_unlimited: ulNA,
    dl_mbps: dlNA ? 0 : dlVal,
    ul_mbps: ulNA ? 0 : ulVal,
  });
}

/**
 * Split one interface's combined `tc class show` chunk (formatted as
 * `__dl__<download tc output>__ul__<upload tc output>`) into its two halves
 * and parse them into a normalized cap row.
 */
function parseTcSingleIfaceChunk(chunk) {
  const dlTag = "__dl__";
  const ulTag = "__ul__";
  const c = String(chunk || "");
  const i1 = c.indexOf(dlTag);
  const i2 = c.indexOf(ulTag);
  if (i1 < 0 || i2 < 0)
    return normalizeTcJson({ unlimited: true });
  const dlText = c.slice(i1 + dlTag.length, i2);
  const ulText = c.slice(i2 + ulTag.length);
  return tcBlocksToRow(dlText, ulText);
}

/**
 * Legacy/fallback format: parse a batch of `tc class show` outputs that are
 * delimited by `---iface <name>---` header lines, each followed by that
 * interface's combined `__dl__...__ul__...` chunk. Returns `{ [iface]: capRow }`.
 */
function parseTcClassBatchText(text) {
  const out = {};
  const t = String(text || "");
  const re = /^---iface ([a-zA-Z0-9._-]+)---\s*/gm;
  let m;
  const hits = [];
  while ((m = re.exec(t)) !== null) {
    hits.push({
      name: m[1],
      blockStart: m.index,
      afterHeader: m.index + m[0].length,
    });
  }
  for (let i = 0; i < hits.length; i++) {
    const end = i + 1 < hits.length ? hits[i + 1].blockStart : t.length;
    const chunk = t.slice(hits[i].afterHeader, end);
    out[hits[i].name] = parseTcSingleIfaceChunk(chunk);
  }
  return out;
}

// ============================================================================
// Poll snapshot parsing — interface lists, routes, badges
// ============================================================================

/**
 * Legacy fallback for discovering the interface list: scan `ip route` output
 * for `default ... dev <iface> ... metric <n>` lines (each interface's
 * default route), in the order they appear, de-duplicated. Lines without
 * both `dev` and `metric` are ignored.  Same rules as the former CGI awk script.
 */
function parseDefaultRouteIfaces(ipRouteText) {
  const lines = String(ipRouteText || "").split(/\r?\n/);
  const seen = Object.create(null);
  const order = [];
  for (let li = 0; li < lines.length; li++) {
    const parts = lines[li].trim().split(/\s+/);
    if (parts.length === 0 || parts[0] !== "default") continue;
    let di = -1;
    let mi = -1;
    for (let i = 0; i < parts.length; i++) {
      if (parts[i] === "dev" && i + 1 < parts.length) di = i;
      if (parts[i] === "metric" && i + 1 < parts.length) mi = i;
    }
    if (di < 0 || mi < 0) continue;
    let iface = parts[di + 1].replace(/[^a-zA-Z0-9._-]/g, "");
    if (!iface || seen[iface]) continue;
    seen[iface] = true;
    order.push(iface);
  }
  return order;
}

/**
 * Split `block` into lines and keep only the ones that look like interface
 * names: non-blank, not a `#` comment, and matching `[a-zA-Z0-9._-]+`.
 * Shared by every place that reads a "one interface name per line" section
 * of the poll snapshot (the `---mptcp-*---`, `---backup---` etc. blocks).
 */
function parseIfaceNameLines(block) {
  const out = [];
  const lines = String(block || "").split(/\r?\n/);
  for (let i = 0; i < lines.length; i++) {
    const line = lines[i].trim().replace(/\r/g, "");
    if (line === "" || line.charAt(0) === "#") continue;
    if (!/^[a-zA-Z0-9._-]+$/.test(line)) continue;
    out.push(line);
  }
  return out;
}

/** Set of interface names from a "one name per line" block — see `parseIfaceNameLines`. */
function parseIfaceLineSet(block) {
  return new Set(parseIfaceNameLines(block));
}

/** Strip leading `Content-Type:` block CGI may print before `---` markers. */
function stripCgiStdoutPrefix(raw) {
  const t = String(raw || "").replace(/\r/g, "");
  if (!/^Content-Type:/im.test(t)) return t;
  const dbl = t.indexOf("\n\n");
  if (dbl >= 0) return t.slice(dbl + 2);
  const cr = t.indexOf("\r\n\r\n");
  if (cr >= 0) return t.slice(cr + 4);
  return t;
}

// ============================================================================
// Poll snapshot fetching
// ============================================================================
//
// The `?poll` CGI endpoint returns one plain-text snapshot per request,
// containing several `---section-name---`-delimited blocks (see
// `POLL_MARKERS` below). We fetch it once per second and hand the raw text to
// every registered poll hook.

/** Last successful `?poll` body (after `stripCgiStdoutPrefix`). Updated once per second. */
let lastPollSnapshotText = null;

/** Functions called with the raw poll text on every successful poll cycle — see `registerClientPollHook`. */
const clientPollHooks = [];

/** Register a function to be called with the raw `?poll` text on every poll cycle. */
function registerClientPollHook(fn) {
  clientPollHooks.push(fn);
}

/** Start a window during which poll cycles only update the speed charts (see `CHART_ONLY_MS`). */
function enterChartOnlyMode() {
  chartOnlyUntil = Date.now() + CHART_ONLY_MS;
}

/** Fetch one `?poll` snapshot and return its body, with the CGI header stripped. */
async function httpFetchPollTextOnce() {
  const res = await fetch("/cgi-bin/bsbf-client-web?poll", { cache: "no-store" });
  if (!res.ok) throw new Error("poll HTTP " + res.status);
  return stripCgiStdoutPrefix(await res.text());
}

/** Guards against overlapping poll cycles if a fetch is slower than `POLL_INTERVAL`. */
let clientPollCycleRunning = false;

/** Fetch one poll snapshot and run every registered hook against it; skips if a cycle is already running. */
async function runClientPollCycle() {
  if (clientPollCycleRunning) return;
  clientPollCycleRunning = true;
  try {
    const text = await httpFetchPollTextOnce();
    lastPollSnapshotText = text;
    for (let i = 0; i < clientPollHooks.length; i++) {
      await clientPollHooks[i](text);
    }
  } catch (_) {
    /* ignore transient poll failures */
  } finally {
    clientPollCycleRunning = false;
  }
}

/**
 * Section markers that divide up the `?poll` response body, in the order
 * they appear in the text. `slicePollSection` uses this order to find where
 * one section ends (the start of the next marker that actually appears).
 */
const POLL_MARKERS = [
  "---bonding-status---",
  "---interfaces---",
  "---mptcp-no-connectivity---",
  "---mptcp-not-supported---",
  "---mptcp-supported---",
  "---dl-ul-limit---",
  "---mptcp-subflow-backup---",
  "---dl-ul-throughput---",
];

/**
 * Return the text between `marker` and the next marker that follows it in
 * `POLL_MARKERS` (or end of string), with a single leading newline stripped.
 * Returns "" if `marker` isn't a known marker or isn't present in `raw`.
 */
function slicePollSection(raw, marker) {
  const idx = POLL_MARKERS.indexOf(marker);
  if (idx < 0) return "";
  const start = raw.indexOf(marker);
  if (start < 0) return "";
  const from = start + marker.length;
  let end = raw.length;
  for (let j = idx + 1; j < POLL_MARKERS.length; j++) {
    const n = raw.indexOf(POLL_MARKERS[j], from);
    if (n >= 0 && n < end) end = n;
  }
  return raw.slice(from, end).replace(/^\r?\n/, "");
}

/** Ordered, de-duplicated iface names from `---interfaces---` (one name per line). */
function parseIfaceListBlock(block) {
  const order = [];
  const seen = Object.create(null);
  const lines = parseIfaceNameLines(block);
  for (let i = 0; i < lines.length; i++) {
    const line = lines[i];
    if (seen[line]) continue;
    seen[line] = true;
    order.push(line);
  }
  return order;
}

/** Read the `---bonding-status---` line as lowercase text (expected: "enabled" or "disabled"). */
function parseBondingStatusLineFromPoll(raw) {
  const block = slicePollSection(raw, "---bonding-status---").trim();
  const line = block.split(/\r?\n/)[0] || "";
  return line.trim().toLowerCase();
}

/**
 * Parse `---dl-ul-throughput---`: for each interface (in `serverIfaceOrder`,
 * the same order as `---interfaces---`), the section has two lines — RX byte
 * counter, then TX byte counter. Returns `{ [iface]: { rx_bytes, tx_bytes } }`
 * (raw string counters; converted to rates later in `applyStatsFromThroughputMap`).
 */
function parseThroughputMapFromPoll(raw, serverIfaceOrder) {
  const section = slicePollSection(raw, "---dl-ul-throughput---");
  const lines = section.split(/\r?\n/).map(function (l) {
    return l.trim();
  });
  const out = {};
  for (let i = 0; i < serverIfaceOrder.length; i++) {
    const rxI = 2 * i;
    const txI = 2 * i + 1;
    if (txI >= lines.length) break;
    const rx = lines[rxI];
    const tx = lines[txI];
    if (rx === "" || tx === "") continue;
    out[serverIfaceOrder[i]] = { tx_bytes: tx, rx_bytes: rx };
  }
  return out;
}

/** Set of interface names currently in MPTCP "backup" (lower-priority) role, from `---mptcp-subflow-backup---`. */
function subflowBackupSetFromPollText(text) {
  return parseIfaceLineSet(slicePollSection(text, "---mptcp-subflow-backup---"));
}

/**
 * Split `---dl-ul-limit---` body into 2×N tc outputs (IFB then iface per row in
 * `---interfaces---`). Empty `tc class show` is one line `0`. Multiline output is
 * split on lines that are exactly `0` only when that yields exactly 2×N parts;
 * otherwise require exactly 2×N non-empty lines (one line per tc output).
 */
function splitDlUlLimitIntoTcParts(body, numIfaces) {
  const want = numIfaces * 2;
  const raw = String(body || "").replace(/\r/g, "").trim();
  if (numIfaces <= 0 || !raw) return [];
  const byZero = raw
    .split(/\n(?=^0$)/m)
    .map(function (s) {
      return s.trim();
    })
    .filter(Boolean);
  if (byZero.length === want) return byZero;
  const nonEmpty = raw.split(/\r?\n/).filter(function (l) {
    return String(l).replace(/\r/g, "").trim().length > 0;
  });
  if (nonEmpty.length === want) return nonEmpty.slice();
  return [];
}

/**
 * Parse `---dl-ul-limit---` into per-interface normalized cap rows, using the
 * 2×N parts produced by `splitDlUlLimitIntoTcParts` (IFB/download chunk then
 * upload chunk for each interface, in `serverIfaceOrder`). Returns `{}` if
 * the section couldn't be split into exactly 2×N parts.
 */
function parseTcClassBatchFromPollLimit(raw, serverIfaceOrder) {
  const section = slicePollSection(raw, "---dl-ul-limit---");
  const n = serverIfaceOrder.length;
  const parts = splitDlUlLimitIntoTcParts(section, n);
  const out = {};
  if (parts.length < 2 * n) return out;
  for (let i = 0; i < n; i++) {
    const ifb = parts[2 * i];
    const ifaceTc = parts[2 * i + 1];
    const name = serverIfaceOrder[i];
    const tagged = "__dl__" + ifb + "__ul__" + ifaceTc;
    out[name] = parseTcSingleIfaceChunk(tagged);
  }
  return out;
}

/**
 * Combine an interface list with the connectivity/MPTCP-support sets and the
 * backup list into the `{ interfaces, subflow_backup }` shape returned by
 * both `parsePollSnapshotText` and `parseInterfacesSnapshotText`.
 */
function buildInterfacesResult(list, noConn, ns, sup, backupRaw) {
  const interfaces = list.map(function (iface) {
    return {
      iface: iface,
      no_connectivity: noConn.has(iface),
      mptcp_not_supported: ns.has(iface),
      mptcp_supported: sup.has(iface),
    };
  });
  return {
    interfaces: interfaces,
    subflow_backup: parseIfaceNameLines(backupRaw),
  };
}

/**
 * Parse the modern `---interfaces---`-style poll snapshot into
 * `{ interfaces: [{iface, no_connectivity, mptcp_not_supported, mptcp_supported}, ...], subflow_backup: [...] }`.
 * Throws if the snapshot is an `ERR\t<message>` error response.
 */
function parsePollSnapshotText(raw) {
  const text = stripCgiStdoutPrefix(raw);
  if (text.trimStart().startsWith("ERR\t")) {
    throw new Error(text.trim().split(/\t/)[1] || "interfaces error");
  }
  const list = parseIfaceListBlock(slicePollSection(text, "---interfaces---"));
  const noConn = parseIfaceLineSet(
    slicePollSection(text, "---mptcp-no-connectivity---")
  );
  const ns = parseIfaceLineSet(
    slicePollSection(text, "---mptcp-not-supported---")
  );
  const sup = parseIfaceLineSet(slicePollSection(text, "---mptcp-supported---"));
  const backupRaw = slicePollSection(text, "---mptcp-subflow-backup---");
  return buildInterfacesResult(list, noConn, ns, sup, backupRaw);
}

/**
 * Top-level entry point for parsing a `?poll` snapshot into the interface
 * list + flags. Two formats are supported:
 *  - Modern: contains `---interfaces---` -> delegates to `parsePollSnapshotText`.
 *  - Legacy: `---routes---` / `---no_conn---` / `---mptcp_ns---` /
 *    `---mptcp_sup---` / `---backup---` markers, where the interface list
 *    comes from parsing `ip route` default-route lines instead of an
 *    explicit interface list.
 * Throws if the snapshot is an `ERR\t<message>` error response.
 */
function parseInterfacesSnapshotText(text) {
  const raw = String(text || "").replace(/\r/g, "");
  if (raw.indexOf("---interfaces---") >= 0) {
    return parsePollSnapshotText(raw);
  }
  if (raw.trimStart().startsWith("ERR\t")) {
    throw new Error(raw.trim().split(/\t/)[1] || "interfaces error");
  }
  const markers = [
    "---routes---",
    "---no_conn---",
    "---mptcp_ns---",
    "---mptcp_sup---",
    "---backup---",
  ];
  const parts = {};
  for (let m = 0; m < markers.length; m++) {
    const mark = markers[m];
    const start = raw.indexOf(mark);
    if (start < 0) {
      parts[mark] = "";
      continue;
    }
    const contentStart = start + mark.length;
    let next = raw.length;
    for (let n = m + 1; n < markers.length; n++) {
      const ns = raw.indexOf(markers[n], contentStart);
      if (ns >= 0 && ns < next) next = ns;
    }
    parts[mark] = raw.slice(contentStart, next).replace(/^\n/, "");
  }
  const list = parseDefaultRouteIfaces(parts["---routes---"]);
  const noConn = parseIfaceLineSet(parts["---no_conn---"]);
  const ns = parseIfaceLineSet(parts["---mptcp_ns---"]);
  const sup = parseIfaceLineSet(parts["---mptcp_sup---"]);
  return buildInterfacesResult(list, noConn, ns, sup, parts["---backup---"] || "");
}

// ============================================================================
// `tc` cap normalization & batch fetching
// ============================================================================

/**
 * Normalize a cap row to `{ unlimited, dl_unlimited, ul_unlimited, dl_mbps, ul_mbps }`,
 * accepting a few input shapes for backwards compatibility:
 *  - `{ unlimited: true }` or any falsy/error input -> fully unlimited.
 *  - Old single-direction shape: `{ dl_mbps|max_mbps, ul_mbps|max_mbps }` with
 *    no `dl_unlimited`/`ul_unlimited` keys -> both directions limited to that Mbps.
 *  - New per-direction shape: `{ dl_unlimited, ul_unlimited, dl_mbps, ul_mbps }`.
 * Any invalid/negative Mbps value (for a direction that isn't unlimited)
 * falls back to fully unlimited, so a bad cap never silently throttles traffic.
 */
function normalizeTcJson(j) {
  const empty = {
    unlimited: true,
    dl_unlimited: true,
    ul_unlimited: true,
    dl_mbps: 0,
    ul_mbps: 0,
  };
  if (!j || j.error) return empty;
  if (j.unlimited === true) return empty;

  const hasPerDir =
    j.dl_unlimited !== undefined || j.ul_unlimited !== undefined;

  if (!hasPerDir) {
    const dl = Number(j.dl_mbps != null ? j.dl_mbps : j.max_mbps);
    const ul = Number(j.ul_mbps != null ? j.ul_mbps : j.max_mbps);
    if (!isFinite(dl) || !isFinite(ul) || dl < 0 || ul < 0) return empty;
    return {
      unlimited: false,
      dl_unlimited: false,
      ul_unlimited: false,
      dl_mbps: dl,
      ul_mbps: ul,
    };
  }

  const dlUnl = j.dl_unlimited === true;
  const ulUnl = j.ul_unlimited === true;
  const dl = dlUnl ? 0 : Number(j.dl_mbps != null ? j.dl_mbps : 0);
  const ul = ulUnl ? 0 : Number(j.ul_mbps != null ? j.ul_mbps : 0);
  if (!dlUnl && (!isFinite(dl) || dl < 0)) return empty;
  if (!ulUnl && (!isFinite(ul) || ul < 0)) return empty;

  return {
    unlimited: dlUnl && ulUnl,
    dl_unlimited: dlUnl,
    ul_unlimited: ulUnl,
    dl_mbps: dl,
    ul_mbps: ul,
  };
}

/**
 * Fetch normalized caps for a batch of interfaces. Prefers data already
 * present in `pollTextOpt` (or the last poll snapshot) over making a fresh
 * request. On any error, or if the snapshot is an `ERR\t` response, every
 * requested interface gets the "unlimited" fallback rather than failing.
 */
async function fetchTcClassBatch(ifaces, pollTextOpt) {
  const empty = normalizeTcJson({ unlimited: true });
  if (!ifaces || ifaces.length === 0) return {};
  let text;
  try {
    text =
      pollTextOpt != null
        ? pollTextOpt
        : lastPollSnapshotText != null
          ? lastPollSnapshotText
          : await httpFetchPollTextOnce();
  } catch (_) {
    const o = {};
    for (let i = 0; i < ifaces.length; i++) o[ifaces[i]] = empty;
    return o;
  }
  if (text.trimStart().startsWith("ERR\t")) {
    const o = {};
    for (let i = 0; i < ifaces.length; i++) o[ifaces[i]] = empty;
    return o;
  }
  const serverIfaces = parseIfaceListBlock(
    slicePollSection(text, "---interfaces---")
  );
  const fromPoll = parseTcClassBatchFromPollLimit(text, serverIfaces);
  let parsed = fromPoll;
  if (Object.keys(fromPoll).length === 0) {
    const limitSec = slicePollSection(text, "---dl-ul-limit---");
    parsed = parseTcClassBatchText(limitSec);
  }
  const o = {};
  for (let i = 0; i < ifaces.length; i++) {
    const n = ifaces[i];
    o[n] = parsed[n] != null ? parsed[n] : empty;
  }
  return o;
}

/** Convenience single-interface wrapper around `fetchTcClassBatch`. */
async function fetchTcClass(iface) {
  const m = await fetchTcClassBatch([iface]);
  const empty = normalizeTcJson({ unlimited: true });
  return m[iface] != null ? m[iface] : empty;
}

/** Record a normalized tc cap row as both the "reference" (`tcInfo`) and the editable/sortable cap (`sortCap`) for an interface. */
function applyTcRowToCapState(iface, row) {
  tcInfo[iface] = row;
  sortCap[iface] = {
    dlUnl: !!row.dl_unlimited,
    ulUnl: !!row.ul_unlimited,
    dlMbps: row.dl_mbps || 0,
    ulMbps: row.ul_mbps || 0,
  };
}

/** Guards `applyTcClassSettingsFromPollText` against overlapping runs across poll cycles. */
let tcClassPollInFlight = false;

/** Refresh Download/Upload limits from `tc class` in a poll snapshot when the user has no pending edits on that card. */
async function applyTcClassSettingsFromPollText(text) {
  if (tcClassPollInFlight || ifaceOrder.length === 0) return;
  tcClassPollInFlight = true;
  try {
    const targets = ifaceOrder.filter(function (iface) {
      return !ifaceHasPendingChanges(iface);
    });
    if (targets.length === 0) return;
    const batch = await fetchTcClassBatch(targets, text);
    const empty = normalizeTcJson({ unlimited: true });
    for (let i = 0; i < targets.length; i++) {
      const iface = targets[i];
      const row = batch[iface] != null ? batch[iface] : empty;
      applyTcRowToCapState(iface, row);
      syncCapUIFromSortCap(iface);
    }
  } finally {
    tcClassPollInFlight = false;
  }
}

// ============================================================================
// Card ordering & FLIP reorder animation
// ============================================================================

/** True if an interface's cap is "fully unlimited" (both download and upload). */
function fullUnlimitedCap(c) {
  return !!(c && c.dlUnl && c.ulUnl);
}

/**
 * Sort comparator for `ifaceOrder`. Priority order:
 *  1. Fully-unlimited interfaces first (tie-broken by original discovery order)
 *     — these are the "primary" links.
 *  2. Among limited interfaces, higher download Mbps first (unlimited download
 *     counts as infinite).
 *  3. Then higher upload Mbps first (same unlimited-as-infinite rule).
 *  4. Finally, original discovery order as the last tie-break.
 * This keeps the highest-capacity / primary links at the top of the list.
 */
function compareIfacesForSort(a, b) {
  const ca = sortCap[a];
  const cb = sortCap[b];
  const ia = origIfaceIndex[a] ?? 9999;
  const ib = origIfaceIndex[b] ?? 9999;
  const aFu = fullUnlimitedCap(ca);
  const bFu = fullUnlimitedCap(cb);
  if (aFu && bFu) return ia - ib;
  if (aFu) return -1;
  if (bFu) return 1;

  const dlA = ca && ca.dlUnl ? Infinity : (ca && ca.dlMbps) || 0;
  const dlB = cb && cb.dlUnl ? Infinity : (cb && cb.dlMbps) || 0;
  if (dlB !== dlA) return dlB - dlA;

  const ulA = ca && ca.ulUnl ? Infinity : (ca && ca.ulMbps) || 0;
  const ulB = cb && cb.ulUnl ? Infinity : (cb && cb.ulMbps) || 0;
  if (ulB !== ulA) return ulB - ulA;

  return ia - ib;
}

/** True if the user's OS/browser asks to minimize motion — disables the reorder slide animation. */
function prefersReducedMotion() {
  return (
    typeof matchMedia === "function" &&
    matchMedia("(prefers-reduced-motion: reduce)").matches
  );
}

/** Re-append each card to `container` in `namesInOrder`, which reorders them in the DOM (and visually, once any animation settles). */
function reorderDomInOrder(container, namesInOrder) {
  if (!container) return;
  for (let k = 0; k < namesInOrder.length; k++) {
    const name = namesInOrder[k];
    const el = document.getElementById("card-" + name);
    if (el) container.appendChild(el);
  }
}

/** Remove the inline styles used to make a card float and follow the pointer during a drag. */
function clearDragFloatStyles(card) {
  if (!card) return;
  card.classList.remove("iface-card-floating");
  card.style.left = "";
  card.style.top = "";
  card.style.width = "";
  card.style.position = "";
  card.style.transform = "";
}

/**
 * Reorder all cards in `container` to match `namesInOrder`, animating any
 * card that visually moves using the "FLIP" technique
 * (First, Last, Invert, Play):
 *   1. FIRST  — record each card's current on-screen position.
 *   2. (reorder the DOM to the new order — cards jump instantly)
 *   3. LAST   — record each card's new on-screen position.
 *   4. INVERT — for cards whose position changed, apply a transform that
 *               moves them back to where they were (so visually nothing
 *               has moved yet).
 *   5. PLAY   — on the next frame, transition that transform back to
 *               `translate(0,0)`, animating the card sliding from its old
 *               spot to its new one.
 *
 * If `dragContext` is given, the order change came from a pointer drag:
 *  - `dragContext.placeholder` is the empty placeholder element left in the
 *    drop target's old slot; it's removed as part of step 1->2 so that
 *    cards sliding into the vacated slot animate from their pre-drop position.
 *  - `dragContext.card` is the dragged card itself, which is still
 *    absolutely positioned/floating; it gets dropped back into normal flow
 *    (via `clearDragFloatStyles`) and temporarily raised above its siblings
 *    (`z-index`) so it stays visible while sliding into its new slot.
 *
 * `raiseIface`, if given, names a card (e.g. the one the user just clicked
 * "Apply" on) that should likewise be temporarily raised above its siblings
 * while it animates into its new slot — independent of `dragContext`.
 *
 * If the user prefers reduced motion, skip the animation entirely and just
 * reorder the DOM.
 */
function animateReorderCards(container, namesInOrder, dragContext, raiseIface) {
  if (!container || namesInOrder.length === 0) return;

  const raiseEl = raiseIface ? document.getElementById("card-" + raiseIface) : null;

  const cards = [];
  for (let i = 0; i < namesInOrder.length; i++) {
    const el = document.getElementById("card-" + namesInOrder[i]);
    if (el) cards.push(el);
  }
  if (cards.length === 0) return;

  if (prefersReducedMotion()) {
    if (dragContext && dragContext.placeholder) {
      dragContext.placeholder.remove();
    }
    reorderDomInOrder(container, namesInOrder);
    if (dragContext && dragContext.card) {
      clearDragFloatStyles(dragContext.card);
      dragContext.card.style.zIndex = "";
    }
    return;
  }

  // FIRST: record each card's position before reordering.
  const firstRects = new Map();
  for (let i = 0; i < cards.length; i++) {
    const el = cards[i];
    firstRects.set(el, el.getBoundingClientRect());
  }

  // Remove the drag placeholder *after* recording "first" rects, so cards that
  // shift into its vacated slot animate from their pre-drop position too.
  if (dragContext && dragContext.placeholder) {
    dragContext.placeholder.remove();
  }

  // Reorder the DOM — cards visually jump to their final positions immediately.
  reorderDomInOrder(container, namesInOrder);

  if (dragContext && dragContext.card) {
    clearDragFloatStyles(dragContext.card);
    // Keep the dropped card above its now-overlapping siblings while it
    // animates into its new slot (e.g. sliding to the top of the stack).
    dragContext.card.style.zIndex = "10000";
  }
  if (raiseEl) {
    // Likewise keep the card the user just applied changes to above its
    // now-overlapping siblings while it slides into its new slot.
    raiseEl.style.zIndex = "10000";
  }

  // Force a layout flush so the LAST rects below reflect the new DOM order.
  void container.offsetHeight;

  // LAST + INVERT: for any card whose position changed, jump it back to its
  // "first" position using a transform (with transitions disabled), so it
  // appears not to have moved yet.
  const animated = [];
  for (let i = 0; i < cards.length; i++) {
    const el = cards[i];
    const first = firstRects.get(el);
    const last = el.getBoundingClientRect();
    const dx = first.left - last.left;
    const dy = first.top - last.top;
    if (Math.abs(dx) < 0.5 && Math.abs(dy) < 0.5) continue;
    animated.push(el);
    el.style.transformOrigin = "0 0";
    el.style.transform = "translate(" + dx + "px, " + dy + "px)";
    el.style.transition = "none";
  }

  if (animated.length === 0) {
    if (dragContext && dragContext.card) {
      dragContext.card.style.zIndex = "";
    }
    if (raiseEl) raiseEl.style.zIndex = "";
    return;
  }

  // Flush again so the "no transition" transform above is applied before we
  // enable transitions below — otherwise the browser may coalesce both style
  // changes into a single frame and skip the animation.
  void container.offsetHeight;

  // PLAY: on the next frame, enable the transition and animate back to
  // translate(0,0) — the card visibly slides into its real position.
  requestAnimationFrame(function () {
    requestAnimationFrame(function () {
      for (let i = 0; i < animated.length; i++) {
        const el = animated[i];
        el.style.transition =
          "transform " + REORDER_ANIM_MS + "ms " + REORDER_ANIM_EASING;
        el.style.transform = "translate(0, 0)";
      }
    });
  });

  window.setTimeout(function () {
    for (let i = 0; i < animated.length; i++) {
      const el = animated[i];
      el.style.transform = "";
      el.style.transition = "";
      el.style.transformOrigin = "";
    }
    if (dragContext && dragContext.card) {
      dragContext.card.style.zIndex = "";
    }
    if (raiseEl) raiseEl.style.zIndex = "";
  }, REORDER_ANIM_MS + 60);
}

/**
 * Re-sort `ifaceOrder` by current caps (`compareIfacesForSort`), animate the
 * cards into their new positions, and push the resulting limits to the
 * router via `applyRateLimits`. Resolves once the `applyRateLimits` request
 * has finished — the slide animation runs independently and is not awaited,
 * so callers (e.g. the per-card "Apply" button) reflect only the CGI call's
 * progress, not the animation's.
 * @param {string[]|undefined} limitApplyToIfaces  If set and non-empty, only those names get `limit&` CGI calls (e.g. per-card Apply). Omit for all interfaces (e.g. after drag tiering).
 * @param {string|undefined} raiseIface  If set, that card is temporarily raised above its siblings (like the dragged card) while it animates into its new slot — used so the card the user just clicked "Apply" on stays visible.
 */
function resortBySortCap(limitApplyToIfaces, raiseIface) {
  ifaceOrder.sort(compareIfacesForSort);
  const container = document.getElementById("interfaces");
  animateReorderCards(container, ifaceOrder.slice(), undefined, raiseIface);
  return applyRateLimits(limitApplyToIfaces);
}

// ============================================================================
// Per-card cap controls (Download/Upload inputs, Unlimited checkboxes, Apply)
// ============================================================================

/**
 * Wire up one card's cap controls: Download/Upload number inputs, their
 * "Unlimited" checkboxes, the "Backup" checkbox, and the "Apply" button.
 * Also triggers the initial WAN IP/ISP lookup for this card.
 */
function bindSpeedControls(iface) {
  const card = document.getElementById("card-" + iface);
  if (!card) return;
  const inpDl = card.querySelector(".cap-dl-input");
  const inpUl = card.querySelector(".cap-ul-input");
  const cbDl = card.querySelector(".cap-dl-unl");
  const cbUl = card.querySelector(".cap-ul-unl");

  syncCapUIFromSortCap(iface);

  function readCapFromInputs() {
    return readCapFromInputsForIface(iface);
  }

  // When "Unlimited" is unchecked for a direction, the number input becomes
  // editable again. If it's currently empty, seed it with a sensible value:
  // the last known tc limit for that direction if there was one, otherwise "1".
  function onDlUnlChange() {
    if (cbDl.checked) {
      inpDl.disabled = true;
      inpDl.value = "";
    } else {
      inpDl.disabled = false;
      if (!String(inpDl.value).trim()) {
        const base = tcInfo[iface];
        if (base && base.dl_unlimited !== true && base.unlimited !== true)
          inpDl.value = formatMbpsShort(base.dl_mbps);
        else inpDl.value = "1";
      }
    }
  }

  // Same as `onDlUnlChange`, for the upload direction.
  function onUlUnlChange() {
    if (cbUl.checked) {
      inpUl.disabled = true;
      inpUl.value = "";
    } else {
      inpUl.disabled = false;
      if (!String(inpUl.value).trim()) {
        const base = tcInfo[iface];
        if (base && base.ul_unlimited !== true && base.unlimited !== true)
          inpUl.value = formatMbpsShort(base.ul_mbps);
        else inpUl.value = "1";
      }
    }
  }

  cbDl.addEventListener("change", function () {
    onDlUnlChange();
    updateCapApplyButtonState(iface);
  });
  cbUl.addEventListener("change", function () {
    onUlUnlChange();
    updateCapApplyButtonState(iface);
  });
  inpDl.addEventListener("input", function () {
    updateCapApplyButtonState(iface);
  });
  inpUl.addEventListener("input", function () {
    updateCapApplyButtonState(iface);
  });
  const cbFbInit = card.querySelector(".cap-force-backup");
  if (cbFbInit) {
    cbFbInit.addEventListener("change", function () {
      updateCapApplyButtonState(iface);
    });
  }

  const btnApply = card.querySelector(".btn-cap-apply");
  if (btnApply) {
    // Apply: push whatever changed (cap and/or backup flag) to the router.
    // - Cap change: update `sortCap`, then resort/animate cards and send the
    //   new limit via `applyRateLimits` (only for this interface).
    // - Backup change: send `?backup&`/`?nobackup&` via `applyForceBackup`.
    // Both run in sequence (cap first) only if needed; the button shows a
    // loading state until everything settles, then re-enters chart-only mode
    // so the next poll(s) don't immediately overwrite the just-applied values.
    btnApply.addEventListener("click", function () {
      const next = readCapFromInputs();
      if (!next) {
        syncCapUIFromSortCap(iface);
        return;
      }
      const prevCap = sortCap[iface];
      const capChanged = !prevCap || !capsEqual(prevCap, next);
      const cbFb = card.querySelector(".cap-force-backup");
      const wantBackup = !!(cbFb && cbFb.checked);
      const savedFb = !!(ifaceFlags[iface] && ifaceFlags[iface].forceBackupConfig);
      const backupChanged = wantBackup !== savedFb;
      if (!capChanged && !backupChanged) {
        return;
      }
      if (capChanged) {
        sortCap[iface] = next;
      }
      // Enter chart-only mode right away so a poll that lands while the
      // CGI calls below are in flight doesn't re-read the router's
      // not-yet-updated `tc` settings and flip the inputs back to their
      // old values before snapping to the new ones.
      enterChartOnlyMode();
      setCapApplyButtonLoading(iface, true);
      let chain = Promise.resolve();
      if (capChanged) {
        chain = chain.then(function () {
          return resortBySortCap([iface], iface);
        });
      }
      if (backupChanged) {
        chain = chain.then(function () {
          return applyForceBackup(iface, wantBackup);
        });
      }
      chain.finally(function () {
        setCapApplyButtonLoading(iface, false);
        enterChartOnlyMode();
      });
    });
  }

  updateCapApplyButtonState(iface);
  refreshIfaceWanIsp(iface);
}

// ============================================================================
// WAN IP / ISP lookup
// ============================================================================

/** Host runs `curl https://ipwho.is --interface iface`; on success only, show IP · ISP · CC (no loading text). */
async function refreshIfaceWanIsp(iface) {
  const el = document.getElementById("wan-" + iface);
  if (!el) return;
  el.textContent = "";
  el.hidden = true;
  try {
    const res = await fetch(
      "/cgi-bin/bsbf-client-web?ip-info&" + encodeURIComponent(iface),
      { cache: "no-store" }
    );
    const raw = stripCgiStdoutPrefix(await res.text());
    if (!res.ok) {
      throw new Error("HTTP " + res.status);
    }
    let j;
    try {
      j = JSON.parse(raw.trim());
    } catch (_) {
      throw new Error("invalid JSON from ipwho.is");
    }
    const ip = j.ip;
    const isp = j.connection && j.connection.isp;
    const cc = j.country_code;
    if (!ip) throw new Error("no address in response");
    if (!isp) throw new Error("no ISP in response");
    el.hidden = false;
    el.textContent =
      ip + " · " + isp + (cc ? " · " + String(cc) : "");
  } catch (_) {
    el.textContent = "";
    el.hidden = true;
  }
}

// ============================================================================
// Card creation & drag-to-reorder
// ============================================================================

/**
 * Build the DOM for one interface card (header with name/badges/rate,
 * cap-editing panel, and graph canvas), wire up its drag handle, and return
 * the (not-yet-attached) card element. Caller is responsible for appending it
 * to the `#interfaces` container and calling `bindSpeedControls`.
 *
 * Drag-to-reorder works via Pointer Events on the `.drag-handle`:
 *  - pointerdown: detach the card visually (absolute position, "floating"),
 *    leave a placeholder in its place so layout doesn't jump, and start
 *    tracking pointer movement.
 *  - pointermove: move the floating card with the pointer, and highlight
 *    whichever other card is currently under the pointer ("drag-over").
 *  - pointerup/cancel: if dropped over another card, reorder via
 *    `reorderInterfaces` (which re-attaches this card via the FLIP
 *    animation); otherwise snap back into place.
 */
function createCard(iface) {
  const card = document.createElement("div");
  card.className = "iface-card";
  card.id = "card-" + iface;
  card.dataset.iface = iface;
  card.innerHTML = `
    <div class="iface-header">
      <span class="drag-handle" title="Drag to reorder">&#9776;</span>
      <div class="iface-header-body">
        <div class="iface-header-main">
          <div class="iface-title-row">
            <span class="iface-name">${iface}</span>
            <span class="iface-badges">
              <span class="badge badge-no-conn" id="badge-no-conn-${iface}" hidden>No connectivity</span>
              <span class="badge badge-mptcp-ns" id="badge-mptcp-ns-${iface}" hidden>MPTCP not supported</span>
              <span class="badge badge-mptcp-sup" id="badge-mptcp-sup-${iface}" hidden>MPTCP supported</span>
            </span>
          </div>
          <span class="iface-rate" id="rate-${iface}">
            <span class="rx">RX ---</span> / <span class="tx">TX ---</span>
          </span>
        </div>
        <div class="iface-wan" id="wan-${iface}" hidden></div>
      </div>
    </div>
    <div class="iface-cap-panel">
      <div class="iface-cap-row">
        <div class="iface-cap-group iface-cap-dl">
          <span class="iface-cap-label">Download</span>
          <input type="number" class="cap-dl-input" min="0" step="any" inputmode="decimal" title="Mbps" />
          <label class="cap-unl-wrap"><input type="checkbox" class="cap-dl-unl" /> Unlimited</label>
        </div>
        <div class="iface-cap-group iface-cap-ul">
          <span class="iface-cap-label">Upload</span>
          <input type="number" class="cap-ul-input" min="0" step="any" inputmode="decimal" title="Mbps" />
          <label class="cap-unl-wrap"><input type="checkbox" class="cap-ul-unl" /> Unlimited</label>
        </div>
        <div class="iface-cap-group iface-cap-force">
          <label class="cap-force-backup-wrap">
            <input type="checkbox" class="cap-force-backup" ${
              ifaceFlags[iface] && ifaceFlags[iface].forceBackupConfig
                ? "checked"
                : ""
            } />
            Backup
          </label>
        </div>
        <div class="iface-cap-apply-wrap">
          <button type="button" class="btn-cap-apply">
            <span class="btn-cap-apply-label">Apply</span>
            <span class="btn-cap-apply-spinner" aria-hidden="true"></span>
          </button>
        </div>
      </div>
    </div>
    <div class="graph-container">
      <canvas id="canvas-${iface}"></canvas>
    </div>
  `;
  applyMptcpBadgesForIface(iface);
  const handle = card.querySelector(".drag-handle");
  handle.addEventListener("pointerdown", (e) => {
    if (e.button !== 0) return; // left button only
    if (panelDragActive) {
      // Another card is already being dragged — ignore (no multi-drag).
      e.preventDefault();
      return;
    }
    e.preventDefault();

    // Detach the card visually: fix it at its current screen position with
    // `position: absolute` (".iface-card-floating", set via CSS) and leave a
    // same-height placeholder behind so the layout doesn't collapse.
    const rect = card.getBoundingClientRect();
    const ph = document.createElement("div");
    ph.className = "iface-card-placeholder";
    ph.style.height = card.offsetHeight + "px";
    card.parentNode.insertBefore(ph, card);

    panelDragActive = true;
    const pointerId = e.pointerId;

    // Pointer capture keeps move/up events targeted at `handle` even if the
    // pointer leaves it; if unsupported, fall back to listening on `document`.
    let useDocumentFallback = false;
    try {
      handle.setPointerCapture(pointerId);
    } catch (_) {
      useDocumentFallback = true;
    }

    card.classList.add("iface-card-floating");
    card.style.width = rect.width + "px";
    card.style.left = rect.left + "px";
    card.style.top = rect.top + "px";

    // Offset between the pointer and the card's top-left corner, so the card
    // doesn't jump to be centered under the pointer.
    const offsetX = e.clientX - rect.left;
    const offsetY = e.clientY - rect.top;
    draggedIface = iface;

    /** Card under cursor by geometry (stable while dragging over another card). */
    function targetCardAtPoint(clientX, clientY) {
      const container = document.getElementById("interfaces");
      const cards = container.querySelectorAll(".iface-card");
      for (let i = cards.length - 1; i >= 0; i--) {
        const el = cards[i];
        if (el === card) continue;
        const r = el.getBoundingClientRect();
        if (
          clientX >= r.left &&
          clientX < r.right &&
          clientY >= r.top &&
          clientY < r.bottom
        ) {
          return el;
        }
      }
      return null;
    }

    let lastOver = null;

    /** Toggle the "drag-over" highlight class on whichever card the pointer is currently over. */
    function setDragOver(over) {
      if (over === lastOver) return;
      if (lastOver) lastOver.classList.remove("drag-over");
      lastOver = over;
      if (lastOver) lastOver.classList.add("drag-over");
    }

    /** Follow the pointer with the floating card, and highlight the card underneath it. */
    function onMove(ev) {
      if (ev.pointerId !== pointerId) return;
      card.style.left = ev.clientX - offsetX + "px";
      card.style.top = ev.clientY - offsetY + "px";
      setDragOver(targetCardAtPoint(ev.clientX, ev.clientY));
    }

    const moveTarget = useDocumentFallback ? document : handle;
    const endTarget = useDocumentFallback ? document : handle;

    /** On pointerup/cancel: drop onto the card under the pointer (reorder), or snap back if dropped on empty space / itself. */
    function finish(ev) {
      if (ev.pointerId !== pointerId) return;

      moveTarget.removeEventListener("pointermove", onMove);
      endTarget.removeEventListener("pointerup", finish);
      endTarget.removeEventListener("pointercancel", finish);

      try {
        try {
          if (typeof handle.hasPointerCapture === "function" && handle.hasPointerCapture(pointerId)) {
            handle.releasePointerCapture(pointerId);
          }
        } catch (_) {
          /* ignore */
        }

        const over = targetCardAtPoint(ev.clientX, ev.clientY);
        const targetIface = over && over.dataset.iface;

        setDragOver(null);

        if (targetIface && targetIface !== iface) {
          // Dropped onto another card: reorder. `reorderInterfaces` handles
          // cleaning up the placeholder and floating styles via FLIP.
          reorderInterfaces(iface, targetIface, { card: card, placeholder: ph });
        } else {
          // Dropped on itself or empty space: just snap back into place.
          ph.remove();
          card.classList.remove("iface-card-floating");
          card.style.left = "";
          card.style.top = "";
          card.style.width = "";
        }
      } finally {
        draggedIface = null;
        panelDragActive = false;
      }
    }

    moveTarget.addEventListener("pointermove", onMove);
    endTarget.addEventListener("pointerup", finish);
    endTarget.addEventListener("pointercancel", finish);
  });
  return card;
}

/**
 * Move `fromIface` to the position of `toIface` in `ifaceOrder`, animate the
 * cards into their new positions, and then re-tier caps as if this were a
 * fresh drag-based reorder: the new top card becomes unlimited, every other
 * card gets `TIER_NON_TOP_FRAC_OF_TOP` of the top card's observed peak speed.
 * Only the interfaces whose cap actually changed as a result (`changedIfaces`)
 * have their new limits pushed to the router and show the Apply-button
 * spinner; if none changed, this returns without pushing anything.
 * @param {object} [dragContext]  After a pointer drag, pass `{ card, placeholder }` (card still floating, placeholder still in flow) so FLIP uses its drop position.
 */
function reorderInterfaces(fromIface, toIface, dragContext) {
  const fromIdx = ifaceOrder.indexOf(fromIface);
  const toIdx = ifaceOrder.indexOf(toIface);
  if (fromIdx === -1 || toIdx === -1 || fromIdx === toIdx) return;

  const next = ifaceOrder.slice();
  const [item] = next.splice(fromIdx, 1);
  // Insert at original target index (after removal). Using toIdx-1 placed the item
  // *before* the drop target, so the top card could never be moved to the true bottom.
  const insertAt = toIdx;
  next.splice(insertAt, 0, item);
  ifaceOrder = next;

  const container = document.getElementById("interfaces");
  animateReorderCards(
    container,
    ifaceOrder.slice(),
    dragContext && dragContext.card ? dragContext : undefined
  );

  // Snapshot the caps as they were before retiering, so we can tell below
  // which interfaces (if any) actually got a different cap — e.g. swapping
  // two non-top cards re-tiers both to the same values they already had
  // (neither becomes the new top reference), and a card that was already
  // unlimited stays unlimited if it becomes the new top.
  const prevCaps = {};
  for (let i = 0; i < ifaceOrder.length; i++) {
    prevCaps[ifaceOrder[i]] = sortCap[ifaceOrder[i]];
  }

  applyTieredCapsFromTopPeaks();
  syncAllCapUIFromSortCap();

  const changedIfaces = ifaceOrder.filter(function (name) {
    return !capsEqual(prevCaps[name], sortCap[name]);
  });
  if (changedIfaces.length === 0) return;

  // Enter chart-only mode right away (and again once the limits have been
  // pushed) so a poll that lands before the router applies the new tiered
  // caps doesn't read its still-stale `tc class show` output and flip the
  // inputs back to their pre-reorder values — see the "Apply" button handler
  // in bindSpeedControls for the same pattern.
  enterChartOnlyMode();
  // Only push (and show the loading spinner for) the cards whose cap
  // actually changed — e.g. a card that was already unlimited shouldn't get
  // re-pushed just because some other card's tier changed.
  for (let i = 0; i < changedIfaces.length; i++) {
    setCapApplyButtonLoading(changedIfaces[i], true);
  }
  applyRateLimits(changedIfaces)
    .catch(function () {})
    .finally(function () {
      enterChartOnlyMode();
      for (let i = 0; i < changedIfaces.length; i++) {
        setCapApplyButtonLoading(changedIfaces[i], false);
      }
    });
}

// ============================================================================
// Speed graphs (canvas)
// ============================================================================

/** Highest peak TX or RX (bytes/s) across all interfaces — shared graph Y scale. */
function getGlobalPeakScaleBytesPerSec() {
  let g = 0;
  for (const name of ifaceOrder) {
    const d = ifaceData[name];
    if (!d) continue;
    const p = Math.max(d.peakTx, d.peakRx);
    if (p > g) g = p;
  }
  return g;
}

/** Read graph colors from CSS custom properties, so graphs follow the active theme. */
function readGraphColors() {
  const cs = getComputedStyle(document.documentElement);
  function c(varName, fallback) {
    const v = cs.getPropertyValue(varName).trim();
    return v || fallback;
  }
  return {
    grid: c("--graph-grid", "#2a2d3a"),
    label: c("--graph-label", "#555"),
    rx: c("--color-rx", "#4fc3f7"),
    tx: c("--color-tx", "#f5a623"),
  };
}

/**
 * Redraw one interface's RX/TX speed graph on its canvas.
 *
 * The canvas is resized to match its container at the current device pixel
 * ratio (so it stays sharp on high-DPI screens), then cleared and redrawn
 * from scratch every call — there's no incremental update.
 *
 * Y-axis scale: all graphs share the same scale, `getGlobalPeakScaleBytesPerSec()`
 * (the highest peak RX/TX seen on *any* interface, plus 10% headroom), so
 * cards are visually comparable. If no interface has any peak yet (e.g. right
 * after page load), fall back to this graph's own highest sample.
 */
function drawGraph(iface) {
  const data = ifaceData[iface];
  if (!data) return;

  const canvas = document.getElementById("canvas-" + iface);
  if (!canvas) return;

  const rect = canvas.parentElement.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = rect.width * dpr;
  canvas.height = rect.height * dpr;

  const ctx = canvas.getContext("2d");
  ctx.scale(dpr, dpr);

  const w = rect.width;
  const h = rect.height;

  ctx.clearRect(0, 0, w, h);

  const txRates = data.txRates;
  const rxRates = data.rxRates;

  let peakScale = getGlobalPeakScaleBytesPerSec();
  if (peakScale <= 0) {
    for (let i = 0; i < HISTORY_LEN; i++) {
      if (txRates[i] > peakScale) peakScale = txRates[i];
      if (rxRates[i] > peakScale) peakScale = rxRates[i];
    }
  }
  peakScale = peakScale || 1;
  const maxVal = peakScale * 1.1; // 10% headroom above the peak

  const stepX = w / (HISTORY_LEN - 1);

  // Horizontal grid lines at 25/50/75/100% of height.
  const gc = readGraphColors();
  ctx.lineWidth = 1;
  ctx.strokeStyle = gc.grid;
  for (let i = 1; i <= 4; i++) {
    const y = h - (h * i) / 4;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }

  // Draw one rate history as a polyline; negative samples (no data yet) are
  // drawn at 0.
  function drawLine(rates, color) {
    ctx.beginPath();
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.lineJoin = "round";
    for (let i = 0; i < HISTORY_LEN; i++) {
      const x = i * stepX;
      const val = rates[i] >= 0 ? rates[i] : 0;
      const y = h - (val / maxVal) * h;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }

  drawLine(rxRates, gc.rx);
  drawLine(txRates, gc.tx);

  // Y-axis labels: top = max scale, middle = half of it, bottom = 0.
  ctx.fillStyle = gc.label;
  ctx.font = "10px monospace";
  ctx.textAlign = "right";
  ctx.fillText(formatRate(maxVal / 1.1), w - 4, 12);
  ctx.fillText(formatRate(maxVal / 1.1 / 2), w - 4, h / 2 + 4);
  ctx.fillText("0", w - 4, h - 4);
}

// ============================================================================
// Interface discovery & bonding-backup set
// ============================================================================

/** Fetch a fresh poll snapshot and parse it into the interface list + flags (used once at startup). */
async function fetchInterfaces() {
  const text = await httpFetchPollTextOnce();
  lastPollSnapshotText = text;
  return parseInterfacesSnapshotText(text);
}

/** Names from bsbf-mptcp-subflow-backup (one iface per line). */
async function fetchBondingBackupSet() {
  try {
    const t =
      lastPollSnapshotText != null
        ? lastPollSnapshotText
        : await httpFetchPollTextOnce();
    return subflowBackupSetFromPollText(t);
  } catch (_) {
    return new Set();
  }
}

/** Apply server backup membership to checkboxes; skips interfaces with unsaved cap/backup edits. */
function syncBackupUIFromBondingSet(set) {
  for (let i = 0; i < ifaceOrder.length; i++) {
    const iface = ifaceOrder[i];
    if (ifaceHasPendingChanges(iface)) continue;
    const want = set.has(String(iface).trim());
    ifaceFlags[iface] = ifaceFlags[iface] || {};
    ifaceFlags[iface].forceBackupConfig = want;
    const card = document.getElementById("card-" + iface);
    const cb = card && card.querySelector(".cap-force-backup");
    if (cb) cb.checked = want;
    updateCapApplyButtonState(iface);
  }
}

/**
 * Show `extra` in the shared `#rate-limit-msg` error banner — appended to any
 * existing message (separated by "; "), or as a fresh message if the banner
 * was hidden/empty.
 */
function appendRateLimitMsg(extra) {
  const msgEl = document.getElementById("rate-limit-msg");
  if (!msgEl) return;
  if (!msgEl.hidden && msgEl.textContent) {
    msgEl.textContent += "; " + extra;
  } else {
    msgEl.hidden = false;
    msgEl.className = "rate-limit-msg err";
    msgEl.textContent = extra;
  }
}

/**
 * Update subflow backup membership via CGI `?backup&<iface>` / `?nobackup&<iface>`.
 * On success, updates `ifaceFlags[iface].forceBackupConfig` and returns `true`.
 * On any failure (HTTP error, `ERR\t...` response, unexpected body, or network
 * error), reports it via `appendRateLimitMsg` and returns `false` — the caller
 * (`bindSpeedControls`'s Apply handler) leaves the checkbox as the user left it.
 */
async function applyForceBackup(iface, enabled) {
  try {
    const u =
      "/cgi-bin/bsbf-client-web?" +
      (enabled ? "backup&" : "nobackup&") +
      encodeURIComponent(iface);
    const res = await fetch(u, { cache: "no-store" });
    const text = stripCgiStdoutPrefix(await res.text());
    const line = text.trim().split(/\r?\n/)[0] || "";
    if (!res.ok || line.startsWith("ERR\t")) {
      const detail = line.startsWith("ERR\t")
        ? line.slice(4).trim()
        : "HTTP " + res.status;
      appendRateLimitMsg("Backup (" + iface + "): " + detail);
      return false;
    }
    if (line && line !== "ok") {
      const snippet = text.trim().replace(/\s+/g, " ").slice(0, 280);
      appendRateLimitMsg("Backup (" + iface + "): unexpected " + (snippet || line));
      return false;
    }
    ifaceFlags[iface] = ifaceFlags[iface] || {};
    ifaceFlags[iface].forceBackupConfig = enabled;
    return true;
  } catch (e) {
    appendRateLimitMsg("Backup (" + iface + "): " + e.message);
    return false;
  }
}

// ============================================================================
// MPTCP / connectivity badges & interface membership reconciliation
// ============================================================================

/** Show/hide one interface's connectivity/MPTCP badges based on `ifaceFlags[iface]`. */
function applyMptcpBadgesForIface(iface) {
  const f = ifaceFlags[iface] || {};
  const bNo = document.getElementById("badge-no-conn-" + iface);
  const bNs = document.getElementById("badge-mptcp-ns-" + iface);
  const bSup = document.getElementById("badge-mptcp-sup-" + iface);
  if (bNo) bNo.hidden = !f.noConnectivity;
  if (bNs) bNs.hidden = !f.mptcpNotSupported;
  if (bSup) bSup.hidden = !f.mptcpSupported;
}

/**
 * Update `ifaceFlags[iface].{noConnectivity,mptcpNotSupported,mptcpSupported}`
 * for every interface in `list` from `flagsByIface` (keyed by iface name,
 * values like `{ no_connectivity, mptcp_not_supported, mptcp_supported }`),
 * and refresh their badges.
 */
function applyMptcpBadgesForIfaceList(list, flagsByIface) {
  const map = flagsByIface || {};
  for (let i = 0; i < list.length; i++) {
    const iface = list[i];
    const row = map[iface] || {};
    ifaceFlags[iface] = ifaceFlags[iface] || {};
    ifaceFlags[iface].noConnectivity = row.no_connectivity === true;
    ifaceFlags[iface].mptcpNotSupported = row.mptcp_not_supported === true;
    ifaceFlags[iface].mptcpSupported = row.mptcp_supported === true;
    applyMptcpBadgesForIface(iface);
  }
}

/**
 * Normalize the `interfaces` array from `parseInterfacesSnapshotText`/CGI into
 * `{ list: [ifaceNames...], flagsByIface: {[iface]: row} }`. Each entry may be
 * either a plain iface-name string (older format) or a
 * `{ iface, no_connectivity, ... }` row object.
 */
function normalizeInterfacesPayload(payload) {
  const list = [];
  if (!Array.isArray(payload)) return { list: list };
  const flagsByIface = {};

  for (let i = 0; i < payload.length; i++) {
    const row = payload[i];
    if (typeof row === "string") {
      list.push(row);
      continue;
    }
    if (!row || typeof row !== "object") continue;
    const name = typeof row.iface === "string" ? row.iface : null;
    if (!name) continue;
    list.push(name);
    flagsByIface[name] = row;
  }

  return { list: list, flagsByIface: flagsByIface };
}

/**
 * Normalize the full result of `parseInterfacesSnapshotText`/CGI into
 * `{ norm, subflowBackupSet }`:
 *  - Modern shape `{ interfaces, subflow_backup }` -> `norm` from
 *    `normalizeInterfacesPayload(interfaces)`, `subflowBackupSet` is a `Set`
 *    of the `subflow_backup` names.
 *  - Legacy shape: plain array `[...]` of interfaces -> `subflowBackupSet`
 *    is `null` (caller must fetch it separately via `fetchBondingBackupSet`).
 *  - Anything else -> empty interface list, `subflowBackupSet: null`.
 */
function normalizeInterfacesResult(raw) {
  if (
    raw != null &&
    typeof raw === "object" &&
    !Array.isArray(raw) &&
    Array.isArray(raw.interfaces)
  ) {
    const norm = normalizeInterfacesPayload(raw.interfaces);
    const subflowBackupSet = new Set();
    if (Array.isArray(raw.subflow_backup)) {
      for (let i = 0; i < raw.subflow_backup.length; i++) {
        if (typeof raw.subflow_backup[i] !== "string") continue;
        const s = raw.subflow_backup[i].trim().replace(/\r/g, "");
        if (s) subflowBackupSet.add(s);
      }
    }
    return { norm: norm, subflowBackupSet: subflowBackupSet };
  }
  if (Array.isArray(raw)) {
    return {
      norm: normalizeInterfacesPayload(raw),
      subflowBackupSet: null,
    };
  }
  return {
    norm: normalizeInterfacesPayload([]),
    subflowBackupSet: null,
  };
}

/** Remove an interface's card and all of its per-interface state (e.g. when it drops out of the route table). */
function removeIfaceFromUI(iface) {
  const el = document.getElementById("card-" + iface);
  if (el) el.remove();
  delete ifaceData[iface];
  delete tcInfo[iface];
  delete sortCap[iface];
  delete origIfaceIndex[iface];
  delete ifaceFlags[iface];
}

/**
 * Set up a newly-discovered interface: fetch its current tc caps and backup
 * membership, initialize its stats/state, create and append its card, and
 * wire up its controls. Does not add it to `ifaceOrder` — the caller does that.
 */
async function addIfaceToUI(iface, container) {
  const row = await fetchTcClass(iface);
  applyTcRowToCapState(iface, row);
  origIfaceIndex[iface] = nextOrigIfaceIndex++;
  initIfaceData(iface);
  const bondingSet = await fetchBondingBackupSet();
  ifaceFlags[iface] = ifaceFlags[iface] || {};
  ifaceFlags[iface].forceBackupConfig = bondingSet.has(String(iface).trim());
  container.appendChild(createCard(iface));
  bindSpeedControls(iface);
}

/**
 * Reconcile the displayed cards with the current interface list from a parsed
 * `?poll` payload: remove cards for interfaces that disappeared, add cards
 * for newly-discovered ones (via `addIfaceToUI`), update MPTCP/connectivity
 * badges for all of them, and re-sort + re-animate if the set of interfaces
 * changed. Skipped entirely while a card is being dragged, so an in-progress
 * drag doesn't get its DOM rearranged out from under the pointer.
 */
async function applyInterfaceMembershipFromPayload(payload) {
  if (panelDragActive) return;

  const { norm } = normalizeInterfacesResult(payload);
  const newList = norm.list;
  applyMptcpBadgesForIfaceList(newList, norm.flagsByIface);

  const statusEl = document.getElementById("status");
  const container = document.getElementById("interfaces");
  if (!statusEl || !container) return;

  const newSet = new Set(newList);
  let changed = false;

  for (const iface of ifaceOrder.slice()) {
    if (!newSet.has(iface)) {
      removeIfaceFromUI(iface);
      changed = true;
    }
  }

  ifaceOrder = ifaceOrder.filter(function (n) {
    return newSet.has(n);
  });

  for (let i = 0; i < newList.length; i++) {
    const iface = newList[i];
    if (document.getElementById("card-" + iface)) continue;
    await addIfaceToUI(iface, container);
    if (ifaceOrder.indexOf(iface) < 0) ifaceOrder.push(iface);
    changed = true;
  }

  if (newList.length === 0) {
    ifaceOrder.length = 0;
    statusEl.textContent = "No interfaces found.";
    return;
  }

  statusEl.textContent = "";

  if (changed) {
    ifaceOrder.sort(compareIfacesForSort);
    animateReorderCards(container, ifaceOrder.slice());
  }
}

// ============================================================================
// Throughput stats & rate-limit application
// ============================================================================

/**
 * Fetch raw RX/TX byte counters for `ifaces` from a poll snapshot (or fetch a
 * fresh one). Returns `{ [iface]: { rx_bytes, tx_bytes } }` (string counters
 * — see `applyStatsFromThroughputMap` for the rate calculation).
 */
async function fetchStatsBatch(ifaces, pollTextOpt) {
  if (!ifaces || ifaces.length === 0) return {};
  const text =
    pollTextOpt != null
      ? pollTextOpt
      : lastPollSnapshotText != null
        ? lastPollSnapshotText
        : await httpFetchPollTextOnce();
  const serverIfaces = parseIfaceListBlock(
    slicePollSection(text, "---interfaces---")
  );
  const byName = parseThroughputMapFromPoll(text, serverIfaces);
  const out = Object.create(null);
  for (let i = 0; i < ifaces.length; i++) {
    const n = ifaces[i];
    if (byName[n]) out[n] = byName[n];
  }
  return out;
}

/**
 * Per interface: **Set download/upload** checkboxes + Mbps → `bsbf-rate-limiting`.
 * Per-direction **Unlimited** → `0` for that direction; both unlimited → `0 0`.
 * If neither direction is unlimited but both Mbps round to 0, use 1/1.
 * Called after reorder and cap changes (Apply / drag). Not called when route
 * membership changes alone — new interfaces are shown without re-running rate limiting.
 * @param {string[]|undefined} limitApplyToIfaces  If set and non-empty, only those interfaces are pushed to the CGI; otherwise every interface in `ifaceOrder`.
 */
async function applyRateLimits(limitApplyToIfaces) {
  const msgEl = document.getElementById("rate-limit-msg");
  let names;
  if (limitApplyToIfaces != null && limitApplyToIfaces.length > 0) {
    names = limitApplyToIfaces.filter(function (n) {
      return ifaceOrder.indexOf(n) !== -1;
    });
  } else {
    names = ifaceOrder;
  }
  if (names.length === 0) {
    if (msgEl) msgEl.hidden = true;
    return;
  }

  const errs = [];

  for (let i = 0; i < names.length; i++) {
    const name = names[i];
    let dlStr;
    let ulStr;

    // Encode this interface's cap as the two integers the `limit` CGI expects:
    // both-unlimited -> "0 0"; otherwise each direction is its rounded Mbps,
    // bumped up to 1 if it would otherwise round down to 0 (0 would mean
    // "unlimited" for that direction, which we don't want here).
    const cap = sortCap[name];
    const dlUnl = !!(cap && cap.dlUnl);
    const ulUnl = !!(cap && cap.ulUnl);
    if (dlUnl && ulUnl) {
      dlStr = "0";
      ulStr = "0";
    } else {
      let dlRounded = dlUnl ? 0 : Math.round(cap ? cap.dlMbps : 0);
      let ulRounded = ulUnl ? 0 : Math.round(cap ? cap.ulMbps : 0);
      if (!dlUnl && dlRounded === 0) dlRounded = 1;
      if (!ulUnl && ulRounded === 0) ulRounded = 1;
      dlStr = String(Math.max(0, dlRounded));
      ulStr = String(Math.max(0, ulRounded));
    }

    try {
      const u =
        "/cgi-bin/bsbf-client-web?" +
        ["limit", name, dlStr, ulStr].map(encodeURIComponent).join("&");
      const res = await fetch(u, { cache: "no-store" });
      const text = stripCgiStdoutPrefix(await res.text());
      const line = text.trim().split(/\r?\n/)[0] || "";
      if (!res.ok || line.startsWith("ERR\t")) {
        errs.push(
          name +
            ": " +
            (line.startsWith("ERR\t")
              ? line.slice(4).trim()
              : "HTTP " + res.status)
        );
        continue;
      }
      if (line && line !== "ok") {
        errs.push(name + ": " + (line || "unexpected response"));
      }
    } catch (e) {
      errs.push(name + ": " + e.message);
    }
  }

  if (!msgEl) return;
  if (errs.length) {
    msgEl.hidden = false;
    msgEl.className = "rate-limit-msg err";
    msgEl.textContent = "Rate limits failed: " + errs.join("; ");
  }
}

/** Create the empty `ifaceData[iface]` history entry used to track RX/TX rates and peaks. */
function initIfaceData(iface) {
  ifaceData[iface] = {
    prevTx: null,
    prevRx: null,
    peakTx: 0,
    peakRx: 0,
    txRates: new Array(HISTORY_LEN).fill(-1),
    rxRates: new Array(HISTORY_LEN).fill(-1),
  };
}

/** Drop the oldest sample and append `val`, keeping `arr` at its fixed `HISTORY_LEN` size. */
function pushRate(arr, val) {
  arr.shift();
  arr.push(val);
}

/** Refresh graphs and rate labels from a throughput map (same keys as `ifaceOrder`). */
function applyStatsFromThroughputMap(byIface, ifaces) {
  for (let i = 0; i < ifaces.length; i++) {
    const iface = ifaces[i];
    const data = ifaceData[iface];
    if (!data) continue;

    const row = byIface[iface];
    if (!row) {
      pushRate(data.txRates, -1);
      pushRate(data.rxRates, -1);
      continue;
    }

    const { tx_bytes, rx_bytes } = row;
    const tx = parseInt(tx_bytes, 10);
    const rx = parseInt(rx_bytes, 10);

    if (data.prevTx !== null) {
      const txRate = tx - data.prevTx;
      const rxRate = rx - data.prevRx;
      pushRate(data.txRates, txRate >= 0 ? txRate : 0);
      pushRate(data.rxRates, rxRate >= 0 ? rxRate : 0);
      if (txRate > data.peakTx) data.peakTx = txRate;
      if (rxRate > data.peakRx) data.peakRx = rxRate;
    } else {
      pushRate(data.txRates, -1);
      pushRate(data.rxRates, -1);
    }

    data.prevTx = tx;
    data.prevRx = rx;

    const lastTx = data.txRates[HISTORY_LEN - 1];
    const lastRx = data.rxRates[HISTORY_LEN - 1];
    const rateEl = document.getElementById("rate-" + iface);
    if (rateEl) {
      rateEl.innerHTML =
        `<span class="rx">RX ${formatRate(lastRx)}</span>` +
        ` <span class="peak">(peak ${formatRate(data.peakRx)})</span>` +
        ` / ` +
        `<span class="tx">TX ${formatRate(lastTx)}</span>` +
        ` <span class="peak">(peak ${formatRate(data.peakTx)})</span>`;
    }

    drawGraph(iface);
  }
}

// ============================================================================
// Core poll hook & app startup
// ============================================================================

/**
 * The main per-poll handler, registered with `registerClientPollHook` so it
 * runs every `POLL_INTERVAL` (see `runClientPollCycle`) with the latest
 * snapshot text. Responsibilities, in order:
 *
 * 1. Bail out early on an `ERR\t...` snapshot or one that fails to parse.
 * 2. While in "chart-only mode" (`chartOnlyUntil`, set right after the user
 *    edits something) skip interface add/remove/reorder and just refresh the
 *    MPTCP badges, to avoid the UI jumping under the user's cursor. Otherwise
 *    run the full membership reconciliation (`applyInterfaceMembershipFromPayload`).
 * 3. Recompute RX/TX rates and redraw graphs for every interface currently
 *    shown (`applyStatsFromThroughputMap`).
 * 4. Outside chart-only mode, re-sync the cap UI from the device's actual
 *    `tc` settings (`applyTcClassSettingsFromPollText`), in case caps were
 *    changed by something other than this UI.
 */
async function coreClientPollHook(text) {
  if (text.trimStart().startsWith("ERR\t")) return;
  let payload;
  try {
    payload = parseInterfacesSnapshotText(text);
  } catch (_) {
    return;
  }
  const chartOnly = Date.now() < chartOnlyUntil;

  if (!chartOnly) {
    await applyInterfaceMembershipFromPayload(payload);
  } else {
    const { norm } = normalizeInterfacesResult(payload);
    applyMptcpBadgesForIfaceList(norm.list, norm.flagsByIface);
  }

  const serverOrder = parseIfaceListBlock(
    slicePollSection(text, "---interfaces---")
  );
  const fullStats = parseThroughputMapFromPoll(text, serverOrder);
  const byIface = Object.create(null);
  for (let i = 0; i < ifaceOrder.length; i++) {
    const n = ifaceOrder[i];
    if (fullStats[n]) byIface[n] = fullStats[n];
  }
  applyStatsFromThroughputMap(byIface, ifaceOrder);

  if (!chartOnly) {
    await applyTcClassSettingsFromPollText(text);
  }
}

registerClientPollHook(coreClientPollHook);

/**
 * Entry point, called once from the bootstrap IIFE at the bottom of this file.
 * Performs the one-time startup sequence:
 *
 * 1. Discover interfaces (`fetchInterfaces`) — show an error and stop if that fails.
 * 2. Apply MPTCP badges and bail with "No interfaces found." if the list is empty.
 * 3. Record each interface's original discovery order in `origIfaceIndex`
 *    (used as a tiebreaker by `compareIfacesForSort`).
 * 4. Fetch current `tc` caps for all interfaces in one batch and seed `sortCap`
 *    via `applyTcRowToCapState` (defaulting to fully-unlimited if missing).
 * 5. Build `ifaceOrder` and sort it (download cap descending, then original order).
 * 6. Determine the bonding-backup set (from the snapshot if available, else a
 *    dedicated fetch) and create a card + speed controls for each interface.
 */
async function main() {
  const statusEl = document.getElementById("status");
  const container = document.getElementById("interfaces");

  let payload;
  try {
    payload = await fetchInterfaces();
  } catch (e) {
    statusEl.innerHTML = "";
    container.innerHTML =
      `<div class="error-msg">Failed to discover interfaces: ${e.message}</div>`;
    return;
  }

  const { norm, subflowBackupSet } = normalizeInterfacesResult(payload);
  const ifaces = norm.list;
  applyMptcpBadgesForIfaceList(ifaces, norm.flagsByIface);

  if (ifaces.length === 0) {
    statusEl.textContent = "No interfaces found.";
    ifaceOrder = [];
    return;
  }

  statusEl.textContent = "";

  for (let i = 0; i < ifaces.length; i++) {
    origIfaceIndex[ifaces[i]] = i;
  }
  nextOrigIfaceIndex = ifaces.length;

  const tcBatch = await fetchTcClassBatch(ifaces);
  const tcEmpty = normalizeTcJson({ unlimited: true });
  for (let i = 0; i < ifaces.length; i++) {
    const iface = ifaces[i];
    applyTcRowToCapState(
      iface,
      tcBatch[iface] != null ? tcBatch[iface] : tcEmpty
    );
  }

  ifaceOrder = ifaces.slice();
  ifaceOrder.sort(compareIfacesForSort);

  let bondingSet =
    subflowBackupSet === null ? await fetchBondingBackupSet() : subflowBackupSet;
  for (const iface of ifaceOrder) {
    initIfaceData(iface);
    ifaceFlags[iface] = ifaceFlags[iface] || {};
    ifaceFlags[iface].forceBackupConfig = bondingSet.has(String(iface).trim());
    container.appendChild(createCard(iface));
    bindSpeedControls(iface);
  }
}

// ============================================================================
// Speed test embed
// ============================================================================

/**
 * Wire up the collapsible OpenSpeedTest iframe panel. The iframe is only
 * given a `src` while the panel is open (and reset to `about:blank` when
 * closed) so the embedded test doesn't keep running in the background.
 * The iframe is hidden (opacity 0) until it fires `load` or 3.5s elapse,
 * whichever comes first, to avoid showing a blank/white flash while it loads.
 */
function initSpeedTestEmbed() {
  const OST_URL = "https://openspeedtest.com/speedtest";
  const iframe = document.getElementById("ost-iframe");
  const btn = document.getElementById("ost-toggle");
  const panel = document.getElementById("ost-panel");
  const section = document.getElementById("ost-section");
  if (!iframe || !btn || !panel) return;

  let revealTimer = null;

  function clearRevealTimer() {
    if (revealTimer != null) {
      window.clearTimeout(revealTimer);
      revealTimer = null;
    }
  }

  function setOpen(open) {
    if (open) {
      clearRevealTimer();
      panel.hidden = false;
      iframe.style.opacity = "0";
      iframe.style.transition = "opacity 0.2s ease";
      const show = function () {
        clearRevealTimer();
        iframe.style.opacity = "1";
      };
      iframe.addEventListener("load", show, { once: true });
      revealTimer = window.setTimeout(show, 3500);
      iframe.src = OST_URL + "?_=" + Date.now();
      btn.textContent = "Hide";
      btn.title = "Hide the speed test";
      if (section) {
        section.classList.remove("speedtest-collapsed");
        section.classList.add("speedtest-open");
      }
    } else {
      clearRevealTimer();
      panel.hidden = true;
      iframe.src = "about:blank";
      iframe.style.opacity = "";
      iframe.style.transition = "";
      btn.textContent = "Show";
      btn.title = "Open the speed test embed";
      if (section) {
        section.classList.add("speedtest-collapsed");
        section.classList.remove("speedtest-open");
      }
    }
  }

  btn.addEventListener("click", function () {
    if (panel.hidden) setOpen(true);
    else setOpen(false);
  });
}

// ============================================================================
// Theme toggle (light/dark)
// ============================================================================

/** From `<html data-theme>`: default is dark (no attribute); `light` selects the light palette. */
function getThemeMode() {
  const t = document.documentElement.getAttribute("data-theme");
  if (t === "light") return "light";
  return "dark";
}

/** Set or clear `data-theme="light"` on `<html>`; `dark` is the implicit default (no attribute). */
function applyThemeMode(mode) {
  const root = document.documentElement;
  if (mode === "light") root.setAttribute("data-theme", "light");
  else root.removeAttribute("data-theme");
}

/**
 * Optional URL override: `?theme=light` or `?theme=dark`.
 * Runs once at startup; invalid values are ignored. Case-insensitive.
 */
function applyThemeFromQuery() {
  let raw;
  try {
    raw = new URLSearchParams(window.location.search).get("theme");
  } catch (_) {
    return;
  }
  if (raw == null) return;
  const v = String(raw).trim().toLowerCase();
  if (v === "light" || v === "dark") applyThemeMode(v);
}

/** Redraw every visible graph, e.g. after a theme change (colors are read from CSS). */
function redrawAllGraphs() {
  for (let i = 0; i < ifaceOrder.length; i++) {
    drawGraph(ifaceOrder[i]);
  }
}

/** Wire up the theme-toggle button: label reflects the current mode, click flips it and redraws graphs. */
function initThemeToggle() {
  const btn = document.getElementById("theme-toggle");
  if (!btn) return;

  function updateLabel() {
    const mode = getThemeMode();
    if (mode === "light") {
      btn.textContent = "Theme: Light";
      btn.title = "Using light theme. Click for dark.";
    } else {
      btn.textContent = "Theme: Dark";
      btn.title = "Using dark theme. Click for light.";
    }
  }

  btn.addEventListener("click", function () {
    const cur = getThemeMode();
    applyThemeMode(cur === "dark" ? "light" : "dark");
    updateLabel();
    redrawAllGraphs();
  });

  updateLabel();
}

// ============================================================================
// Bonding toggle (enable/disable MPTCP bonding)
// ============================================================================

/**
 * Current bonding state, preferring the most recent poll snapshot (avoids an
 * extra request) and falling back to a fresh poll if none is cached yet.
 * Throws if the snapshot doesn't contain a recognized `---bonding---` line.
 */
async function fetchBondingStatus() {
  const text =
    lastPollSnapshotText != null
      ? lastPollSnapshotText
      : await httpFetchPollTextOnce();
  const line = parseBondingStatusLineFromPoll(text);
  if (line === "enabled") return { enabled: true };
  if (line === "disabled") return { enabled: false };
  if (text.trimStart().startsWith("ERR\t"))
    throw new Error(text.trim().split(/\t/)[1] || "bonding status error");
  throw new Error(line || "bad bonding status");
}

/** Call the `enable`/`disable` CGI action; throws on `ERR\t...` or non-OK HTTP status. */
async function setBondingEnabled(enabled) {
  const q = enabled ? "enable" : "disable";
  const res = await fetch("/cgi-bin/bsbf-client-web?" + q, {
    cache: "no-store",
  });
  const text = stripCgiStdoutPrefix(await res.text());
  const line = text.trim().split(/\r?\n/)[0] || "";
  if (line.startsWith("ERR\t"))
    throw new Error(line.slice(4).trim() || "bonding command failed");
  if (!res.ok) throw new Error("HTTP " + res.status);
  return { ok: true };
}

/** Show/clear a busy spinner state on the bonding button while a toggle request is in flight. */
function setBondingButtonLoading(btn, loading) {
  if (!btn) return;
  const label = btn.querySelector(".btn-bonding-label");
  if (loading) {
    btn.disabled = true;
    btn.classList.add("is-loading");
    btn.setAttribute("aria-busy", "true");
    if (label) label.textContent = "Bonding:";
    btn.title = "Changing bonding… waiting for status";
  } else {
    btn.classList.remove("is-loading");
    btn.removeAttribute("aria-busy");
  }
}

/** Update the bonding button's label/title to reflect `enabled`. No-op while a toggle is loading. */
function renderBondingButton(btn, enabled) {
  if (!btn) return;
  if (btn.classList.contains("is-loading")) return;
  const label = btn.querySelector(".btn-bonding-label");
  btn.disabled = false;
  if (label) label.textContent = enabled ? "Bonding: Enabled" : "Bonding: Disabled";
  else btn.textContent = enabled ? "Bonding: Enabled" : "Bonding: Disabled";
  btn.title = enabled ? "Click to disable bonding" : "Click to enable bonding";
}

/**
 * Wire up the bonding toggle button and its poll hook:
 * - Click handler calls `setBondingEnabled` and re-renders the button,
 *   reporting any error via `#rate-limit-msg`. Enters chart-only mode
 *   afterwards so the membership/cap reconciliation doesn't run immediately
 *   on top of the user's change.
 * - `bondingClientPollHook` keeps the button's label in sync with the
 *   device's actual bonding state (unless a toggle is in flight) and keeps
 *   each card's "Backup" checkbox in sync with the bonding-backup set.
 * - On init, fetches the current bonding status once to set the initial label.
 */
function initBondingToggle() {
  const btn = document.getElementById("bonding-toggle");
  const msgEl = document.getElementById("rate-limit-msg");
  if (!btn) return;

  let cur = null;
  let bondingToggleBusy = false;
  let bondingPollInFlight = false;

  btn.addEventListener("click", async function () {
    if (cur === null) return;
    if (bondingToggleBusy) return;
    bondingToggleBusy = true;
    setBondingButtonLoading(btn, true);
    if (msgEl) msgEl.hidden = true;
    const next = !cur;
    try {
      await setBondingEnabled(next);
      cur = next;
      setBondingButtonLoading(btn, false);
      renderBondingButton(btn, cur);
    } catch (e) {
      const extra = "Bonding: " + (e.message || String(e));
      if (msgEl) {
        msgEl.hidden = false;
        msgEl.className = "rate-limit-msg err";
        msgEl.textContent = extra;
      }
      setBondingButtonLoading(btn, false);
      renderBondingButton(btn, cur);
    } finally {
      bondingToggleBusy = false;
      enterChartOnlyMode();
    }
  });

  registerClientPollHook(async function bondingClientPollHook(text) {
    if (Date.now() < chartOnlyUntil) return;
    if (bondingPollInFlight) return;
    bondingPollInFlight = true;
    try {
      const line = parseBondingStatusLineFromPoll(text);
      if (
        !bondingToggleBusy &&
        (line === "enabled" || line === "disabled")
      ) {
        cur = line === "enabled";
        renderBondingButton(btn, cur);
      }
      if (ifaceOrder.length > 0) {
        syncBackupUIFromBondingSet(subflowBackupSetFromPollText(text));
      }
    } catch (_) {
      /* ignore */
    } finally {
      bondingPollInFlight = false;
    }
  });

  (async function () {
    try {
      setBondingButtonLoading(btn, true);
      const res = await fetchBondingStatus();
      cur = res.enabled;
      setBondingButtonLoading(btn, false);
      renderBondingButton(btn, cur);
    } catch (_) {
      cur = false;
      setBondingButtonLoading(btn, false);
      renderBondingButton(btn, cur);
      btn.title =
        "Could not read bonding status; label may be wrong until refresh. Toggle still runs enable/disable.";
    }
  })();
}

// ============================================================================
// Bootstrap
// ============================================================================

// Apply ?theme=... immediately, before any UI is built, to avoid a flash of
// the wrong theme.
applyThemeFromQuery();

void (async function bootstrap() {
  initSpeedTestEmbed();
  initThemeToggle();
  // Build the initial cards/state, then start the recurring poll loop
  // (coreClientPollHook and bondingClientPollHook are invoked from there).
  await main();
  initBondingToggle();
  void runClientPollCycle();
  setInterval(function () {
    void runClientPollCycle();
  }, POLL_INTERVAL);
})();
