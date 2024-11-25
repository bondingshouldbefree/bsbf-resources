"use strict";

const HISTORY_LEN = 60;
const POLL_INTERVAL = 1000;
/** Bonding button + backup checkbox sync from server (same cadence as stats). */
const STATUS_POLL_MS = POLL_INTERVAL;
const REORDER_ANIM_MS = 340;
const REORDER_ANIM_EASING = "cubic-bezier(0.22, 1, 0.36, 1)";

const ifaceData = {};
const ifaceFlags = {};

/** Order of interfaces (top to bottom); updated when user reorders cards or caps change. */
let ifaceOrder = [];
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

function formatRate(bytesPerSec) {
  if (bytesPerSec < 0) return "---";
  const bitsPerSec = bytesPerSec * 8;
  if (bitsPerSec >= 1e9) return (bitsPerSec / 1e9).toFixed(2) + " Gbps";
  if (bitsPerSec >= 1e6) return (bitsPerSec / 1e6).toFixed(2) + " Mbps";
  if (bitsPerSec >= 1e3) return (bitsPerSec / 1e3).toFixed(2) + " Kbps";
  return bitsPerSec.toFixed(0) + " bps";
}

function formatMbpsShort(mbps) {
  if (!isFinite(mbps)) return "0";
  const r = Math.round(mbps * 1000) / 1000;
  if (Math.abs(r - Math.round(r)) < 1e-9) return String(Math.round(r));
  const s = r.toFixed(3).replace(/\.?0+$/, "");
  return s;
}

function bytesPerSecPeakToMbps(bps) {
  return (bps * 8) / 1e6;
}

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

/** Read cap fields from a card (same rules as bindSpeedControls readCapFromInputs). */
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

function capsEqual(a, b) {
  if (!a || !b) return false;
  if (a.dlUnl !== b.dlUnl || a.ulUnl !== b.ulUnl) return false;
  const eps = 1e-6;
  if (!a.dlUnl && Math.abs(a.dlMbps - b.dlMbps) > eps) return false;
  if (!a.ulUnl && Math.abs(a.ulMbps - b.ulMbps) > eps) return false;
  return true;
}

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

function updateCapApplyButtonState(iface) {
  const card = document.getElementById("card-" + iface);
  if (!card) return;
  const btn = card.querySelector(".btn-cap-apply");
  if (!btn) return;
  btn.disabled = !ifaceHasPendingChanges(iface);
}

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
      if (dlRounded === 0 && ulRounded === 0) {
        dlRounded = 1;
        ulRounded = 1;
      }
      sortCap[name] = {
        dlUnl: false,
        ulUnl: false,
        dlMbps: dlRounded,
        ulMbps: ulRounded,
      };
    }
  }
}

function syncAllCapUIFromSortCap() {
  for (let i = 0; i < ifaceOrder.length; i++) {
    syncCapUIFromSortCap(ifaceOrder[i]);
  }
}

/** Max Mbps from one `tc class show` block (same rules as the former CGI awk). */
function parseTcClassMaxMbps(block) {
  const lines = String(block || "").split(/\r?\n/);
  let max = 0;
  let found = false;
  function parseTok(tok) {
    if (!/bit$/.test(tok)) return NaN;
    let s = tok.replace(/bit$/, "");
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
    if (u === "") return v / 1000000;
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

function tcBlocksToRow(dlText, ulText) {
  const empty = {
    unlimited: true,
    dl_unlimited: true,
    ul_unlimited: true,
    dl_mbps: 0,
    ul_mbps: 0,
  };
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

/** Default routes with explicit metric + dev — same iface order rules as before (CGI awk). */
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

function parseIfaceLineSet(block) {
  const set = new Set();
  const lines = String(block || "").split(/\r?\n/);
  for (let i = 0; i < lines.length; i++) {
    let line = lines[i].trim().replace(/\r/g, "");
    if (line === "" || line.charAt(0) === "#") continue;
    if (!/^[a-zA-Z0-9._-]+$/.test(line)) continue;
    set.add(line);
  }
  return set;
}

function parseInterfacesSnapshotText(text) {
  const raw = String(text || "").replace(/\r/g, "");
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
  const backupLines = [];
  const backupRaw = parts["---backup---"] || "";
  const blines = backupRaw.split(/\r?\n/);
  for (let i = 0; i < blines.length; i++) {
    let line = blines[i].trim();
    if (line === "" || line.charAt(0) === "#") continue;
    if (!/^[a-zA-Z0-9._-]+$/.test(line)) continue;
    backupLines.push(line);
  }
  const flagsByIface = {};
  for (let i = 0; i < list.length; i++) {
    const iface = list[i];
    flagsByIface[iface] = {
      iface: iface,
      no_connectivity: noConn.has(iface),
      mptcp_not_supported: ns.has(iface),
      mptcp_supported: sup.has(iface),
    };
  }
  const interfaces = list.map(function (name) {
    return flagsByIface[name] || { iface: name };
  });
  return {
    interfaces: interfaces,
    subflow_backup: backupLines,
  };
}

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

async function fetchTcClassBatch(ifaces) {
  const empty = normalizeTcJson({ unlimited: true });
  if (!ifaces || ifaces.length === 0) return {};
  const res = await fetch(
    "/cgi-bin/bsbf-client-web?op=tc-class&ifaces=" +
      encodeURIComponent(ifaces.join(","))
  );
  if (!res.ok) {
    const o = {};
    for (let i = 0; i < ifaces.length; i++) o[ifaces[i]] = empty;
    return o;
  }
  let text;
  try {
    text = await res.text();
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
  const parsed = parseTcClassBatchText(text);
  const o = {};
  for (let i = 0; i < ifaces.length; i++) {
    const n = ifaces[i];
    o[n] = parsed[n] != null ? parsed[n] : empty;
  }
  return o;
}

async function fetchTcClass(iface) {
  const m = await fetchTcClassBatch([iface]);
  const empty = normalizeTcJson({ unlimited: true });
  return m[iface] != null ? m[iface] : empty;
}

function applyTcRowToCapState(iface, row) {
  tcInfo[iface] = row;
  sortCap[iface] = {
    dlUnl: !!row.dl_unlimited,
    ulUnl: !!row.ul_unlimited,
    dlMbps: row.dl_mbps || 0,
    ulMbps: row.ul_mbps || 0,
  };
}

let tcClassPollInFlight = false;

/** Refresh Download/Upload limits from `tc class` when the user has no pending edits on that card. */
async function pollTcClassSettingsOnce() {
  if (tcClassPollInFlight || ifaceOrder.length === 0) return;
  tcClassPollInFlight = true;
  try {
    const targets = ifaceOrder.filter(function (iface) {
      return !ifaceHasPendingChanges(iface);
    });
    if (targets.length === 0) return;
    const batch = await fetchTcClassBatch(targets);
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

function fullUnlimitedCap(c) {
  return !!(c && c.dlUnl && c.ulUnl);
}

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

function prefersReducedMotion() {
  return (
    typeof matchMedia === "function" &&
    matchMedia("(prefers-reduced-motion: reduce)").matches
  );
}

function reorderDomInOrder(container, namesInOrder) {
  if (!container) return;
  for (let k = 0; k < namesInOrder.length; k++) {
    const name = namesInOrder[k];
    const el = document.getElementById("card-" + name);
    if (el) container.appendChild(el);
  }
}

function clearDragFloatStyles(card) {
  if (!card) return;
  card.classList.remove("iface-card-floating");
  card.style.left = "";
  card.style.top = "";
  card.style.width = "";
  card.style.zIndex = "";
  card.style.position = "";
  card.style.transform = "";
}

/**
 * FLIP reorder for all cards: record screen positions (floating dragged card uses
 * its on-screen drop rect as “first”), reorder DOM, drop dragged card into flow,
 * then invert + play transform for every card that moved.
 * @param {object} [dragContext]  If the order change came from a pointer drag, pass `{ card }` still floating.
 */
function animateReorderCards(container, namesInOrder, dragContext) {
  if (!container || namesInOrder.length === 0) return;

  const cards = [];
  for (let i = 0; i < namesInOrder.length; i++) {
    const el = document.getElementById("card-" + namesInOrder[i]);
    if (el) cards.push(el);
  }
  if (cards.length === 0) return;

  if (prefersReducedMotion()) {
    reorderDomInOrder(container, namesInOrder);
    if (dragContext && dragContext.card) {
      clearDragFloatStyles(dragContext.card);
    }
    return;
  }

  const firstRects = new Map();
  for (let i = 0; i < cards.length; i++) {
    const el = cards[i];
    firstRects.set(el, el.getBoundingClientRect());
  }

  reorderDomInOrder(container, namesInOrder);

  if (dragContext && dragContext.card) {
    clearDragFloatStyles(dragContext.card);
  }

  void container.offsetHeight;

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

  if (animated.length === 0) return;

  void container.offsetHeight;

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
  }, REORDER_ANIM_MS + 60);
}

function resortBySortCap() {
  ifaceOrder.sort(compareIfacesForSort);
  const container = document.getElementById("interfaces");
  animateReorderCards(container, ifaceOrder.slice());
  return applyRateLimits();
}

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
    btnApply.addEventListener("click", function () {
      const next = readCapFromInputs();
      if (!next) {
        syncCapUIFromSortCap(iface);
        return;
      }
      sortCap[iface] = next;
      btnApply.disabled = true;
      btnApply.classList.add("is-loading");
      btnApply.setAttribute("aria-busy", "true");
      const cbFb = card.querySelector(".cap-force-backup");
      const wantBackup = !!(cbFb && cbFb.checked);
      resortBySortCap()
        .then(function () {
          return applyForceBackup(iface, wantBackup);
        })
        .finally(function () {
          btnApply.classList.remove("is-loading");
          btnApply.removeAttribute("aria-busy");
          updateCapApplyButtonState(iface);
        });
    });
  }

  updateCapApplyButtonState(iface);
  refreshIfaceWanIsp(iface);
}

/** Host runs `curl https://ipwho.is --interface iface`; on success only, show IP · ISP · CC (no loading text). */
async function refreshIfaceWanIsp(iface) {
  const el = document.getElementById("wan-" + iface);
  if (!el) return;
  el.textContent = "";
  el.hidden = true;
  try {
    const res = await fetch(
      "/cgi-bin/bsbf-client-web?op=iface-ip&iface=" + encodeURIComponent(iface)
    );
    const raw = await res.text();
    const nl = raw.indexOf("\n");
    const status = nl >= 0 ? raw.slice(0, nl).trim() : raw.trim();
    const body = nl >= 0 ? raw.slice(nl + 1) : "";
    if (!res.ok || status === "ERR" || status.startsWith("ERR\t")) {
      const msg =
        status.startsWith("ERR\t") ? status.slice(4).trim() : status;
      throw new Error(msg || "HTTP " + res.status);
    }
    if (status !== "OK") {
      throw new Error(status || "iface-ip failed");
    }
    let j;
    try {
      j = JSON.parse(body.trim());
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
    if (e.button !== 0) return;
    if (panelDragActive) {
      e.preventDefault();
      return;
    }
    e.preventDefault();

    const rect = card.getBoundingClientRect();
    const ph = document.createElement("div");
    ph.className = "iface-card-placeholder";
    ph.style.height = card.offsetHeight + "px";
    card.parentNode.insertBefore(ph, card);

    panelDragActive = true;
    const pointerId = e.pointerId;

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

    function setDragOver(over) {
      if (over === lastOver) return;
      if (lastOver) lastOver.classList.remove("drag-over");
      lastOver = over;
      if (lastOver) lastOver.classList.add("drag-over");
    }

    function onMove(ev) {
      if (ev.pointerId !== pointerId) return;
      card.style.left = ev.clientX - offsetX + "px";
      card.style.top = ev.clientY - offsetY + "px";
      setDragOver(targetCardAtPoint(ev.clientX, ev.clientY));
    }

    const moveTarget = useDocumentFallback ? document : handle;
    const endTarget = useDocumentFallback ? document : handle;

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

        ph.remove();

        if (targetIface && targetIface !== iface) {
          reorderInterfaces(iface, targetIface, { card: card });
        } else {
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
 * @param {object} [dragContext]  After a pointer drag, pass `{ card }` (still floating) so FLIP uses its drop position.
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
  applyTieredCapsFromTopPeaks();
  syncAllCapUIFromSortCap();
  applyRateLimits().catch(function () {});
}

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
  const maxVal = peakScale * 1.1;

  const stepX = w / (HISTORY_LEN - 1);

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

  ctx.fillStyle = gc.label;
  ctx.font = "10px monospace";
  ctx.textAlign = "right";
  ctx.fillText(formatRate(maxVal / 1.1), w - 4, 12);
  ctx.fillText(formatRate(maxVal / 1.1 / 2), w - 4, h / 2 + 4);
  ctx.fillText("0", w - 4, h - 4);
}

async function fetchInterfaces() {
  const res = await fetch(
    "/cgi-bin/bsbf-client-web?op=interfaces&_=" + encodeURIComponent(String(Date.now())),
    { cache: "no-store" }
  );
  if (!res.ok) throw new Error("Failed to fetch interfaces");
  const text = await res.text();
  return parseInterfacesSnapshotText(text);
}

/** Names from bsbf-mptcp-subflow-backup (one iface per line). */
async function fetchBondingBackupSet() {
  try {
    const res = await fetch(
      "/cgi-bin/bsbf-client-web?op=backup&_=" + encodeURIComponent(String(Date.now())),
      { cache: "no-store" }
    );
    if (!res.ok) return new Set();
    const text = await res.text();
    if (text.trimStart().startsWith("ERR\t")) return new Set();
    const set = new Set();
    const lines = text.split(/\r?\n/);
    for (let i = 0; i < lines.length; i++) {
      let line = lines[i].trim().replace(/\r/g, "");
      if (line === "" || line.charAt(0) === "#") continue;
      if (!/^[a-zA-Z0-9._-]+$/.test(line)) continue;
      set.add(line);
    }
    return set;
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

/** Updates /tmp/bsbf/subflow_backup via bsbf-force-backup. Updates ifaceFlags.forceBackupConfig on success. */
async function applyForceBackup(iface, enabled) {
  const msgEl = document.getElementById("rate-limit-msg");
  try {
    const u =
      "/cgi-bin/bsbf-client-web?op=force-backup&iface=" +
      encodeURIComponent(iface) +
      "&enabled=" +
      (enabled ? "1" : "0");
    const res = await fetch(u);
    const text = await res.text();
    const line = text.trim().split(/\r?\n/)[0] || "";
    if (!res.ok || line.startsWith("ERR\t")) {
      const detail = line.startsWith("ERR\t")
        ? line.slice(4).trim()
        : "HTTP " + res.status;
      const extra = "Backup (" + iface + "): " + detail;
      if (msgEl) {
        if (!msgEl.hidden && msgEl.textContent) {
          msgEl.textContent += "; " + extra;
        } else {
          msgEl.hidden = false;
          msgEl.className = "rate-limit-msg err";
          msgEl.textContent = extra;
        }
      }
      return false;
    }
    if (line !== "ok") {
      const snippet = text.trim().replace(/\s+/g, " ").slice(0, 280);
      const extra =
        "Backup (" + iface + "): unexpected " + (snippet || line);
      if (msgEl) {
        if (!msgEl.hidden && msgEl.textContent) {
          msgEl.textContent += "; " + extra;
        } else {
          msgEl.hidden = false;
          msgEl.className = "rate-limit-msg err";
          msgEl.textContent = extra;
        }
      }
      return false;
    }
    ifaceFlags[iface] = ifaceFlags[iface] || {};
    ifaceFlags[iface].forceBackupConfig = enabled;
    return true;
  } catch (e) {
    const extra = "Backup (" + iface + "): " + e.message;
    if (msgEl) {
      if (!msgEl.hidden && msgEl.textContent) {
        msgEl.textContent += "; " + extra;
      } else {
        msgEl.hidden = false;
        msgEl.className = "rate-limit-msg err";
        msgEl.textContent = extra;
      }
    }
    return false;
  }
}

function applyMptcpBadgesForIface(iface) {
  const f = ifaceFlags[iface] || {};
  const bNo = document.getElementById("badge-no-conn-" + iface);
  const bNs = document.getElementById("badge-mptcp-ns-" + iface);
  const bSup = document.getElementById("badge-mptcp-sup-" + iface);
  if (bNo) bNo.hidden = !f.noConnectivity;
  if (bNs) bNs.hidden = !f.mptcpNotSupported;
  if (bSup) bSup.hidden = !f.mptcpSupported;
}

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
 * CGI returns `{ interfaces, subflow_backup }` from plaintext snapshot parsing.
 * Legacy: plain array [...]. subflowBackupSet is null only for legacy (fetch `op=backup`).
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

function removeIfaceFromUI(iface) {
  const el = document.getElementById("card-" + iface);
  if (el) el.remove();
  delete ifaceData[iface];
  delete tcInfo[iface];
  delete sortCap[iface];
  delete origIfaceIndex[iface];
  delete ifaceFlags[iface];
}

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
 * Reconcile UI with routes (via /cgi-bin/bsbf-client-web?op=interfaces): drop removed ifaces, add new ones.
 * Skips while a card is being dragged so the DOM is not torn mid-gesture.
 */
async function syncInterfaceMembership() {
  if (panelDragActive) return;

  let payload;
  try {
    payload = await fetchInterfaces();
  } catch (_) {
    return;
  }

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
    if (ifaceData[iface]) continue;
    await addIfaceToUI(iface, container);
    ifaceOrder.push(iface);
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

async function fetchStatsBatch(ifaces) {
  if (!ifaces || ifaces.length === 0) return {};
  const res = await fetch(
    "/cgi-bin/bsbf-client-web?op=stats-batch&ifaces=" +
      encodeURIComponent(ifaces.join(","))
  );
  if (!res.ok) throw new Error("Failed to fetch stats (HTTP " + res.status + ")");
  const text = await res.text();
  const out = Object.create(null);
  const lines = text.split(/\r?\n/);
  for (let i = 0; i < lines.length; i++) {
    const line = lines[i].trim();
    if (!line) continue;
    const parts = line.split("\t");
    if (parts.length >= 3 && parts[1] !== "ERR") {
      out[parts[0]] = { tx_bytes: parts[1], rx_bytes: parts[2] };
    }
  }
  return out;
}

/**
 * Per interface: **Set download/upload** checkboxes + Mbps → `bsbf-rate-limiting`.
 * Per-direction **Unlimited** → `0` for that direction; both unlimited → `0 0`.
 * If neither direction is unlimited but both Mbps round to 0, use 1/1.
 * Called after reorder and cap changes (Apply / drag). Not called when route
 * membership changes alone — new interfaces are shown without re-running rate limiting.
 */
async function applyRateLimits() {
  const msgEl = document.getElementById("rate-limit-msg");
  if (ifaceOrder.length === 0) {
    if (msgEl) msgEl.hidden = true;
    return;
  }

  const errs = [];

  for (let i = 0; i < ifaceOrder.length; i++) {
    const name = ifaceOrder[i];
    let dlStr;
    let ulStr;

    const cap = sortCap[name];
    const dlUnl = !!(cap && cap.dlUnl);
    const ulUnl = !!(cap && cap.ulUnl);
    if (dlUnl && ulUnl) {
      dlStr = "0";
      ulStr = "0";
    } else {
      let dlRounded = dlUnl ? 0 : Math.round(cap ? cap.dlMbps : 0);
      let ulRounded = ulUnl ? 0 : Math.round(cap ? cap.ulMbps : 0);
      if (!dlUnl && !ulUnl && dlRounded === 0 && ulRounded === 0) {
        dlRounded = 1;
        ulRounded = 1;
      }
      dlStr = String(Math.max(0, dlRounded));
      ulStr = String(Math.max(0, ulRounded));
    }

    try {
      const u =
        "/cgi-bin/bsbf-client-web?op=priority&iface=" +
        encodeURIComponent(name) +
        "&dl=" +
        encodeURIComponent(dlStr) +
        "&ul=" +
        encodeURIComponent(ulStr);
      const res = await fetch(u);
      const text = await res.text();
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
      if (line !== "ok") {
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

function pushRate(arr, val) {
  arr.shift();
  arr.push(val);
}

async function pollOnce(ifaces) {
  let byIface;
  try {
    byIface = await fetchStatsBatch(ifaces);
  } catch (_) {
    byIface = {};
  }

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
    pollOnce(ifaceOrder);
    setInterval(function () {
      pollOnce(ifaceOrder);
    }, POLL_INTERVAL);
    setInterval(function () {
      syncInterfaceMembership().catch(function () {});
    }, POLL_INTERVAL);
    setInterval(function () {
      pollTcClassSettingsOnce().catch(function () {});
    }, POLL_INTERVAL);
    syncInterfaceMembership().catch(function () {});
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

  pollOnce(ifaceOrder);
  setInterval(() => pollOnce(ifaceOrder), POLL_INTERVAL);
  setInterval(function () {
    syncInterfaceMembership().catch(function () {});
  }, POLL_INTERVAL);
  setInterval(function () {
    pollTcClassSettingsOnce().catch(function () {});
  }, POLL_INTERVAL);
}

function initSpeedTestEmbed() {
  const OST_URL = "https://openspeedtest.com/speedtest";
  const iframe = document.getElementById("ost-iframe");
  const btn = document.getElementById("ost-toggle");
  const panel = document.getElementById("ost-panel");
  const section = document.getElementById("ost-section");
  if (!iframe || !btn || !panel) return;

  function setOpen(open) {
    if (open) {
      panel.hidden = false;
      iframe.src = OST_URL + "?_=" + Date.now();
      btn.textContent = "Hide";
      btn.title = "Hide the speed test";
      if (section) {
        section.classList.remove("speedtest-collapsed");
        section.classList.add("speedtest-open");
      }
    } else {
      panel.hidden = true;
      iframe.src = "about:blank";
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

/** From `<html data-theme>`: missing or unknown → system (auto); `light` / `dark` forces that palette. */
function getThemeMode() {
  const t = document.documentElement.getAttribute("data-theme");
  if (t === "light" || t === "dark") return t;
  return "auto";
}

function applyThemeMode(mode) {
  const root = document.documentElement;
  if (mode === "auto") root.removeAttribute("data-theme");
  else root.setAttribute("data-theme", mode);
}

/**
 * Optional URL override: `?theme=light`, `?theme=dark`, or `?theme=auto`.
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
  else if (v === "auto" || v === "") applyThemeMode("auto");
}

function redrawAllGraphs() {
  for (let i = 0; i < ifaceOrder.length; i++) {
    drawGraph(ifaceOrder[i]);
  }
}

function initThemeToggle() {
  const btn = document.getElementById("theme-toggle");
  if (!btn) return;

  function updateLabel() {
    const mode = getThemeMode();
    if (mode === "light") {
      btn.textContent = "Theme: Light";
      btn.title = "Using light theme. Click for dark, then system.";
    } else if (mode === "dark") {
      btn.textContent = "Theme: Dark";
      btn.title = "Using dark theme. Click for system (auto), then light.";
    } else {
      btn.textContent = "Theme: Auto";
      btn.title = "Following system light/dark. Click for light, then dark.";
    }
  }

  btn.addEventListener("click", function () {
    const cur = getThemeMode();
    const next =
      cur === "auto" ? "light" : cur === "light" ? "dark" : "auto";
    applyThemeMode(next);
    updateLabel();
    redrawAllGraphs();
  });

  updateLabel();

  if (window.matchMedia) {
    const mq = window.matchMedia("(prefers-color-scheme: light)");
    function onSchemeChange() {
      if (getThemeMode() === "auto") redrawAllGraphs();
    }
    if (mq.addEventListener) mq.addEventListener("change", onSchemeChange);
    else if (mq.addListener) mq.addListener(onSchemeChange);
  }
}

async function fetchBondingStatus() {
  const res = await fetch("/cgi-bin/bsbf-client-web?op=bonding-status&_=" + Date.now(), {
    cache: "no-store",
  });
  const text = await res.text();
  const line = text.trim().split(/\r?\n/)[0] || "";
  if (line === "enabled") return { enabled: true };
  if (line === "disabled") return { enabled: false };
  if (!res.ok || line.startsWith("ERR\t"))
    throw new Error(
      line.startsWith("ERR\t") ? line.slice(4).trim() : "HTTP " + res.status
    );
  throw new Error(line || "bad bonding status");
}

/** True when fetch likely failed because lighttpd restarted mid-request (bonding toggles restart services). */
function bondingLikelyDroppedDuringToggle(e) {
  if (!e) return false;
  const name = e.name || "";
  if (name === "TypeError") return true;
  if (name === "NetworkError") return true;
  const msg = String(e.message || "");
  if (/failed to fetch|load failed|networkerror|network error/i.test(msg)) return true;
  return false;
}

async function setBondingEnabled(enabled) {
  const op = enabled ? "bonding-enable" : "bonding-disable";
  const res = await fetch("/cgi-bin/bsbf-client-web?op=" + op + "&_=" + Date.now(), {
    cache: "no-store",
  });
  let text;
  try {
    text = await res.text();
  } catch (e) {
    if (bondingLikelyDroppedDuringToggle(e)) throw e;
    throw e;
  }
  const line = text.trim().split(/\r?\n/)[0] || "";
  if (line === "ok") return { ok: true };
  if (!res.ok || line.startsWith("ERR\t"))
    throw new Error(
      line.startsWith("ERR\t") ? line.slice(4).trim() : "HTTP " + res.status
    );
  throw new Error(line || "bonding toggle failed");
}

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

function renderBondingButton(btn, enabled) {
  if (!btn) return;
  if (btn.classList.contains("is-loading")) return;
  const label = btn.querySelector(".btn-bonding-label");
  btn.disabled = false;
  if (label) label.textContent = enabled ? "Bonding: Enabled" : "Bonding: Disabled";
  else btn.textContent = enabled ? "Bonding: Enabled" : "Bonding: Disabled";
  btn.title = enabled ? "Click to disable bonding" : "Click to enable bonding";
}

function initBondingToggle() {
  const btn = document.getElementById("bonding-toggle");
  const msgEl = document.getElementById("rate-limit-msg");
  if (!btn) return;

  let cur = null;
  let bondingToggleBusy = false;
  let bondingPollInFlight = false;
  let backupPollInFlight = false;
  let lastRequested = null;

  function setError(msg) {
    if (!msgEl) return;
    msgEl.hidden = false;
    msgEl.className = "rate-limit-msg err";
    msgEl.textContent = msg;
  }

  btn.addEventListener("click", async function () {
    if (cur === null) return;
    if (bondingToggleBusy) return;
    setBondingButtonLoading(btn, true);
    if (msgEl) msgEl.hidden = true;
    const next = !cur;
    bondingToggleBusy = true;
    lastRequested = next;
    try {
      // Fire the command but do NOT gate UI on its response. We only stop loading
      // once `bsbf-bonding --status` comes back (bonding toggles can restart services).
      setBondingEnabled(next).catch(function (_) {});
    } catch (_) {
      /* ignore */
    }
  });

  setInterval(function () {
    if (bondingPollInFlight) return;
    bondingPollInFlight = true;
    fetchBondingStatus()
      .then(function (res) {
        cur = res.enabled;
        if (bondingToggleBusy) {
          // Only clear "loading" after status is observed. If the request was dropped,
          // status will keep showing the old state and we intentionally remain busy.
          if (lastRequested === null || cur === lastRequested) {
            bondingToggleBusy = false;
            lastRequested = null;
            setBondingButtonLoading(btn, false);
            renderBondingButton(btn, cur);
          }
          return;
        }
        renderBondingButton(btn, cur);
      })
      .catch(function () {})
      .finally(function () {
        bondingPollInFlight = false;
      });
  }, STATUS_POLL_MS);

  setInterval(function () {
    if (backupPollInFlight || ifaceOrder.length === 0) return;
    backupPollInFlight = true;
    fetchBondingBackupSet()
      .then(function (set) {
        syncBackupUIFromBondingSet(set);
      })
      .catch(function () {})
      .finally(function () {
        backupPollInFlight = false;
      });
  }, STATUS_POLL_MS);

  (async function () {
    try {
      setBondingButtonLoading(btn, true);
      const res = await fetchBondingStatus();
      cur = res.enabled;
      bondingToggleBusy = false;
      lastRequested = null;
      setBondingButtonLoading(btn, false);
      renderBondingButton(btn, cur);
    } catch (_) {
      cur = false;
      bondingToggleBusy = false;
      lastRequested = null;
      setBondingButtonLoading(btn, false);
      renderBondingButton(btn, cur);
      btn.title =
        "Could not read bonding status; label may be wrong until refresh. Toggle still runs enable/disable.";
    }
  })();
}

applyThemeFromQuery();
main();
initSpeedTestEmbed();
initThemeToggle();
initBondingToggle();
