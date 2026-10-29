/**
 * E-reader colour screensaver backend (Google Apps Script web app).
 *
 * Sheet tab "__SHEET__", row 1 = header, then one row per chunk:
 *   A image_id | B chunk_idx | C crc32 (decimal) | D base64 data
 * Generate the CSV with the Screensaver Converter page and import it once; you never have to
 * update it again. The device asks for N random images every week and prefers ones it does not
 * already hold, so a sheet with as few as ~20 images keeps rotating forever.
 *
 * Deploy: Deploy > New deployment > Web app > Execute as: Me, Who has access: Anyone.
 * Device calls (k must equal SECRET):
 *   ?a=list&n=14&have=3,8&k=SECRET   -> "SS1\nL 5,9,12,...\nEND"   (n random ids, unseen first)
 *   ?a=list&n=14&mode=seq&after=9&k=SECRET -> next n ids in sheet order after id 9, wrapping (picture frame loop)
 *   ?a=img&id=5&k=SECRET             -> "SS1\nC 5 0 <crc hex> <base64>\n...\nEND"
 *   ?a=img&id=5&c=3,7&k=SECRET       -> only the listed chunk indexes (used for repairs)
 */
const SECRET = '__SECRET__';
const SHEET_NAME = '__SHEET__';

function doGet(e) {
  const p = e.parameter || {};
  if (p.k !== SECRET) return text_('ERR forbidden');
  if (p.a === 'list') return text_(list_(parseInt(p.n, 10) || 14, p.have, p.mode, parseInt(p.after, 10) || 0));
  if (p.a === 'img') return text_(image_(parseInt(p.id, 10), p.c));
  return text_('ERR bad request');
}

function text_(s) {
  return ContentService.createTextOutput(s).setMimeType(ContentService.MimeType.TEXT);
}

// Image ids are cached for 10 min so repeated device calls do not rescan the sheet.
function ids_() {
  const cache = CacheService.getScriptCache();
  const hit = cache.get('ids');
  if (hit) return JSON.parse(hit);
  const sh = SpreadsheetApp.getActive().getSheetByName(SHEET_NAME);
  const last = sh.getLastRow();
  const col = last > 1 ? sh.getRange(2, 1, last - 1, 1).getValues() : [];
  const seen = {};
  col.forEach(function (r) { if (r[0] !== '') seen[r[0]] = true; });
  const ids = Object.keys(seen).map(Number);
  cache.put('ids', JSON.stringify(ids), 600);
  return ids;
}

function shuffle_(a) {
  for (let i = a.length - 1; i > 0; i--) {
    const j = Math.floor(Math.random() * (i + 1));
    const t = a[i]; a[i] = a[j]; a[j] = t;
  }
  return a;
}

// Random picks, images the device does not hold yet come first. With 30 images and n=14 every
// sync brings new pictures; with fewer than n images you simply get all of them.
function list_(n, haveCsv, mode, after) {
  if (mode === 'seq') {   // fixed loop: the next n ids in ascending order after `after`, wrapping around
    const all = ids_().slice().sort(function (a, b) { return a - b; });
    let s = 0;
    while (s < all.length && all[s] <= after) s++;
    const seq = [];
    for (let i = 0; i < Math.min(n, all.length); i++) seq.push(all[(s + i) % all.length]);
    return 'SS1\nL ' + seq.join(',') + '\nEND';
  }
  const have = {};
  (haveCsv ? String(haveCsv).split(',') : []).forEach(function (x) { have[Number(x)] = true; });
  const ids = ids_();
  const fresh = shuffle_(ids.filter(function (id) { return !have[id]; }));
  const old = shuffle_(ids.filter(function (id) { return have[id]; }));
  const pick = fresh.concat(old).slice(0, Math.min(n, ids.length));
  return 'SS1\nL ' + pick.join(',') + '\nEND';
}

function image_(id, chunkCsv) {
  const sh = SpreadsheetApp.getActive().getSheetByName(SHEET_NAME);
  const last = sh.getLastRow();
  if (last < 2) return 'ERR empty';
  const want = chunkCsv ? String(chunkCsv).split(',').map(Number) : null;
  const data = sh.getRange(2, 1, last - 1, 4).getValues();
  const out = ['SS1'];
  for (let i = 0; i < data.length; i++) {
    const r = data[i];
    if (Number(r[0]) !== id) continue;
    const idx = Number(r[1]);
    if (want && want.indexOf(idx) < 0) continue;
    out.push('C ' + id + ' ' + idx + ' ' + Number(r[2]).toString(16) + ' ' + r[3]);
  }
  out.push('END');
  return out.join('\n');
}
