/**
 * E-reader colour screensaver backend (Google Apps Script web app).
 *
 * Sheet tab "Images", row 1 = header, then one row per chunk:
 *   A image_id | B chunk_idx | C crc32 (decimal) | D base64 data
 * (generate the CSV with the Screensaver Converter page, see firmware docs in include/screensaver.h)
 *
 * Deploy: Deploy > New deployment > Web app > Execute as: Me, Who has access: Anyone.
 * Device calls (k must equal SECRET):
 *   ?a=list&n=14&k=SECRET            -> "SS1\nL 5,9,12,...\nEND"      (n random distinct image ids)
 *   ?a=img&id=5&k=SECRET             -> "SS1\nC 5 0 <crc hex> <base64>\n...\nEND"
 *   ?a=img&id=5&c=3,7&k=SECRET       -> only the listed chunk indexes (used for repairs)
 */
const SECRET = 'change-me';
const SHEET_NAME = 'Images';

function doGet(e) {
  const p = e.parameter || {};
  if (p.k !== SECRET) return text_('ERR forbidden');
  if (p.a === 'list') return text_(list_(parseInt(p.n, 10) || 14));
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

function list_(n) {
  const ids = ids_();
  // Fisher-Yates shuffle, take n
  for (let i = ids.length - 1; i > 0; i--) {
    const j = Math.floor(Math.random() * (i + 1));
    const t = ids[i]; ids[i] = ids[j]; ids[j] = t;
  }
  return 'SS1\nL ' + ids.slice(0, Math.min(n, ids.length)).join(',') + '\nEND';
}

function image_(id, chunkCsv) {
  const sh = SpreadsheetApp.getActive().getSheetByName(SHEET_NAME);
  const last = sh.getLastRow();
  if (last < 2) return 'ERR empty';
  const want = chunkCsv ? chunkCsv.split(',').map(Number) : null;
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
