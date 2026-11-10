// 判定用の Web Worker（検出器を入れると 1 回 0.5〜3 秒かかるので、ページのスレッドでは回さない）
// 入: {type:'regions', text} / {type:'bg', pixels|null} / {type:'det', model|null}
//     {type:'frame', cur, prev, useDet}            （1280x720 の RGBA、prev は 0.33 秒前）
// 出: {type:'regions', json} / {type:'det', nodes} / {type:'result', json} / {type:'error', text}
let M = null, loading = null;
const W = 1280, H = 720;

// メッセージは続けて届くので、初期化は 1 回だけ（2 回作ると、検出器を読んだモジュールと判定するモジュールが別物になる）
async function mod() {
  if (!loading) { self.importScripts('tj.js'); loading = createTj().then(m => (M = m)); }
  return loading;
}
function put(buf) { const b = new Uint8Array(buf); const p = M._malloc(b.length); M.HEAPU8.set(b, p); return p; }

self.onmessage = async (e) => {
  const m = e.data;
  try {
    await mod();
    if (m.type === 'regions') {
      const n = M.lengthBytesUTF8(m.text) + 1, p = M._malloc(n);
      M.stringToUTF8(m.text, p, n);
      M._tj_set_regions(p);
      M._free(p);
      self.postMessage({ type: 'regions', json: M.UTF8ToString(M._tj_json()) });
    } else if (m.type === 'bg') {
      if (m.pixels) { const p = put(m.pixels); M._tj_set_bg(p, W, H); M._free(p); }
      else M._tj_set_bg(0, 0, 0);
    } else if (m.type === 'det') {
      let nodes = 0;
      if (m.model) { const b = new Uint8Array(m.model); const p = put(b); nodes = M._tj_load_det(p, b.length); M._free(p); }
      self.postMessage({ type: 'det', nodes });
    } else if (m.type === 'param') {
      const n = M.lengthBytesUTF8(m.name) + 1, p = M._malloc(n);
      M.stringToUTF8(m.name, p, n);
      M._tj_set_param(p, m.value);
      M._free(p);
    } else if (m.type === 'frame') {
      const cp = put(m.cur), pp = m.prev ? put(m.prev) : 0;
      const json = M.UTF8ToString(M._tj_analyse(cp, pp, W, H, m.useDet ? 1 : 0));
      M._free(cp); if (pp) M._free(pp);
      self.postMessage({ type: 'result', json });
    }
  } catch (err) {
    self.postMessage({ type: 'error', text: String(err) });
  }
};
