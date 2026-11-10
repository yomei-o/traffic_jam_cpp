// node wasm/test_node.js regions.txt bg.rgba det.onnx prev.rgba cur.rgba   （1280x720）
const fs = require('fs');
const createTj = require('../docs/tj.js');
(async () => {
  const [reg, bg, det, prev, cur] = process.argv.slice(2);
  const M = await createTj();
  const put = (buf) => { const p = M._malloc(buf.length); M.HEAPU8.set(buf, p); return p; };
  const txt = fs.readFileSync(reg, 'utf8');
  const n = M.lengthBytesUTF8(txt) + 1, tp = M._malloc(n); M.stringToUTF8(txt, tp, n);
  console.log('regions', M._tj_set_regions(tp));
  if (bg !== '-') console.log('bg', M._tj_set_bg(put(fs.readFileSync(bg)), 1280, 720));
  const ob = fs.readFileSync(det); console.log('det nodes', M._tj_load_det(put(ob), ob.length));
  const pp = put(fs.readFileSync(prev)), cp = put(fs.readFileSync(cur));
  console.log(M.UTF8ToString(M._tj_analyse(cp, pp, 1280, 720, 1)).replace(/,"boxes":\[.*?\]\],/, ',"boxes":[...],'));
})();
