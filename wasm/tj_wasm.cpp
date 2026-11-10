// WASM の入口。ページ（の Web Worker）が道路の縁・背景・検出器を渡し、1280x720 の RGBA フレームを押し込む。
// ネイティブ（main.cpp）と同じ traffic_jam.hpp / vehicle_det.hpp を使うので、同じ画素からは同じ判定が出る。
//
// build: sh wasm/build.sh   → docs/tj.js, docs/tj.wasm
#include "traffic_jam.hpp"
#include "vehicle_det.hpp"
#include <emscripten/emscripten.h>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

using namespace tj;

static std::vector<Region> g_regions;
static std::vector<StripMap> g_maps;
static std::vector<std::vector<uint8_t>> g_bg;   // 領域ごとの背景の帯（空なら校正なし）
static vd::Detector g_det;
static Params g_P;
static std::string g_json;

static void to_rgb(const unsigned char* rgba, int w, int h, Rgb& im) {
  im.w = w; im.h = h; im.px.resize((size_t)w * h * 3);
  for (size_t i = 0, n = (size_t)w * h; i < n; ++i) {
    im.px[i * 3] = rgba[i * 4]; im.px[i * 3 + 1] = rgba[i * 4 + 1]; im.px[i * 3 + 2] = rgba[i * 4 + 2];
  }
}

static void num(std::string& s, double v) {
  char b[32];
  if (std::isnan(v) || std::isinf(v)) s += "null";
  else { snprintf(b, sizeof b, "%.4g", v); s += b; }
}

extern "C" {

// 道路の縁の設定（cams/*.txt の中身）。戻り値は領域の数、失敗なら 0。エラーは tj_json() に
EMSCRIPTEN_KEEPALIVE int tj_set_regions(const char* text) {
  std::string err;
  std::vector<Region> rs;
  if (!parse_regions(text, rs, err)) { g_json = "{\"error\":\"" + err + "\"}"; return 0; }
  g_regions = rs;
  g_maps.clear();
  for (auto& r : g_regions) g_maps.push_back(make_strip_map(r));
  g_bg.assign(g_regions.size(), {});
  // 描画用：各領域の縁（左の縁 → 右の縁を逆順）と、帯の行の左右の端
  g_json = "{\"regions\":[";
  for (size_t i = 0; i < g_regions.size(); ++i) {
    const Region& r = g_regions[i];
    g_json += (i ? "," : "") + std::string("{\"name\":\"") + r.name + "\",\"lanes\":" + std::to_string(r.lanes) + ",\"poly\":[";
    bool first = true;
    auto pt = [&](const Pt& p) { g_json += first ? "" : ","; first = false; g_json += "["; num(g_json, p.x); g_json += ","; num(g_json, p.y); g_json += "]"; };
    for (const Pt& p : r.left) pt(p);
    for (auto it = r.right.rbegin(); it != r.right.rend(); ++it) pt(*it);
    g_json += "],\"rows\":" + std::to_string(g_maps[i].rows) + "}";
  }
  g_json += "]}";
  return (int)g_regions.size();
}

// 背景（車の無い道路、1280x720 の RGBA）。w=0 で校正なしに戻す
EMSCRIPTEN_KEEPALIVE int tj_set_bg(const unsigned char* rgba, int w, int h) {
  g_bg.assign(g_regions.size(), {});
  if (!rgba || w <= 0) return 0;
  Rgb im;
  to_rgb(rgba, w, h, im);
  for (size_t i = 0; i < g_regions.size(); ++i) sample_strip(im, g_maps[i], g_bg[i]);
  return 1;
}

// 車両検出器の ONNX。戻り値はノード数（0 なら失敗）
EMSCRIPTEN_KEEPALIVE int tj_load_det(const unsigned char* bytes, int n) {
  g_det.g = onx::parse_onnx(bytes, (size_t)n);
  g_det.cfg.kind = jl::DetKind::V8;
  g_det.cfg.nc = 1;
  g_det.cfg.plate_class = 0;
  g_det.cfg.imgsz = 0;
  g_det.cfg.conf = 0.25f;
  g_det.cfg.nms = 0.45f;
  g_det.cfg.v8_fmt = BoxFmt::CXCYWH;
  int iw = 0, ih = 0;
  jl::graph_input_hw(g_det.g, iw, ih);
  g_det.ok = iw > 0 && ih > 0;
  return g_det.ok ? (int)g_det.g.nodes.size() : 0;
}

// しきい値を変える（name は Params の名前、例 "jam_occ"）
EMSCRIPTEN_KEEPALIVE int tj_set_param(const char* name, double v) {
  std::string n = name;
  double* p = n == "jam_edge" ? &g_P.jam_edge : n == "jam_occ" ? &g_P.jam_occ : n == "jam_gap" ? &g_P.jam_gap
            : n == "jam_density" ? &g_P.jam_density : n == "jam_red" ? &g_P.jam_red : n == "jam_motion" ? &g_P.jam_motion
            : n == "jam_det_density" ? &g_P.jam_det_density : n == "jam_det_headway" ? &g_P.jam_det_headway : nullptr;
  if (!p) return 0;
  *p = v;
  return 1;
}

// 1 枚を判定する。prev は Δ 前のフレーム（参照の動き用、無ければ 0）。use_det=0 なら F・G を飛ばす
EMSCRIPTEN_KEEPALIVE const char* tj_analyse(const unsigned char* rgba, const unsigned char* prev_rgba, int w, int h, int use_det) {
  auto t0 = std::chrono::steady_clock::now();
  Rgb cur, prev;
  to_rgb(rgba, w, h, cur);
  if (prev_rgba) to_rgb(prev_rgba, w, h, prev);
  std::vector<DetBox> boxes;
  double det_ms = 0;
  if (use_det && g_det.ok) {
    auto d0 = std::chrono::steady_clock::now();
    for (const vd::Box& b : g_det.detect(cur.px.data(), w, h)) boxes.push_back({b.x1, b.y1, b.x2, b.y2, b.score});
    det_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - d0).count();
  }
  g_json = "{\"regions\":[";
  for (size_t i = 0; i < g_regions.size(); ++i) {
    std::vector<uint8_t> s, sp;
    sample_strip(cur, g_maps[i], s);
    if (prev_rgba) sample_strip(prev, g_maps[i], sp);
    Result r = analyse_strip(s, g_maps[i], g_regions[i].lanes, g_P, prev_rgba ? &sp : nullptr, g_bg[i].empty() ? nullptr : &g_bg[i]);
    bool has_det = use_det && g_det.ok;
    DetResult d;
    if (has_det) d = analyse_boxes(g_maps[i], g_regions[i].lanes, boxes, g_P.jam_det_density, g_P.jam_det_headway);
    int votes = r.votes + (has_det ? (int)d.jam_density + (int)d.jam_headway : 0);
    int total = has_det ? 7 : 5;
    g_json += i ? "," : "";
    g_json += "{\"name\":\"" + g_regions[i].name + "\",\"votes\":" + std::to_string(votes) + ",\"total\":" + std::to_string(total) + ",\"v\":[";
    double vs[] = {r.edge, r.occ, r.gap, r.density, r.red, has_det ? d.density : NAN, has_det ? d.headway : NAN};
    for (int k = 0; k < 7; ++k) { if (k) g_json += ","; num(g_json, vs[k]); }
    g_json += "],\"jam\":[";
    int js[] = {r.jam_edge, r.jam_occ, r.jam_gap, r.jam_density, r.jam_red, has_det ? (int)d.jam_density : -1, has_det ? (int)d.jam_headway : -1};
    for (int k = 0; k < 7; ++k) { if (k) g_json += ","; g_json += std::to_string(js[k]); }
    g_json += "],\"motion\":";
    num(g_json, r.motion);
    g_json += ",\"ref\":" + std::string(std::isnan(r.motion) ? "-1" : (r.jam_ref ? "1" : "0")) + ",\"count\":" + std::to_string(has_det ? d.count : -1) + "}";
  }
  g_json += "],\"boxes\":[";
  for (size_t k = 0; k < boxes.size(); ++k) {
    g_json += k ? ",[" : "[";
    num(g_json, boxes[k].x1); g_json += ","; num(g_json, boxes[k].y1); g_json += ","; num(g_json, boxes[k].x2); g_json += ",";
    num(g_json, boxes[k].y2); g_json += ","; num(g_json, boxes[k].score); g_json += "]";
  }
  double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  g_json += "],\"ms\":";
  num(g_json, ms);
  g_json += ",\"det_ms\":";
  num(g_json, det_ms);
  g_json += ",\"bg\":" + std::string(!g_bg.empty() && !g_bg[0].empty() ? "true" : "false") + "}";
  return g_json.c_str();
}

EMSCRIPTEN_KEEPALIVE const char* tj_json() { return g_json.c_str(); }

}  // extern "C"
