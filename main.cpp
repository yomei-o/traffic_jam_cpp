// traffic_jam — 道路カメラの mp4 から、領域（道路）ごとに渋滞かどうかを何種類かの方法で判別する
//
// 判別そのものは 1 枚の静止画で行う（A〜E、--det で検出器を渡せば F・G も）。
// 動画は答え合わせの「参照の動き」R のためだけに使う。
// mp4 の展開・RGB 変換・書き出しは mediakit_cpp、車両検出器は vehicle_det_cpp。
//   traffic_jam <入力.mp4 | 入力.rgb> --regions cams/xxx.txt [--bg bg.bmp] [--det vehicle_det.onnx]
//               [--csv out.csv] [--overlay out.mp4] [--snap out.bmp] [--strips out.bmp] [--every N] [--max N] [--delta N]
#define _CRT_SECURE_NO_WARNINGS
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "mk_codec.h"
#include "traffic_jam.hpp"
#include "vehicle_det.hpp"   // vehicle_det_cpp の車両検出器（F・G）

using namespace tj;

struct Args {
    std::string in, regions, csv = "jam.csv", overlay, snap, strips, bg, make_bg, det;
    int every = 1, max_frames = 0, delta = 10, snap_at = 30, bg_every = 30, det_every = 15;
    float det_conf = 0.25f;
    Params P;
};

static Args parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        auto nx = [&]() { if (i + 1 >= argc) { fprintf(stderr, "missing value for %s\n", s.c_str()); exit(2); } return std::string(argv[++i]); };
        if (s == "--regions") a.regions = nx();
        else if (s == "--csv") a.csv = nx();
        else if (s == "--overlay") a.overlay = nx();
        else if (s == "--snap") a.snap = nx();
        else if (s == "--strips") a.strips = nx();
        else if (s == "--bg") a.bg = nx();
        else if (s == "--make-bg") a.make_bg = nx();
        else if (s == "--bg-every") a.bg_every = std::stoi(nx());
        else if (s == "--det") a.det = nx();
        else if (s == "--det-every") a.det_every = std::stoi(nx());
        else if (s == "--det-conf") a.det_conf = std::stof(nx());
        else if (s == "--jam-det-density") a.P.jam_det_density = std::stod(nx());
        else if (s == "--jam-det-headway") a.P.jam_det_headway = std::stod(nx());
        else if (s == "--snap-at") a.snap_at = std::stoi(nx());
        else if (s == "--every") a.every = std::stoi(nx());
        else if (s == "--max") a.max_frames = std::stoi(nx());
        else if (s == "--delta") a.delta = std::stoi(nx());
        else if (s == "--jam-edge") a.P.jam_edge = std::stod(nx());
        else if (s == "--jam-occ") a.P.jam_occ = std::stod(nx());
        else if (s == "--jam-gap") a.P.jam_gap = std::stod(nx());
        else if (s == "--jam-density") a.P.jam_density = std::stod(nx());
        else if (s == "--jam-red") a.P.jam_red = std::stod(nx());
        else if (s == "--jam-motion") a.P.jam_motion = std::stod(nx());
        else if (s[0] != '-' && a.in.empty()) a.in = s;
        else { fprintf(stderr, "unknown option %s\n", s.c_str()); exit(2); }
    }
    if (a.in.empty() || a.regions.empty()) {
        fprintf(stderr, "usage: traffic_jam <video.mp4|video.rgb> --regions file [--csv f] [--overlay f.mp4] [--snap f.bmp] [--strips f.bmp]\n"
                        "                   [--bg bg.bmp] [--make-bg out.bmp [--bg-every N]]\n"
                        "                   [--det vehicle_det.onnx [--det-every N] [--det-conf f] [--jam-det-density v] [--jam-det-headway v]]\n"
                        "                   [--every N] [--max N] [--delta N] [--jam-edge v] [--jam-occ v] [--jam-gap v] [--jam-density v] [--jam-red v]\n");
        exit(2);
    }
    return a;
}

// ---- 描画 ----
struct Color { uint8_t r, g, b; };
static void put(Rgb& im, int x, int y, Color c) {
    if (x < 0 || y < 0 || x >= im.w || y >= im.h) return;
    uint8_t* p = &im.px[((size_t)y * im.w + x) * 3]; p[0] = c.r; p[1] = c.g; p[2] = c.b;
}
static void line(Rgb& im, Pt a, Pt b, Color c, int th = 1) {
    int n = (int)std::max(std::fabs(b.x - a.x), std::fabs(b.y - a.y)) + 1;
    for (int i = 0; i <= n; ++i) {
        double t = (double)i / n;
        int x = (int)std::lround(a.x + (b.x - a.x) * t), y = (int)std::lround(a.y + (b.y - a.y) * t);
        for (int dy = -(th / 2); dy <= th / 2; ++dy) for (int dx = -(th / 2); dx <= th / 2; ++dx) put(im, x + dx, y + dy, c);
    }
}
static void tint_row(Rgb& im, Pt a, Pt b, Color c, double alpha) {
    int n = (int)std::max(std::fabs(b.x - a.x), std::fabs(b.y - a.y)) + 1;
    for (int i = 0; i <= n; ++i) {
        double t = (double)i / n;
        int x = (int)std::lround(a.x + (b.x - a.x) * t), y = (int)std::lround(a.y + (b.y - a.y) * t);
        if (x < 0 || y < 0 || x >= im.w || y >= im.h) continue;
        uint8_t* p = &im.px[((size_t)y * im.w + x) * 3];
        p[0] = (uint8_t)(p[0] * (1 - alpha) + c.r * alpha); p[1] = (uint8_t)(p[1] * (1 - alpha) + c.g * alpha); p[2] = (uint8_t)(p[2] * (1 - alpha) + c.b * alpha);
    }
}
static void box(Rgb& im, int x, int y, int w, int h, Color c) { for (int j = 0; j < h; ++j) for (int i = 0; i < w; ++i) put(im, x + i, y + j, c); }

static const Color C_JAM{230, 40, 40}, C_FREE{40, 200, 80}, C_OFF{90, 90, 90}, C_VEH{255, 220, 0};

// 領域を塗る：票の数で緑→赤、車として拾った区間を黄色の枠、方法ごとの判定を小さな四角で
static void draw_region(Rgb& im, const Region& rg, const StripMap& m, const Result& r, const DetResult* dr) {
    double f = (r.votes + (dr ? dr->jam_density + dr->jam_headway : 0)) / (dr ? 7.0 : 5.0);
    Color tint{(uint8_t)(40 + 190 * f), (uint8_t)(200 - 160 * f), (uint8_t)(80 - 40 * f)};
    for (int y = 0; y < m.rows; ++y) tint_row(im, m.rowL[y], m.rowR[y], tint, 0.28);
    for (size_t i = 1; i < rg.left.size(); ++i) line(im, rg.left[i - 1], rg.left[i], {255, 255, 255});
    for (size_t i = 1; i < rg.right.size(); ++i) line(im, rg.right[i - 1], rg.right[i], {255, 255, 255});
    // 車：車線の帯の中で、始まりと終わりの行を結ぶ四角
    for (size_t k = 0; k < r.lanes.size(); ++k) {
        double t0 = (double)k / r.lanes.size() + 0.04, t1 = (double)(k + 1) / r.lanes.size() - 0.04;
        auto at = [&](int row, double t) {
            row = std::clamp(row, 0, m.rows - 1);
            return Pt{m.rowL[row].x + (m.rowR[row].x - m.rowL[row].x) * t, m.rowL[row].y + (m.rowR[row].y - m.rowL[row].y) * t};
        };
        for (auto [a, b] : r.lanes[k].vehicles) {
            Pt p00 = at(a, t0), p01 = at(a, t1), p10 = at(b - 1, t0), p11 = at(b - 1, t1);
            line(im, p00, p01, C_VEH, 2); line(im, p10, p11, C_VEH, 2); line(im, p00, p10, C_VEH, 2); line(im, p01, p11, C_VEH, 2);
        }
    }
    // 判定の四角（A B C D E F G | R）を手前の左端の近くに。F・G は検出器が無ければ灰色
    Pt o = rg.left[0];
    int x = std::clamp((int)o.x + 6, 0, im.w - 150), y = std::clamp((int)o.y - 26, 0, im.h - 20);
    bool js[] = {r.jam_edge, r.jam_occ, r.jam_gap, r.jam_density, r.jam_red, dr && dr->jam_density, dr && dr->jam_headway};
    box(im, x - 2, y - 2, 8 * 16 + 10, 18, {0, 0, 0});
    for (int i = 0; i < 7; ++i) box(im, x + i * 16, y, 13, 13, (i >= 5 && !dr) ? C_OFF : (js[i] ? C_JAM : C_FREE));
    box(im, x + 7 * 16 + 6, y, 13, 13, std::isnan(r.motion) ? C_OFF : (r.jam_ref ? C_JAM : C_FREE));
}

// 24bit の BMP（write_bmp が書く形）を読む
static bool read_bmp(const std::string& path, Rgb& im) {
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return false;
    uint8_t h[54];
    if (fread(h, 1, 54, f) != 54 || h[0] != 'B' || h[1] != 'M' || h[28] != 24) { fclose(f); return false; }
    auto u32 = [&](int o) { return (uint32_t)h[o] | ((uint32_t)h[o + 1] << 8) | ((uint32_t)h[o + 2] << 16) | ((uint32_t)h[o + 3] << 24); };
    int w = (int)u32(18), hh = (int)u32(22); uint32_t off = u32(10);
    bool bottom_up = hh > 0; if (hh < 0) hh = -hh;
    fseek(f, (long)off, SEEK_SET);
    int row = (w * 3 + 3) & ~3;
    std::vector<uint8_t> ln(row);
    im.w = w; im.h = hh; im.px.assign((size_t)w * hh * 3, 0);
    for (int i = 0; i < hh; ++i) {
        if (fread(ln.data(), 1, row, f) != (size_t)row) { fclose(f); return false; }
        int y = bottom_up ? hh - 1 - i : i;
        for (int x = 0; x < w; ++x) {
            uint8_t* p = &im.px[((size_t)y * w + x) * 3];
            p[0] = ln[x * 3 + 2]; p[1] = ln[x * 3 + 1]; p[2] = ln[x * 3];
        }
    }
    fclose(f);
    return true;
}

static bool write_bmp(const std::string& path, const Rgb& im) {
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) return false;
    int row = (im.w * 3 + 3) & ~3;
    uint32_t size = 54 + (uint32_t)row * im.h;
    uint8_t h[54] = {'B', 'M'};
    auto u32 = [&](int o, uint32_t v) { for (int i = 0; i < 4; ++i) h[o + i] = (uint8_t)(v >> (8 * i)); };
    u32(2, size); u32(10, 54); u32(14, 40); u32(18, im.w); u32(22, im.h);
    h[26] = 1; h[28] = 24; u32(34, size - 54);
    fwrite(h, 1, 54, f);
    std::vector<uint8_t> ln(row, 0);
    for (int y = im.h - 1; y >= 0; --y) {
        for (int x = 0; x < im.w; ++x) {
            const uint8_t* p = &im.px[((size_t)y * im.w + x) * 3];
            ln[x * 3] = p[2]; ln[x * 3 + 1] = p[1]; ln[x * 3 + 2] = p[0];
        }
        fwrite(ln.data(), 1, row, f);
    }
    fclose(f);
    return true;
}

// 帯画像を並べた確認用の絵：領域ごとに [帯 | 舗装でない画素] を 3 倍に拡大して横に並べる
static Rgb strips_image(const std::vector<StripMap>& maps, const std::vector<std::vector<uint8_t>>& ss, const std::vector<Result>& rs) {
    const int Z = 3, GAP = 12;
    int W = GAP, H = 0;
    for (auto& m : maps) { W += 2 * m.cols * Z + 2 * GAP; H = std::max(H, m.rows * Z); }
    Rgb o; o.w = W; o.h = H + 2 * GAP; o.px.assign((size_t)o.w * o.h * 3, 40);
    int x0 = GAP;
    for (size_t i = 0; i < maps.size(); ++i) {
        const auto& m = maps[i];
        for (int y = 0; y < m.rows; ++y) for (int x = 0; x < m.cols; ++x) {
            const uint8_t* p = &ss[i][((size_t)y * m.cols + x) * 3];
            bool nr = rs[i].nonroad[(size_t)y * m.cols + x];
            for (int dy = 0; dy < Z; ++dy) for (int dx = 0; dx < Z; ++dx) {
                // 奥を上に（帯の行 0 ＝手前が下）
                int Y = GAP + (m.rows - 1 - y) * Z + dy;
                put(o, x0 + x * Z + dx, Y, {p[0], p[1], p[2]});
                put(o, x0 + (m.cols + 1) * Z + x * Z + dx, Y, interior_col(x) ? (nr ? Color{255, 255, 255} : Color{0, 0, 0}) : Color{70, 70, 120});
            }
        }
        for (size_t k = 0; k < rs[i].lanes.size(); ++k)
            for (auto [a, b] : rs[i].lanes[k].vehicles)
                for (int y = a; y < b; ++y)
                    box(o, x0 + (m.cols + 1) * Z + (int)k * kColsPerLane * Z, GAP + (m.rows - 1 - y) * Z, 2, Z, C_VEH);
        x0 += 2 * m.cols * Z + 2 * GAP;
    }
    return o;
}

int main(int argc, char** argv) {
    Args a = parse(argc, argv);
    std::ifstream rf(a.regions, std::ios::binary);
    if (!rf) { fprintf(stderr, "cannot read %s\n", a.regions.c_str()); return 1; }
    std::stringstream ss; ss << rf.rdbuf();
    std::vector<Region> regions; std::string err;
    if (!parse_regions(ss.str(), regions, err)) { fprintf(stderr, "%s: %s\n", a.regions.c_str(), err.c_str()); return 1; }
    std::vector<StripMap> maps;
    for (auto& r : regions) maps.push_back(make_strip_map(r));

    const bool raw = a.in.size() > 4 && a.in.compare(a.in.size() - 4, 4, ".rgb") == 0;
    mk::Mp4Reader reader;
    mk::MediaInfo info;
    if (raw) { info.has_video = true; info.width = 1280; info.height = 720; info.fps = 30; }
    else {
        if (!reader.open(a.in)) { fprintf(stderr, "open %s: %s\n", a.in.c_str(), reader.error().c_str()); return 1; }
        info = reader.info();
    }
    if (!info.has_video) { fprintf(stderr, "%s: no video\n", a.in.c_str()); return 1; }
    mk::VideoDecoder dec;
    if (!raw && !dec.open(false)) { fprintf(stderr, "decoder: %s\n", dec.error().c_str()); return 1; }
    mk::FrameConverter conv;
    const int W = 1280, H = 720;
    const double fps = info.fps > 0 ? info.fps : 30;

    mk::VideoEncoder enc;
    mk::Mp4Writer writer;
    if (!a.overlay.empty()) {
        mk::VideoEncoderConfig ec;
        ec.width = W; ec.height = H; ec.fps = std::max(1, (int)std::lround(fps / a.every)); ec.bitrate = 4000000; ec.gop = ec.fps;
        if (!enc.open(ec)) { fprintf(stderr, "encoder: %s\n", enc.error().c_str()); return 1; }
        if (!writer.open(a.overlay, &enc, nullptr)) { fprintf(stderr, "writer: %s\n", writer.error().c_str()); return 1; }
    }
    FILE* csv = fopen(a.csv.c_str(), "w");
    if (!csv) { fprintf(stderr, "cannot write %s\n", a.csv.c_str()); return 1; }
    fprintf(csv, "frame,region,edge,occ,gap,density,red,det_count,det_density,det_headway,motion,"
                 "jam_edge,jam_occ,jam_gap,jam_density,jam_red,jam_det_density,jam_det_headway,votes,jam_ref\n");
    vd::Detector detector;
    if (!a.det.empty() && !detector.load(a.det, a.det_conf)) { fprintf(stderr, "%s: cannot use as a detector\n", a.det.c_str()); return 1; }
    std::vector<DetBox> last_boxes;

    Rgb rgb; rgb.w = W; rgb.h = H; rgb.px.resize((size_t)W * H * 3);
    // 校正の背景（車の無い道路）。領域ごとの帯にしておく
    std::vector<std::vector<uint8_t>> bgstrips(regions.size());
    if (!a.bg.empty()) {
        Rgb bgim;
        if (!read_bmp(a.bg, bgim) || bgim.w != W || bgim.h != H) { fprintf(stderr, "%s: need a 1280x720 24bit BMP\n", a.bg.c_str()); return 1; }
        for (size_t i = 0; i < regions.size(); ++i) sample_strip(bgim, maps[i], bgstrips[i]);
    }
    std::vector<std::vector<uint8_t>> bg_frames;  // --make-bg 用
    // 参照の動きのため、領域ごとに直近 delta フレームの帯を覚えておく（every とは無関係に全フレーム）
    std::vector<std::deque<std::vector<uint8_t>>> past(regions.size());
    // 方法 0..6 = A B C D E F G、v[7] は参照の動き
    struct Stat { int n[7] = {}, ref_n[7] = {}, agree[7] = {}, jam[7] = {}, ref_jam = 0, ref_all = 0; std::vector<double> v[8]; };
    std::vector<Stat> st(regions.size());
    int frame = 0, used = 0;
    bool done = false, snapped = false;
    std::vector<mk::Packet> enc_out;

    auto analyse = [&](int fi) {
        if (!a.make_bg.empty() && fi % a.bg_every == 0) bg_frames.push_back(rgb.px);
        bool use = fi % a.every == 0;
        // 検出器は重いので det_every フレームおき（その間の重ね描きは直前の箱）
        bool det_now = use && detector.ok && fi % a.det_every == 0;
        if (det_now) {
            last_boxes.clear();
            for (const vd::Box& b : detector.detect(rgb.px.data(), W, H)) last_boxes.push_back({b.x1, b.y1, b.x2, b.y2, b.score});
        }
        std::vector<std::vector<uint8_t>> strips(regions.size());
        std::vector<Result> res(regions.size());
        std::vector<DetResult> dres(regions.size());
        for (size_t i = 0; i < regions.size(); ++i) {
            sample_strip(rgb, maps[i], strips[i]);
            auto& q = past[i];
            if (use) {
                const std::vector<uint8_t>* prev = (int)q.size() >= a.delta ? &q.front() : nullptr;
                res[i] = analyse_strip(strips[i], maps[i], regions[i].lanes, a.P, prev, a.bg.empty() ? nullptr : &bgstrips[i]);
                if (detector.ok) dres[i] = analyse_boxes(maps[i], regions[i].lanes, last_boxes, a.P.jam_det_density, a.P.jam_det_headway);
                const Result& r = res[i];
                const DetResult& d = dres[i];
                if (det_now)
                    fprintf(csv, "%d,%s,%.4f,%.4f,%.3f,%.3f,%.5f,%d,%.3f,%.3f,%.4f,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", fi, regions[i].name.c_str(), r.edge, r.occ,
                            r.gap, r.density, r.red, d.count, d.density, d.headway, r.motion, r.jam_edge, r.jam_occ, r.jam_gap, r.jam_density, r.jam_red,
                            (int)d.jam_density, (int)d.jam_headway, r.votes, std::isnan(r.motion) ? -1 : (int)r.jam_ref);
                else
                    fprintf(csv, "%d,%s,%.4f,%.4f,%.3f,%.3f,%.5f,,,,%.4f,%d,%d,%d,%d,%d,,,%d,%d\n", fi, regions[i].name.c_str(), r.edge, r.occ, r.gap,
                            r.density, r.red, r.motion, r.jam_edge, r.jam_occ, r.jam_gap, r.jam_density, r.jam_red, r.votes,
                            std::isnan(r.motion) ? -1 : (int)r.jam_ref);
                Stat& s = st[i];
                bool js[] = {r.jam_edge, r.jam_occ, r.jam_gap, r.jam_density, r.jam_red, d.jam_density, d.jam_headway};
                double vs[] = {r.edge, r.occ, r.gap, r.density, r.red, d.density, d.headway};
                const int nk = det_now ? 7 : 5;
                for (int k = 0; k < nk; ++k) {
                    ++s.n[k]; s.jam[k] += js[k];
                    if (!std::isnan(vs[k])) s.v[k].push_back(vs[k]);
                    if (!std::isnan(r.motion)) { ++s.ref_n[k]; s.agree[k] += js[k] == r.jam_ref; }
                }
                if (!std::isnan(r.motion)) { s.v[7].push_back(r.motion); ++s.ref_all; s.ref_jam += r.jam_ref; }
            }
            q.push_back(strips[i]);
            if ((int)q.size() > a.delta) q.pop_front();
        }
        if (!use) return;
        bool want_snap = !snapped && used >= a.snap_at && (!a.snap.empty() || !a.strips.empty());
        if (!a.overlay.empty() || want_snap) {
            Rgb o = rgb;
            for (size_t i = 0; i < regions.size(); ++i) draw_region(o, regions[i], maps[i], res[i], detector.ok ? &dres[i] : nullptr);
            // 検出器の箱（水色）と、数えた位置（下端の中央）
            for (const DetBox& b : last_boxes) {
                Color c{0, 220, 255};
                line(o, {b.x1, b.y1}, {b.x2, b.y1}, c); line(o, {b.x1, b.y2}, {b.x2, b.y2}, c);
                line(o, {b.x1, b.y1}, {b.x1, b.y2}, c); line(o, {b.x2, b.y1}, {b.x2, b.y2}, c);
                box(o, (int)((b.x1 + b.x2) / 2) - 2, (int)b.y2 - 2, 5, 5, c);
            }
            if (!a.overlay.empty()) {
                mk::VideoFrame of;
                of.width = W; of.height = H; of.format = mk::PixelFormat::RGB24;
                of.buffer = o.px; of.data[0] = of.buffer.data(); of.stride[0] = W * 3;
                of.pts_us = (int64_t)(used * 1e6 * a.every / fps);
                enc_out.clear();
                if (!enc.encode(of, enc_out)) fprintf(stderr, "encode: %s\n", enc.error().c_str());
                for (auto& p : enc_out) writer.write(p);
            }
            if (want_snap) {
                if (!a.snap.empty()) write_bmp(a.snap, o);
                if (!a.strips.empty()) write_bmp(a.strips, strips_image(maps, strips, res));
                snapped = true;
            }
        }
        ++used;
        if (a.max_frames && used >= a.max_frames) done = true;
    };

    auto handle = [&](const mk::VideoFrame& vf) {
        if (done) return;
        int fi = frame++;
        mk::VideoFrame rf2;
        if (!conv.convert(vf, rf2, mk::PixelFormat::RGB24, W, H)) { fprintf(stderr, "convert failed\n"); done = true; return; }
        for (int y = 0; y < H; ++y) memcpy(&rgb.px[(size_t)y * W * 3], rf2.data[0] + (size_t)y * rf2.stride[0], (size_t)W * 3);
        analyse(fi);
    };

    mk::Packet pkt;
    std::vector<mk::VideoFrame> frames;
    if (raw) {
        FILE* f = fopen(a.in.c_str(), "rb");
        if (!f) { fprintf(stderr, "open %s failed\n", a.in.c_str()); return 1; }
        while (!done && fread(rgb.px.data(), 1, rgb.px.size(), f) == rgb.px.size()) analyse(frame++);
        fclose(f);
    } else {
        while (!done && reader.read(pkt)) {
            if (pkt.type != mk::MediaType::Video) continue;
            frames.clear();
            if (!dec.decode(pkt, frames)) { fprintf(stderr, "decode: %s\n", dec.error().c_str()); break; }
            for (auto& f : frames) handle(f);
        }
        if (!done) { frames.clear(); dec.flush(frames); for (auto& f : frames) handle(f); }
    }
    if (!a.overlay.empty()) {
        enc_out.clear(); enc.flush(enc_out);
        for (auto& p : enc_out) writer.write(p);
        writer.close();
    }
    fclose(csv);
    if (!a.make_bg.empty() && !bg_frames.empty()) {
        // 画素・色ごとの中央値＝車の通り過ぎた後の道路（渋滞で止まっている車は消えないので、流れている時間に作る）
        Rgb bgim; bgim.w = W; bgim.h = H; bgim.px.resize((size_t)W * H * 3);
        std::vector<uint8_t> v(bg_frames.size());
        for (size_t k = 0; k < bgim.px.size(); ++k) {
            for (size_t j = 0; j < bg_frames.size(); ++j) v[j] = bg_frames[j][k];
            std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
            bgim.px[k] = v[v.size() / 2];
        }
        write_bmp(a.make_bg, bgim);
        printf("background from %zu frames -> %s\n", bg_frames.size(), a.make_bg.c_str());
    }

    printf("%s: %dx%d %.2f fps, frames analysed: %d (every %d), reference motion over %d frames\n", a.in.c_str(), info.width, info.height, fps, used,
           a.every, a.delta);
    const char* names[] = {"A edge       ", "B occ        ", "C gap        ", "D density    ", "E red        ", "F det-density", "G det-headway"};
    for (size_t i = 0; i < regions.size(); ++i) {
        const Stat& s = st[i];
        printf("\n[%s] %d lanes, strip %dx%d (length %.1f lane widths)\n", regions[i].name.c_str(), regions[i].lanes, maps[i].cols, maps[i].rows,
               (double)maps[i].rows / kColsPerLane);
        printf("  reference R: motion median %.3f, jam in %d/%d frames\n", s.v[7].empty() ? NAN : median_of(s.v[7]), s.ref_jam, s.ref_all);
        printf("  method          median   jam%%   agree-with-R%%  (frames)\n");
        for (int k = 0; k < 7; ++k) {
            if (!s.n[k]) continue;
            printf("  %s %8.4f  %5.1f   %5.1f        %d\n", names[k], median_of(s.v[k]), 100.0 * s.jam[k] / s.n[k],
                   s.ref_n[k] ? 100.0 * s.agree[k] / s.ref_n[k] : NAN, s.n[k]);
        }
    }
    return 0;
}
