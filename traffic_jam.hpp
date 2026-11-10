// traffic_jam.hpp — 道路カメラの静止画 1 枚から渋滞かどうかを判別する（ヘッダーオンリー、依存なし）
//
// 道路は「左の縁」「右の縁」の折れ線（手前→奥、同じ点数）で与える。
// まずその間を、車線幅を物差しにした「まっすぐな帯画像」に直す（Strip）。
//   - 帯の列：道路の幅を C 等分（1 車線あたり kColsPerLane 列）
//   - 帯の行：その場所の道路幅 / C ずつ奥へ進む
// こうすると遠くの車も近くの車も、帯の上ではほぼ同じ大きさになり、
// 長さは「車線幅の何倍か」という比で測れる（ただし道路方向の見かけの縮み＝俯角の分は残る）。
//
// 静止画の方法（1 枚で決まる）
//   A エッジ密度       … 車は輪郭・窓・影で縁が多く、空いた舗装はなめらか
//   B 占有率           … 舗装の色（帯の最頻色）から外れた画素の割合
//   C 車間             … 車線ごとに「車のいる行」を拾い、車と車の隙間の中央値（車線幅の何倍か）
//   D 台数密度         … 車線 1 本・車線幅 10 本分の長さあたりの台数
//   E 赤い尾灯         … 赤く明るい画素の割合（ブレーキランプ。昼は弱い）
// 参照（静止画ではない。答え合わせ用）
//   R 動き             … Δ フレーム前との差で動いた画素 ÷ 車の画素。止まっていれば 0 に近い
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace tj {

constexpr int kColsPerLane = 16;

struct Rgb { int w = 0, h = 0; std::vector<uint8_t> px; };  // RGB24
struct Pt { double x = 0, y = 0; };

struct Region {
    std::string name;
    int lanes = 1;
    std::vector<Pt> left, right;  // 手前→奥
};

// 折れ線を弧長の割合 f(0..1) で引く
inline Pt poly_at(const std::vector<Pt>& p, const std::vector<double>& cum, double f) {
    double t = f * cum.back();
    size_t i = 1;
    while (i + 1 < p.size() && cum[i] < t) ++i;
    double seg = cum[i] - cum[i - 1];
    double u = seg > 0 ? (t - cum[i - 1]) / seg : 0;
    return {p[i - 1].x + (p[i].x - p[i - 1].x) * u, p[i - 1].y + (p[i].y - p[i - 1].y) * u};
}
inline std::vector<double> cumlen(const std::vector<Pt>& p) {
    std::vector<double> c(p.size(), 0);
    for (size_t i = 1; i < p.size(); ++i) c[i] = c[i - 1] + std::hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
    return c;
}

// 帯画像の幾何（領域ごとに 1 回だけ作る）
struct StripMap {
    int cols = 0, rows = 0;
    std::vector<float> sx, sy;      // rows*cols の標本位置（画像座標）
    std::vector<Pt> rowL, rowR;     // 各行の左右の端（描画用）
};

inline StripMap make_strip_map(const Region& r) {
    StripMap m;
    m.cols = r.lanes * kColsPerLane;
    auto cl = cumlen(r.left), cr = cumlen(r.right);
    // 細かい表：s ごとの左右の点・幅・中心線の弧長
    const int N = 4000;
    std::vector<Pt> L(N + 1), R(N + 1);
    std::vector<double> A(N + 1, 0), Wd(N + 1);
    for (int i = 0; i <= N; ++i) {
        double f = (double)i / N;
        L[i] = poly_at(r.left, cl, f); R[i] = poly_at(r.right, cr, f);
        Wd[i] = std::hypot(R[i].x - L[i].x, R[i].y - L[i].y);
        if (i) {
            double cx0 = (L[i - 1].x + R[i - 1].x) / 2, cy0 = (L[i - 1].y + R[i - 1].y) / 2;
            double cx1 = (L[i].x + R[i].x) / 2, cy1 = (L[i].y + R[i].y) / 2;
            A[i] = A[i - 1] + std::hypot(cx1 - cx0, cy1 - cy0);
        }
    }
    // 行：その場所の幅 / cols ずつ中心線に沿って進む
    std::vector<int> idx;
    int i = 0;
    while (i <= N) {
        idx.push_back(i);
        double step = std::max(0.5, Wd[i] / m.cols);
        double target = A[i] + step;
        while (i <= N && A[i] < target) ++i;
    }
    m.rows = (int)idx.size();
    m.sx.resize((size_t)m.rows * m.cols); m.sy.resize((size_t)m.rows * m.cols);
    for (int y = 0; y < m.rows; ++y) {
        const Pt& a = L[idx[y]]; const Pt& b = R[idx[y]];
        m.rowL.push_back(a); m.rowR.push_back(b);
        for (int x = 0; x < m.cols; ++x) {
            double t = (x + 0.5) / m.cols;
            m.sx[(size_t)y * m.cols + x] = (float)(a.x + (b.x - a.x) * t);
            m.sy[(size_t)y * m.cols + x] = (float)(a.y + (b.y - a.y) * t);
        }
    }
    return m;
}

// 帯画像（RGB）を取り出す。双線形補間
inline void sample_strip(const Rgb& im, const StripMap& m, std::vector<uint8_t>& out) {
    out.assign((size_t)m.rows * m.cols * 3, 0);
    for (size_t k = 0; k < (size_t)m.rows * m.cols; ++k) {
        double x = m.sx[k], y = m.sy[k];
        int x0 = (int)std::floor(x - 0.5), y0 = (int)std::floor(y - 0.5);
        double fx = x - 0.5 - x0, fy = y - 0.5 - y0;
        x0 = std::clamp(x0, 0, im.w - 2); y0 = std::clamp(y0, 0, im.h - 2);
        fx = std::clamp(fx, 0.0, 1.0); fy = std::clamp(fy, 0.0, 1.0);
        for (int c = 0; c < 3; ++c) {
            auto P = [&](int xx, int yy) { return (double)im.px[((size_t)yy * im.w + xx) * 3 + c]; };
            double v = (P(x0, y0) * (1 - fx) + P(x0 + 1, y0) * fx) * (1 - fy) + (P(x0, y0 + 1) * (1 - fx) + P(x0 + 1, y0 + 1) * fx) * fy;
            out[k * 3 + c] = (uint8_t)std::lround(v);
        }
    }
}

// 車線の内側の列か（車線の境目の白線と、道路の縁の 2 列ずつは使わない）
inline bool interior_col(int x) { int k = x % kColsPerLane; return k >= 2 && k < kColsPerLane - 2; }

inline int luma(const uint8_t* p) { return (77 * p[0] + 150 * p[1] + 29 * p[2]) >> 8; }

struct Params {
    int road_dy = 24;        // 舗装の明るさからの許容差
    int road_dc = 14;        // 舗装の色（R−B）からの許容差
    double row_occ = 0.4;    // 行の何割が「舗装でない」なら車がいる行とみなすか
    int min_vehicle = 6;     // これより短い（行）占有は車とみなさない
    int max_hole = 3;        // 車の中のこれ以下の隙間（行）は埋める
    int edge_thr = 48;       // エッジとみなす勾配
    int motion_thr = 20;     // 動いたとみなす明るさの差
    // 判定のしきい値（これを越えたら渋滞）
    double jam_edge = 0.16;
    double jam_occ = 0.30;
    double jam_gap = 2.0;    // 車間がこれ（車線幅の倍数）より短い
    double jam_density = 2.5;
    double jam_red = 0.004;
    double jam_motion = 0.35;  // 参照：動きがこれより小さく、占有率が jam_ref_occ より大きい
    double jam_ref_occ = 0.15;
    double jam_det_density = 2.5;   // F：検出台数 / (車線 1 本・車線幅 10 本)
    double jam_det_headway = 3.0;   // G：車頭間隔がこれ（車線幅の倍数）より短い
};

struct Lane {
    std::vector<std::pair<int, int>> vehicles;  // [行の始め, 終わり)
    double gap = NAN;      // 車間の中央値（車線幅の倍数）
    double density = 0;    // 台 / (車線幅 10 本分の長さ)
};

struct Result {
    double edge = 0, occ = 0, gap = NAN, density = 0, red = 0, motion = NAN;
    bool jam_edge = false, jam_occ = false, jam_gap = false, jam_density = false, jam_red = false, jam_ref = false;
    int votes = 0;  // 静止画 5 方法のうち渋滞と言った数
    std::vector<Lane> lanes;
    std::array<int, 2> road_color{0, 0};  // 舗装の (明るさ, R−B)
    std::vector<uint8_t> nonroad;          // 帯画像と同じ大きさ、1 = 舗装でない
};

inline double median_of(std::vector<double> v) {
    if (v.empty()) return NAN;
    std::sort(v.begin(), v.end());
    size_t n = v.size();
    return n % 2 ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2;
}

// 舗装の色（校正なしのとき）。白い車・黒い車は明るさが端に張り付いて 1 か所に集まるので、
// 明るさ 25〜200 で色の薄い（|R−B| < 24）画素だけで明るさの分布を作り、±6 でならした最頻値を舗装とする。
// 色 R−B は、その明るさ ±8 の画素の中央値。車線ごとに明るさが違う（日向・日陰、舗装の新旧）ので車線ごとに求める。
inline std::array<int, 2> road_color_of(const std::vector<uint8_t>& s, int W, int H, int x0, int x1) {
    std::vector<long> hist(256, 0);
    for (int y = 0; y < H; ++y) for (int x = x0; x < x1; ++x) {
        if (!interior_col(x)) continue;
        const uint8_t* p = &s[((size_t)y * W + x) * 3];
        int Y = luma(p);
        if (Y >= 25 && Y <= 200 && std::abs(p[0] - p[2]) < 24) ++hist[Y];
    }
    long best = -1; int ry = 0;
    for (int v = 0; v < 256; ++v) {
        long acc = 0;
        for (int d = -6; d <= 6; ++d) if (v + d >= 0 && v + d < 256) acc += hist[v + d];
        if (acc > best) { best = acc; ry = v; }
    }
    std::vector<int> cs;
    for (int y = 0; y < H; ++y) for (int x = x0; x < x1; ++x) {
        if (!interior_col(x)) continue;
        const uint8_t* p = &s[((size_t)y * W + x) * 3];
        if (std::abs(luma(p) - ry) <= 8) cs.push_back(p[0] - p[2]);
    }
    int rc = 0;
    if (!cs.empty()) { std::nth_element(cs.begin(), cs.begin() + cs.size() / 2, cs.end()); rc = cs[cs.size() / 2]; }
    return {ry, rc};
}

inline int sobel(const std::vector<uint8_t>& s, int W, int x, int y) {
    auto L = [&](int xx, int yy) { return luma(&s[((size_t)yy * W + xx) * 3]); };
    int gx = (L(x + 1, y - 1) + 2 * L(x + 1, y) + L(x + 1, y + 1)) - (L(x - 1, y - 1) + 2 * L(x - 1, y) + L(x - 1, y + 1));
    int gy = (L(x - 1, y + 1) + 2 * L(x, y + 1) + L(x + 1, y + 1)) - (L(x - 1, y - 1) + 2 * L(x, y - 1) + L(x + 1, y - 1));
    return std::abs(gx) + std::abs(gy);
}

// 1 枚の帯画像を判定する。
//   bg   … 校正で作った「車の無い背景」の帯（無ければ nullptr）。あれば舗装かどうかを画素ごとに背景と比べ、
//          エッジも背景に元からある縁（白線・路面の文字）を除いて数える。無ければ車線ごとの舗装の色と比べる。
//   prev … Δ フレーム前の帯（無ければ nullptr）。あれば参照の動きも出す
inline Result analyse_strip(const std::vector<uint8_t>& s, const StripMap& m, int lanes, const Params& P,
                            const std::vector<uint8_t>* prev = nullptr, const std::vector<uint8_t>* bg = nullptr) {
    Result r;
    const int W = m.cols, H = m.rows;
    if (bg && bg->size() != s.size()) bg = nullptr;
    std::vector<std::array<int, 2>> rcol(lanes);
    if (!bg)
        for (int k = 0; k < lanes; ++k) rcol[k] = road_color_of(s, W, H, k * kColsPerLane, (k + 1) * kColsPerLane);
    r.road_color = bg ? std::array<int, 2>{-1, -1} : rcol[0];

    // B 占有率、E 赤、A エッジ
    r.nonroad.assign((size_t)W * H, 0);
    long inner = 0, nonroad = 0, red = 0, edges = 0, edge_n = 0;
    for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) {
        if (!interior_col(x)) continue;
        const uint8_t* p = &s[((size_t)y * W + x) * 3];
        int Y = luma(p), c = p[0] - p[2];
        int ry, rc;
        if (bg) { const uint8_t* q = &(*bg)[((size_t)y * W + x) * 3]; ry = luma(q); rc = q[0] - q[2]; }
        else { ry = rcol[x / kColsPerLane][0]; rc = rcol[x / kColsPerLane][1]; }
        bool nr = std::abs(Y - ry) > P.road_dy || std::abs(c - rc) > P.road_dc;
        r.nonroad[(size_t)y * W + x] = nr;
        ++inner; nonroad += nr;
        red += p[0] >= 140 && p[0] - p[1] >= 60 && p[0] - p[2] >= 60;
        if (y > 0 && y + 1 < H && x > 0 && x + 1 < W) {
            bool e = sobel(s, W, x, y) > 4 * P.edge_thr;
            if (e && bg && sobel(*bg, W, x, y) > 4 * P.edge_thr) e = false;  // 背景に元からある縁
            edges += e;
            ++edge_n;
        }
    }
    r.occ = inner ? (double)nonroad / inner : 0;
    r.red = inner ? (double)red / inner : 0;
    r.edge = edge_n ? (double)edges / edge_n : 0;

    // C 車間・D 台数（車線ごと）
    const double lane_len = (double)H / kColsPerLane;  // 帯の長さ（車線幅の倍数）
    std::vector<double> gaps_all;
    int veh_all = 0;
    for (int k = 0; k < lanes; ++k) {
        Lane ln;
        std::vector<uint8_t> occ(H, 0);
        for (int y = 0; y < H; ++y) {
            int c = 0, t = 0;
            for (int x = k * kColsPerLane; x < (k + 1) * kColsPerLane; ++x) if (interior_col(x)) { c += r.nonroad[(size_t)y * W + x]; ++t; }
            occ[y] = c > P.row_occ * t;
        }
        // 小さな穴を埋めてから、短い島を捨てる
        for (int y = 0; y < H;) {
            if (occ[y]) { ++y; continue; }
            int e = y; while (e < H && !occ[e]) ++e;
            if (y > 0 && e < H && e - y <= P.max_hole) std::fill(occ.begin() + y, occ.begin() + e, 1);
            y = e;
        }
        for (int y = 0; y < H;) {
            if (!occ[y]) { ++y; continue; }
            int e = y; while (e < H && occ[e]) ++e;
            if (e - y >= P.min_vehicle) ln.vehicles.push_back({y, e});
            y = e;
        }
        std::vector<double> g;
        for (size_t i = 1; i < ln.vehicles.size(); ++i) g.push_back((double)(ln.vehicles[i].first - ln.vehicles[i - 1].second) / kColsPerLane);
        if (ln.vehicles.size() <= 1) ln.gap = lane_len;  // 車が 1 台以下なら隙間は帯の長さ全部
        else ln.gap = median_of(g);
        ln.density = lane_len > 0 ? ln.vehicles.size() * 10.0 / lane_len : 0;
        for (double v : g) gaps_all.push_back(v);
        if (ln.vehicles.size() <= 1) gaps_all.push_back(lane_len);
        veh_all += (int)ln.vehicles.size();
        r.lanes.push_back(std::move(ln));
    }
    r.gap = median_of(gaps_all);
    r.density = lanes && lane_len > 0 ? veh_all * 10.0 / (lane_len * lanes) : 0;

    // R 参照の動き
    if (prev && prev->size() == s.size()) {
        long moved = 0;
        for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) {
            if (!interior_col(x)) continue;
            size_t k = (size_t)y * W + x;
            moved += std::abs(luma(&s[k * 3]) - luma(&(*prev)[k * 3])) > P.motion_thr;
        }
        r.motion = nonroad ? (double)moved / nonroad : 0;
        r.jam_ref = r.occ > P.jam_ref_occ && r.motion < P.jam_motion;
    }

    r.jam_edge = r.edge > P.jam_edge;
    r.jam_occ = r.occ > P.jam_occ;
    r.jam_gap = r.gap < P.jam_gap;
    r.jam_density = r.density > P.jam_density;
    r.jam_red = r.red > P.jam_red;
    r.votes = r.jam_edge + r.jam_occ + r.jam_gap + r.jam_density + r.jam_red;
    return r;
}

// ---------------------------------------------------------------------------
// 検出器の箱から数える方法（F・G）
//   F 検出台数密度 … 箱の下端の中央（タイヤの位置）が入る車線の台数を、車線 1 本・車線幅 10 本分の長さあたりに
//   G 車頭間隔     … 同じ車線で前後に並ぶ車の、帯の行での間隔の中央値（車線幅の倍数）。車が 1 台以下なら帯の長さ
// 帯の行に割り当てるので、遠近の補正は A〜E と同じように効く。トラックの背が隣の車線にはみ出しても、
// 下端で決めるのでずれにくい。
// ---------------------------------------------------------------------------
struct DetBox { float x1, y1, x2, y2, score; };

// 画像上の点を帯の (行, 横の割合 t) に。どの行の左右の端を結ぶ線分にも乗らなければ false
inline bool to_strip(const StripMap& m, double px, double py, int& row, double& t) {
    double best = 1e18; row = -1;
    for (int y = 0; y < m.rows; ++y) {
        const Pt& a = m.rowL[y]; const Pt& b = m.rowR[y];
        double dx = b.x - a.x, dy = b.y - a.y, L2 = dx * dx + dy * dy;
        if (L2 <= 0) continue;
        double u = ((px - a.x) * dx + (py - a.y) * dy) / L2;
        if (u < 0 || u > 1) continue;
        double d = std::fabs((px - a.x) * dy - (py - a.y) * dx) / std::sqrt(L2);  // 線分からの距離
        if (d < best) { best = d; row = y; t = u; }
    }
    // 行の間隔より離れていれば帯の外
    if (row < 0) return false;
    double step = row + 1 < m.rows ? std::hypot(m.rowL[row + 1].x - m.rowL[row].x, m.rowL[row + 1].y - m.rowL[row].y) : 2.0;
    return best <= std::max(2.0, step * 1.5);
}

struct DetResult {
    double density = 0, headway = NAN;
    bool jam_density = false, jam_headway = false;
    int count = 0;
    std::vector<std::vector<int>> lane_rows;  // 車線ごとの、車の行（手前から）
};

inline DetResult analyse_boxes(const StripMap& m, int lanes, const std::vector<DetBox>& boxes, double jam_density, double jam_headway) {
    DetResult r;
    r.lane_rows.assign(lanes, {});
    for (const DetBox& b : boxes) {
        int row; double t;
        if (!to_strip(m, (b.x1 + b.x2) / 2, b.y2, row, t)) continue;
        int k = std::clamp((int)(t * lanes), 0, lanes - 1);
        r.lane_rows[k].push_back(row);
        ++r.count;
    }
    const double lane_len = (double)m.rows / kColsPerLane;
    std::vector<double> hw;
    for (auto& v : r.lane_rows) {
        std::sort(v.begin(), v.end());
        if (v.size() <= 1) hw.push_back(lane_len);
        for (size_t i = 1; i < v.size(); ++i) hw.push_back((double)(v[i] - v[i - 1]) / kColsPerLane);
    }
    r.headway = median_of(hw);
    r.density = lanes && lane_len > 0 ? r.count * 10.0 / (lane_len * lanes) : 0;
    r.jam_density = r.density > jam_density;
    r.jam_headway = r.headway < jam_headway;
    return r;
}

// 領域の設定ファイル
//   region <名前> <車線数>
//   L x,y x,y ...   （左の縁、手前→奥）
//   R x,y x,y ...   （右の縁、手前→奥、L と同じ点数でなくてよい）
inline bool parse_regions(const std::string& text, std::vector<Region>& out, std::string& err) {
    out.clear();
    size_t pos = 0; int lineno = 0;
    while (pos < text.size()) {
        size_t e = text.find('\n', pos); if (e == std::string::npos) e = text.size();
        std::string line = text.substr(pos, e - pos); pos = e + 1; ++lineno;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        size_t h = line.find('#'); if (h != std::string::npos) line.resize(h);
        std::vector<std::string> tok; size_t i = 0;
        while (i < line.size()) {
            while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) ++i;
            size_t j = i; while (j < line.size() && line[j] != ' ' && line[j] != '\t') ++j;
            if (j > i) tok.push_back(line.substr(i, j - i));
            i = j;
        }
        if (tok.empty()) continue;
        if (tok[0] == "region" && tok.size() >= 3) { Region r; r.name = tok[1]; r.lanes = std::max(1, std::atoi(tok[2].c_str())); out.push_back(r); }
        else if ((tok[0] == "L" || tok[0] == "R") && !out.empty()) {
            auto& v = tok[0] == "L" ? out.back().left : out.back().right;
            for (size_t k = 1; k < tok.size(); ++k) {
                size_t c = tok[k].find(',');
                if (c == std::string::npos) { err = "line " + std::to_string(lineno) + ": x,y expected"; return false; }
                v.push_back({std::atof(tok[k].substr(0, c).c_str()), std::atof(tok[k].substr(c + 1).c_str())});
            }
        } else { err = "line " + std::to_string(lineno) + ": unknown '" + tok[0] + "'"; return false; }
    }
    for (auto& r : out) if (r.left.size() < 2 || r.right.size() < 2) { err = "region " + r.name + ": L and R need 2+ points"; return false; }
    if (out.empty()) { err = "no region"; return false; }
    return true;
}

}  // namespace tj
