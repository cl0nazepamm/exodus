// udimpacker - pack UDIM / multi-material UV islands and the texture pixels they own into one
// atlas, without rendering. Each island is cut out of its source maps by its own UV coverage and
// moved by a whole-pixel offset and a quarter-turn rotation, so every covered texel is copied
// bit-for-bit. Pixels no island owns are filled by bleeding the islands outward.
#include <algorithm>
#include <cctype>
#include <climits>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <map>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "bleed.h"
#include "decimate.h"
#include "image.h"
#include "pack.h"
#include "subdiv.h"
#include "udimpacker.h"

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

// ------------------------------------------------------------------------------------------------
// options

struct Options {
    std::string input, outDir, prefix, uvSet;
    int sizeW = 0, sizeH = 0;  // 0 = smallest power-of-two size that fits
    int maxSize = 16384;
    int padding = 8;
    int cell = 4;
    int nomapRes = 64;
    bool rotate = true, info = false, obj = true;
    std::vector<std::pair<std::string, double>> scales;  // material glob -> 1, 0.5, 0.25, ...
    std::vector<std::string> excludes;                   // material globs
    std::map<std::string, std::string> kinds;            // channel -> color | normal-gl | normal-dx
    struct Fill { std::string channel, glob; std::vector<double> values; };  // 0..255, last match wins
    std::vector<Fill> fills;
    struct MapOverride { std::string channel, glob, file; };
    std::vector<MapOverride> maps;
    std::string previewChannel = "DiffuseColor";
    // decimation: off unless --tris, --decimate or --max-error is given
    size_t tris = 0;        // triangle budget for the whole scene
    double keep = 0;        // fraction of triangles to keep
    double maxError = -1;   // relative to model size; < 0 = unbounded
    double normalWeight = 0.5, uvWeight = -1;  // uvWeight < 0 = automatic
    bool lockBorder = false;
    std::vector<std::string> noDecimate;       // material globs left at full detail
    // subdivision: runs before decimation
    int unsubdivide = 0;    // Catmull-Clark levels to reverse
    bool cage = false;      // recover the control cage positions instead of keeping the smoothed ones
    bool unsubExact = false;  // refuse meshes that are not a subdivision result instead of coarsening them
    int subdivide = 0;      // Catmull-Clark levels to apply (utility / round-trip testing)
};

static void usage() {
    std::puts(
        "udimpacker <scene.fbx|scene.obj> -o <out-dir> [options]\n"
        "\n"
        "  --info                        list meshes, materials, maps and UDIM tiles, then exit\n"
        "  --size N | WxH                atlas size (default: smallest power of two that fits)\n"
        "  --max-size N                  upper bound for the automatic size (default 16384)\n"
        "  --padding N                   minimum gap between islands in atlas pixels (default 8)\n"
        "  --cell N                      packing grid cell in pixels (default 4)\n"
        "  --no-rotate                   keep every island upright\n"
        "  --scale <material-glob>=S     S = 1, 0.5, 0.25, 0.125 (2x box downsample per step)\n"
        "  --exclude <material-glob>     drop a material's faces from the result\n"
        "  --map <channel>:<glob>=<file> bind or replace a map for matching materials\n"
        "  --kind <channel>=K            K = color (default), normal-gl, normal-dx\n"
        "  --fill <channel>[:<glob>]=r,g,b[,a]\n"
        "                                value (0-255) for islands whose material lacks this map\n"
        "  --unsubdivide N               reverse N Catmull-Clark subdivision levels (before decimation);\n"
        "                                exact where the mesh is a subdivision result, otherwise every\n"
        "                                other vertex is dissolved twice per level (Blender's un-subdivide);\n"
        "                                vertices keep their positions, UVs and normals (seams stay seams)\n"
        "  --unsubdivide-exact           refuse meshes that are not a subdivision result instead\n"
        "  --cage                        with --unsubdivide: recover the control cage positions\n"
        "                                (least squares) instead of keeping the smoothed ones\n"
        "  --subdivide N                 apply N Catmull-Clark levels (positions, UVs, normals)\n"
        "  --tris N                      decimate the whole scene to about N triangles\n"
        "  --decimate R                  decimate to fraction R of the triangles (e.g. 0.25)\n"
        "  --max-error E                 decimation error limit relative to model size (0.01 = 1%);\n"
        "                                alone it decimates as far as E allows\n"
        "  --no-decimate <material-glob> leave a material at full detail (budget still counts it)\n"
        "  --lock-border                 keep open mesh borders fixed while decimating\n"
        "  --normal-weight W             decimation weight of normals (default 0.5)\n"
        "  --uv-weight W                 decimation weight of UVs (default 1/sqrt(mean UV triangle area))\n"
        "  --uvset <name>                UV set to pack (default: first)\n"
        "  --prefix <name>               output file prefix (default: input file stem)\n"
        "  --preview-channel <channel>   map used by the preview OBJ material (default DiffuseColor)\n"
        "  --no-obj                      skip the preview OBJ\n");
}

static bool glob_match(const std::string& pat, const std::string& s) {
    size_t p = 0, i = 0, star = std::string::npos, mark = 0;
    auto eq = [](char a, char b) { return std::tolower((unsigned char)a) == std::tolower((unsigned char)b); };
    while (i < s.size()) {
        if (p < pat.size() && (pat[p] == '?' || eq(pat[p], s[i]))) { ++p; ++i; }
        else if (p < pat.size() && pat[p] == '*') { star = p++; mark = i; }
        else if (star != std::string::npos) { p = star + 1; i = ++mark; }
        else return false;
    }
    while (p < pat.size() && pat[p] == '*') ++p;
    return p == pat.size();
}

static bool split_kv(const std::string& s, std::string& k, std::string& v) {
    size_t eq = s.find('=');
    if (eq == std::string::npos) return false;
    k = s.substr(0, eq);
    v = s.substr(eq + 1);
    return true;
}

static bool parse_args(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) { std::fprintf(stderr, "missing value for %s\n", a.c_str()); std::exit(2); }
            return argv[++i];
        };
        std::string k, v;
        if (a == "-h" || a == "--help") { usage(); std::exit(0); }
        else if (a == "-o" || a == "--out") o.outDir = next();
        else if (a == "--info") o.info = true;
        else if (a == "--size") {
            std::string s = next();
            size_t x = s.find_first_of("xX");
            o.sizeW = std::stoi(s.substr(0, x));
            o.sizeH = x == std::string::npos ? o.sizeW : std::stoi(s.substr(x + 1));
        }
        else if (a == "--max-size") o.maxSize = std::stoi(next());
        else if (a == "--padding") o.padding = std::stoi(next());
        else if (a == "--cell") o.cell = std::stoi(next());
        else if (a == "--no-rotate") o.rotate = false;
        else if (a == "--no-obj") o.obj = false;
        else if (a == "--uvset") o.uvSet = next();
        else if (a == "--prefix") o.prefix = next();
        else if (a == "--preview-channel") o.previewChannel = next();
        else if (a == "--exclude") o.excludes.push_back(next());
        else if (a == "--unsubdivide") o.unsubdivide = std::stoi(next());
        else if (a == "--cage") o.cage = true;
        else if (a == "--unsubdivide-exact") o.unsubExact = true;
        else if (a == "--subdivide") o.subdivide = std::stoi(next());
        else if (a == "--tris") o.tris = std::stoull(next());
        else if (a == "--decimate") {
            o.keep = std::stod(next());
            if (o.keep <= 0 || o.keep > 1) { std::fprintf(stderr, "--decimate expects a fraction in (0, 1]\n"); return false; }
        }
        else if (a == "--max-error") o.maxError = std::stod(next());
        else if (a == "--lock-border") o.lockBorder = true;
        else if (a == "--no-decimate") o.noDecimate.push_back(next());
        else if (a == "--normal-weight") o.normalWeight = std::stod(next());
        else if (a == "--uv-weight") o.uvWeight = std::stod(next());
        else if (a == "--scale" && split_kv(next(), k, v)) o.scales.push_back({k, std::stod(v)});
        else if (a == "--kind" && split_kv(next(), k, v)) o.kinds[k] = v;
        else if (a == "--fill" && split_kv(next(), k, v)) {
            std::vector<double> vals;
            for (size_t p = 0; p <= v.size();) {
                size_t c = v.find(',', p);
                if (c == std::string::npos) c = v.size();
                vals.push_back(std::stod(v.substr(p, c - p)));
                p = c + 1;
            }
            size_t colon = k.find(':');
            o.fills.push_back({k.substr(0, colon), colon == std::string::npos ? "" : k.substr(colon + 1), vals});
        }
        else if (a == "--map" && split_kv(next(), k, v)) {
            size_t colon = k.find(':');
            if (colon == std::string::npos) { std::fprintf(stderr, "--map expects <channel>:<glob>=<file>\n"); return false; }
            o.maps.push_back({k.substr(0, colon), k.substr(colon + 1), v});
        }
        else if (!a.empty() && a[0] == '-') { std::fprintf(stderr, "unknown or malformed option %s\n", a.c_str()); return false; }
        else if (o.input.empty()) o.input = a;
        else { std::fprintf(stderr, "unexpected argument %s\n", a.c_str()); return false; }
    }
    if (o.input.empty() || (!o.info && o.outDir.empty())) { usage(); return false; }
    if (o.prefix.empty()) o.prefix = ustr(upath(o.input).stem());
    return true;
}

static void say(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    std::vprintf(fmt, ap);
    va_end(ap);
    std::fflush(stdout);
}

// ------------------------------------------------------------------------------------------------
// working data

enum Kind { KIND_COLOR = 0, KIND_NORMAL_GL = 1, KIND_NORMAL_DX = 2 };

struct Channel {
    std::string name;
    int kind = KIND_COLOR;
    int c = 1, bps = 1;  // atlas format: widest of the bound maps
};

struct TexEntry {  // one distinct image
    std::string id, path, label;
    const std::vector<uint8_t>* embedded = nullptr;
    ImageInfo info;
};

struct Source {  // one set of maps islands are cut from (one per distinct binding)
    std::vector<int> tex;  // per channel: TexEntry index or -1
    std::vector<uint32_t> materials;
    bool nomap = false;
    int shift = 0;          // 2^shift box downsample
    int baseW = 0, baseH = 0;
    int W = 0, H = 0;       // pixel grid islands are cut from
};

struct TriRef { uint32_t mesh, tri; };

struct Island {
    int source = 0;
    int bx = 0, by = 0, w = 0, h = 0;  // bbox in source pixels (unwrapped, may be negative)
    std::vector<uint8_t> mask;         // w x h coverage
    std::vector<TriRef> tris;
    int rot = 0, ox = 0, oy = 0;       // placement: stamp origin in atlas pixels
};

struct Tile { int u = 0, v = 0; };

static int wrap(int a, int n) { a %= n; return a < 0 ? a + n : a; }

static int udim_number(Tile t) { return 1001 + t.u + 10 * t.v; }

// "Body_1001.png" -> "Body_1002.png" when a shell sits in 1002 and that file exists;
// "<UDIM>" tokens are always substituted.
static std::string tile_path(const std::string& path, Tile t) {
    if (path.empty()) return path;
    const std::string num = std::to_string(udim_number(t));
    size_t tok = path.find("<UDIM>");
    if (tok != std::string::npos) return path.substr(0, tok) + num + path.substr(tok + 6);
    fs::path p = upath(path);
    std::string stem = ustr(p.stem());
    if (stem.size() < 5) return path;
    std::string tail = stem.substr(stem.size() - 4);
    char sep = stem[stem.size() - 5];
    if ((sep != '_' && sep != '.') || tail[0] != '1' || !std::all_of(tail.begin(), tail.end(), ::isdigit)) return path;
    if (tail == num) return path;
    fs::path cand = p.parent_path() / upath(stem.substr(0, stem.size() - 4) + num + ustr(p.extension()));
    std::error_code ec;
    return fs::is_regular_file(cand, ec) ? ustr(cand) : path;
}

// Quarter-turn rotation of a tangent-space normal stored in RG. Clockwise image-space turns
// rotate the UV frame by -90 degrees each, and the XY of the normal must turn with it.
static void swizzle_normal(int kind, int rot, uint32_t v[4], uint32_t maxv) {
    if (kind == KIND_COLOR || (rot & 3) == 0) return;
    uint32_t r = v[0], g = v[1];
    int k = rot & 3;
    if (kind == KIND_NORMAL_DX && k != 2) k = 4 - k;  // DX stores -Y: the turn runs the other way
    switch (k) {
        case 1: v[0] = g; v[1] = maxv - r; break;
        case 2: v[0] = maxv - r; v[1] = maxv - g; break;
        case 3: v[0] = maxv - g; v[1] = r; break;
    }
}

// Conservative rasterization: every pixel whose square touches the triangle.
template <class F>
static void raster_tri(const Vec2d p[3], int gw, int gh, F&& mark) {
    double minx = std::min({p[0].x, p[1].x, p[2].x}), maxx = std::max({p[0].x, p[1].x, p[2].x});
    double miny = std::min({p[0].y, p[1].y, p[2].y}), maxy = std::max({p[0].y, p[1].y, p[2].y});
    int x0 = std::max(0, (int)std::floor(minx)), x1 = std::min(gw - 1, (int)std::floor(maxx));
    int y0 = std::max(0, (int)std::floor(miny)), y1 = std::min(gh - 1, (int)std::floor(maxy));
    double area = (p[1].x - p[0].x) * (p[2].y - p[0].y) - (p[1].y - p[0].y) * (p[2].x - p[0].x);
    double s = area >= 0 ? 1.0 : -1.0;
    double A[3], B[3], C[3], R[3];
    for (int e = 0; e < 3; ++e) {
        const Vec2d& a = p[e];
        const Vec2d& b = p[(e + 1) % 3];
        A[e] = -s * (b.y - a.y);
        B[e] = s * (b.x - a.x);
        C[e] = -(A[e] * a.x + B[e] * a.y);
        R[e] = 0.5 * (std::abs(A[e]) + std::abs(B[e]));  // reach of a half-pixel square
    }
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) {
            double cx = x + 0.5, cy = y + 0.5;
            bool in = true;
            for (int e = 0; e < 3 && in; ++e) in = A[e] * cx + B[e] * cy + C[e] + R[e] >= -1e-9;
            if (in) mark(x, y);
        }
}

static std::string json_str(const std::string& s) {
    std::string o = "\"";
    for (unsigned char ch : s) {
        if (ch == '"' || ch == '\\') { o += '\\'; o += (char)ch; }
        else if (ch < 0x20) { char b[8]; std::snprintf(b, sizeof b, "\\u%04x", ch); o += b; }
        else o += (char)ch;
    }
    return o + "\"";
}

static std::string safe_name(const std::string& s) {
    std::string o;
    for (unsigned char ch : s) o += (std::isalnum(ch) || ch == '-' || ch == '_') ? (char)ch : '_';
    return o;
}

// ------------------------------------------------------------------------------------------------

struct Packer {
    Options o;
    InScene scene;
    std::vector<Channel> channels;
    std::vector<TexEntry> texEntries;
    std::vector<Source> sources;
    std::vector<Island> islands;
    std::vector<uint8_t> excluded;                        // per material
    std::vector<std::vector<int>> faceSource;             // per mesh, per face (-1 = excluded)
    std::vector<std::vector<Tile>> faceTile;              // per mesh, per face
    std::vector<std::vector<int>> faceIsland;             // per mesh, per face
    int AW = 0, AH = 0, pad = 0;
    bool decimated = false;
    DecimateResult dec;
    bool unsubdivided = false;
    SubdivReport sub;

    double scale_for(const std::string& material) const {
        double s = 1.0;
        for (const auto& [g, v] : o.scales)
            if (glob_match(g, material)) s = v;
        return s;
    }

    int tex_entry(const InTexture& t, Tile tile) {
        std::string path = tile_path(t.path, tile);
        std::string id = path.empty() ? "embedded:" + t.label : path;
        for (size_t i = 0; i < texEntries.size(); ++i)
            if (texEntries[i].id == id) return (int)i;
        TexEntry e;
        e.id = id;
        e.path = path;
        e.label = path.empty() ? t.label : ustr(upath(path).filename());
        if (path.empty()) e.embedded = t.embedded.empty() ? nullptr : &t.embedded;
        std::vector<uint8_t> bytes;
        bool ok = e.embedded ? image_info(*e.embedded, e.info)
                             : (!path.empty() && read_file(path, bytes) && image_info(bytes, e.info));
        if (!ok) {
            say("  ! cannot read map '%s' (%s) - treated as missing\n", t.label.c_str(), path.c_str());
            return -1;
        }
        texEntries.push_back(std::move(e));
        return (int)texEntries.size() - 1;
    }

    bool load_image(int entry, Image& img) {
        const TexEntry& e = texEntries[entry];
        std::vector<uint8_t> bytes;
        const std::vector<uint8_t>* src = e.embedded;
        if (!src) {
            if (!read_file(e.path, bytes)) { say("  ! cannot read %s\n", e.path.c_str()); return false; }
            src = &bytes;
        }
        std::string err;
        if (!image_decode(*src, img, err)) { say("  ! cannot decode %s: %s\n", e.label.c_str(), err.c_str()); return false; }
        return true;
    }

    // Source image for one channel at the source's cutting resolution.
    bool source_image(const Source& s, int ch, Image& out) {
        if (!load_image(s.tex[ch], out)) return false;
        if (out.w != s.baseW || out.h != s.baseH) out = upscale_nearest(out, s.baseW / out.w, s.baseH / out.h);
        for (int i = 0; i < s.shift; ++i) out = downsample2(out);
        return true;
    }

    std::vector<uint8_t> excluded_materials() const {
        std::vector<uint8_t> ex(scene.materials.size(), 0);
        for (size_t i = 0; i < scene.materials.size(); ++i)
            for (const std::string& g : o.excludes)
                if (glob_match(g, scene.materials[i].name)) ex[i] = 1;
        return ex;
    }

    // ---- 0a. subdivision (optional) -----------------------------------------------------------
    // Reverse subdivision runs first, so decimation and the atlas both see the coarse mesh.
    void subdivision() {
        if (o.subdivide > 0) {
            say("subdividing (%d level%s)\n", o.subdivide, o.subdivide == 1 ? "" : "s");
            const SubdivReport r = subdivide_scene(scene, o.subdivide);
            for (const SubdivReport::Mesh& m : r.meshes) say("  %-40s faces %7zu -> %7zu\n", m.name.c_str(), m.facesBefore, m.facesAfter);
            say("  total %zu -> %zu faces (%zu -> %zu triangles), %.0f ms\n", r.facesBefore, r.facesAfter, r.trisBefore, r.trisAfter, r.ms);
        }
        if (o.unsubdivide > 0) {
            say("reversing subdivision (%d level%s%s)\n", o.unsubdivide, o.unsubdivide == 1 ? "" : "s", o.cage ? ", solving the cage" : "");
            UnsubdivideOptions u;
            u.levels = o.unsubdivide;
            u.solveCage = o.cage;
            u.exactOnly = o.unsubExact;
            u.keep.assign(scene.materials.size(), 0);
            for (const std::string& g : o.noDecimate)
                for (size_t i = 0; i < scene.materials.size(); ++i)
                    if (glob_match(g, scene.materials[i].name)) u.keep[i] = 1;
            sub = unsubdivide_scene(scene, u);
            unsubdivided = true;
            for (const SubdivReport::Mesh& m : sub.meshes) {
                say("  %-40s faces %7zu -> %7zu ", m.name.c_str(), m.facesBefore, m.facesAfter);
                if (m.levels) say(" %d exact", m.levels);
                if (m.approxLevels)
                    say(" %d approx (%zu vertices dissolved in %zu passes, %zu triangle pairs joined)", m.approxLevels,
                        m.dissolved, m.passes, m.joined);
                if (!m.levels && !m.approxLevels) say(" unchanged");
                if (m.levels + m.approxLevels < o.unsubdivide) {
                    if (!m.stop.empty()) say("  ! stopped: %s", m.stop.c_str());
                    else if (m.keptFaces == m.facesBefore || (m.levels && m.keptFaces == m.facesAfter))
                        say("  ! %s: %s", m.levels ? "stopped early" : "not a subdivision result", m.reason.c_str());
                    else
                        say("  ! %zu faces in %zu face set%s left as is: %s", m.keptFaces, m.keptSets, m.keptSets == 1 ? "" : "s", m.reason.c_str());
                } else if (m.approxLevels && !m.reason.empty())
                    say("  (not a subdivision result: %s)", m.reason.c_str());
                if (o.cage && m.levels) say("  cage fit rms %.3g%% max %.3g%%", m.fitRms * 100, m.fitMax * 100);
                say("\n");
            }
            say("  total %zu -> %zu faces (%zu -> %zu triangles), %.0f ms\n", sub.facesBefore, sub.facesAfter, sub.trisBefore, sub.trisAfter, sub.ms);
            if (sub.materialMismatch) say("  ! %zu rebuilt faces had sub-quads of different materials (first one used)\n", sub.materialMismatch);
        }
    }

    // ---- 0b. decimation (optional) ------------------------------------------------------------
    // Runs before anything is bound, so islands, the atlas and the verify pass all use the game mesh.
    bool decimating() const { return o.tris || o.keep > 0 || o.maxError >= 0; }

    void decimate() {
        const std::vector<uint8_t> drop = excluded_materials();
        size_t input = 0;
        for (const InMesh& m : scene.meshes)
            for (uint32_t f : m.triFace) input += !drop[m.faces[f].material];
        DecimateOptions d;
        d.targetTris = o.tris ? o.tris : (size_t)std::llround((double)input * o.keep);  // 0 = error-limited
        d.maxError = o.maxError;
        d.normalWeight = o.normalWeight;
        d.uvWeight = o.uvWeight;
        d.lockBorder = o.lockBorder;
        d.keep.assign(scene.materials.size(), 0);
        for (const std::string& g : o.noDecimate) {
            int hits = 0;
            for (size_t i = 0; i < scene.materials.size(); ++i)
                if (glob_match(g, scene.materials[i].name)) { d.keep[i] = 1; ++hits; }
            if (!hits) say("  ! --no-decimate %s matched no material\n", g.c_str());
        }
        char limit[32] = "none";
        if (d.maxError >= 0) std::snprintf(limit, sizeof limit, "%g%%", d.maxError * 100);
        say("  %zu triangles in, target %s, error limit %s%s\n", input,
            d.targetTris ? std::to_string(d.targetTris).c_str() : "none", limit, d.lockBorder ? ", borders locked" : "");

        auto t0 = Clock::now();
        dec = decimate_scene(scene, drop, d);
        decimated = true;
        for (const DecimateResult::Mesh& m : dec.meshes)
            if (m.before) say("  %-40s tris %7zu -> %7zu\n", m.name.c_str(), m.before, m.after);
        say("  total %zu -> %zu triangles (%zu simplifier vertices), error %.3g%% of model size, uv weight %.3g, %.0f ms\n",
            dec.before, dec.after, dec.vertices, dec.error * 100, dec.uvWeight,
            std::chrono::duration<double, std::milli>(Clock::now() - t0).count());
        if (d.targetTris && dec.after > d.targetTris + d.targetTris / 50)
            say("  ! stopped above the target: %s\n", d.maxError >= 0 ? "error limit reached, or seams/borders/small parts"
                                                                       : "seams, borders or small separate parts cannot collapse further");
        if (dec.flaps) say("  removed %zu back-to-back triangle pair%s left by collapses\n", dec.flaps, dec.flaps == 1 ? "" : "s");
        if (dec.mixedTris) say("  ! %zu triangles mix meshes or materials\n", dec.mixedTris);
    }

    // ---- 1. bindings ------------------------------------------------------------------------
    bool bind() {
        for (const auto& m : o.maps) {
            int hits = 0;
            for (InMaterial& mat : scene.materials) {
                if (!glob_match(m.glob, mat.name)) continue;
                auto it = std::find_if(mat.textures.begin(), mat.textures.end(),
                                       [&](const InTexture& t) { return t.channel == m.channel; });
                InTexture t;
                t.channel = m.channel;
                t.path = ustr(fs::absolute(upath(m.file)));
                t.label = ustr(upath(m.file).filename());
                if (it != mat.textures.end()) *it = t; else mat.textures.push_back(t);
                ++hits;
            }
            if (!hits) say("  ! --map %s:%s matched no material\n", m.channel.c_str(), m.glob.c_str());
        }

        excluded = excluded_materials();

        std::set<std::string> names;
        for (size_t i = 0; i < scene.materials.size(); ++i)
            if (!excluded[i])
                for (const InTexture& t : scene.materials[i].textures) names.insert(t.channel);
        for (const std::string& n : names) {
            Channel c;
            c.name = n;
            auto k = o.kinds.find(n);
            if (k != o.kinds.end()) {
                if (k->second == "normal-gl") c.kind = KIND_NORMAL_GL;
                else if (k->second == "normal-dx") c.kind = KIND_NORMAL_DX;
                else if (k->second != "color") { say("unknown kind %s\n", k->second.c_str()); return false; }
            } else if (o.rotate) {
                std::string low = n;
                for (char& ch : low) ch = (char)std::tolower((unsigned char)ch);
                if (low.find("normal") != std::string::npos)
                    say("  ! channel '%s' looks like a normal map; pass --kind %s=normal-gl|normal-dx "
                        "so rotated islands keep correct tangents\n", n.c_str(), n.c_str());
            }
            channels.push_back(c);
        }
        if (channels.empty()) { say("no maps bound to any material\n"); return false; }

        // Face -> tile -> source.
        std::map<std::string, int> sourceByKey;
        faceSource.resize(scene.meshes.size());
        faceTile.resize(scene.meshes.size());
        for (size_t mi = 0; mi < scene.meshes.size(); ++mi) {
            const InMesh& mesh = scene.meshes[mi];
            faceSource[mi].assign(mesh.faces.size(), -1);
            faceTile[mi].resize(mesh.faces.size());
            for (size_t fi = 0; fi < mesh.faces.size(); ++fi) {
                const InFace& f = mesh.faces[fi];
                if (excluded[f.material]) continue;
                double cu = 0, cv = 0;
                for (uint32_t c = 0; c < f.count; ++c) {
                    const Vec2d& uv = mesh.uvs[mesh.cornerUv[f.first + c]];
                    cu += uv.x; cv += uv.y;
                }
                Tile t{(int)std::floor(cu / f.count), (int)std::floor(cv / f.count)};
                faceTile[mi][fi] = t;

                const InMaterial& mat = scene.materials[f.material];
                std::vector<int> tex(channels.size(), -1);
                std::string key;
                bool any = false;
                for (size_t ch = 0; ch < channels.size(); ++ch) {
                    for (const InTexture& it : mat.textures)
                        if (it.channel == channels[ch].name) { tex[ch] = tex_entry(it, t); break; }
                    any |= tex[ch] >= 0;
                    key += std::to_string(tex[ch]) + "|";
                }
                if (!any) key = "nomap:" + std::to_string(f.material);
                auto it = sourceByKey.find(key);
                int sid;
                if (it == sourceByKey.end()) {
                    sid = (int)sources.size();
                    sourceByKey[key] = sid;
                    Source s;
                    s.tex = tex;
                    s.nomap = !any;
                    sources.push_back(s);
                } else sid = it->second;
                Source& s = sources[sid];
                if (std::find(s.materials.begin(), s.materials.end(), f.material) == s.materials.end())
                    s.materials.push_back(f.material);
                faceSource[mi][fi] = sid;
            }
        }

        // Source resolution and scale.
        for (Source& s : sources) {
            if (s.nomap) { s.baseW = s.baseH = s.W = s.H = o.nomapRes; continue; }
            for (size_t ch = 0; ch < channels.size(); ++ch) {
                if (s.tex[ch] < 0) continue;
                const ImageInfo& in = texEntries[s.tex[ch]].info;
                s.baseW = std::max(s.baseW, in.w);
                s.baseH = std::max(s.baseH, in.h);
                channels[ch].c = std::max(channels[ch].c, in.c);
                channels[ch].bps = std::max(channels[ch].bps, in.bps);
            }
            for (size_t ch = 0; ch < channels.size(); ++ch) {
                if (s.tex[ch] < 0) continue;
                const TexEntry& e = texEntries[s.tex[ch]];
                if (s.baseW % e.info.w || s.baseH % e.info.h) {
                    say("maps of one material must share a resolution (or divide it): %s is %dx%d, set is %dx%d\n",
                        e.label.c_str(), e.info.w, e.info.h, s.baseW, s.baseH);
                    return false;
                }
                if (e.info.w != s.baseW || e.info.h != s.baseH)
                    say("  ! %s is %dx%d, replicated to %dx%d to match its set\n",
                        e.label.c_str(), e.info.w, e.info.h, s.baseW, s.baseH);
            }
            double sc = 0;
            for (uint32_t m : s.materials) sc = std::max(sc, scale_for(scene.materials[m].name));
            int shift = (int)std::lround(-std::log2(sc));
            if (sc <= 0 || shift < 0 || shift > 6 || std::abs(std::ldexp(1.0, -shift) - sc) > 1e-9) {
                say("scale %g is not 1, 0.5, 0.25, ... (needed for a clean box downsample)\n", sc);
                return false;
            }
            if (s.baseW % (1 << shift) || s.baseH % (1 << shift)) {
                say("%dx%d does not divide by %d for scale %g\n", s.baseW, s.baseH, 1 << shift, sc);
                return false;
            }
            s.shift = shift;
            s.W = s.baseW >> shift;
            s.H = s.baseH >> shift;
        }
        return true;
    }

    // ---- 2. islands: UV coverage in source pixel space, split into connected blobs -----------
    Vec2d src_px(const Source& s, const Vec2d& uv, Tile t) const {
        return {(uv.x - t.u) * s.W, (1.0 - (uv.y - t.v)) * s.H};
    }

    void build_islands() {
        faceIsland.resize(scene.meshes.size());
        for (size_t mi = 0; mi < scene.meshes.size(); ++mi) faceIsland[mi].assign(scene.meshes[mi].faces.size(), -1);

        std::vector<std::vector<TriRef>> bySource(sources.size());
        for (size_t mi = 0; mi < scene.meshes.size(); ++mi) {
            const InMesh& mesh = scene.meshes[mi];
            for (size_t ti = 0; ti < mesh.tris.size(); ++ti) {
                int sid = faceSource[mi][mesh.triFace[ti]];
                if (sid >= 0) bySource[sid].push_back({(uint32_t)mi, (uint32_t)ti});
            }
        }

        size_t splitFaces = 0;
        for (size_t sid = 0; sid < sources.size(); ++sid) {
            const Source& s = sources[sid];
            auto tri_px = [&](const TriRef& r, Vec2d p[3]) {
                const InMesh& mesh = scene.meshes[r.mesh];
                Tile t = faceTile[r.mesh][mesh.triFace[r.tri]];
                for (int k = 0; k < 3; ++k) p[k] = src_px(s, mesh.uvs[mesh.cornerUv[mesh.tris[r.tri][k]]], t);
            };

            double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
            for (const TriRef& r : bySource[sid]) {
                Vec2d p[3];
                tri_px(r, p);
                for (auto& q : p) {
                    minx = std::min(minx, q.x); maxx = std::max(maxx, q.x);
                    miny = std::min(miny, q.y); maxy = std::max(maxy, q.y);
                }
            }
            if (bySource[sid].empty()) continue;
            const int gx = (int)std::floor(minx) - 1, gy = (int)std::floor(miny) - 1;
            const int gw = (int)std::floor(maxx) - gx + 2, gh = (int)std::floor(maxy) - gy + 2;

            std::vector<int32_t> label((size_t)gw * gh, 0);  // 0 empty, -1 covered, >0 island+1
            for (const TriRef& r : bySource[sid]) {
                Vec2d p[3];
                tri_px(r, p);
                for (auto& q : p) { q.x -= gx; q.y -= gy; }
                raster_tri(p, gw, gh, [&](int x, int y) { label[(size_t)y * gw + x] = -1; });
            }

            // 8-connected components.
            const int first = (int)islands.size();
            std::vector<uint32_t> stack;
            for (size_t i = 0; i < label.size(); ++i) {
                if (label[i] != -1) continue;
                const int id = (int)islands.size();
                Island isl;
                isl.source = (int)sid;
                int x0 = INT_MAX, y0 = INT_MAX, x1 = INT_MIN, y1 = INT_MIN;
                label[i] = id + 1;
                stack.push_back((uint32_t)i);
                while (!stack.empty()) {
                    uint32_t c = stack.back();
                    stack.pop_back();
                    int x = (int)(c % gw), y = (int)(c / gw);
                    x0 = std::min(x0, x); x1 = std::max(x1, x);
                    y0 = std::min(y0, y); y1 = std::max(y1, y);
                    for (int dy = -1; dy <= 1; ++dy)
                        for (int dx = -1; dx <= 1; ++dx) {
                            int nx = x + dx, ny = y + dy;
                            if (nx < 0 || ny < 0 || nx >= gw || ny >= gh) continue;
                            size_t j = (size_t)ny * gw + nx;
                            if (label[j] == -1) { label[j] = id + 1; stack.push_back((uint32_t)j); }
                        }
                }
                isl.bx = gx + x0;
                isl.by = gy + y0;
                isl.w = x1 - x0 + 1;
                isl.h = y1 - y0 + 1;
                isl.mask.assign((size_t)isl.w * isl.h, 0);
                for (int y = y0; y <= y1; ++y)
                    for (int x = x0; x <= x1; ++x)
                        if (label[(size_t)y * gw + x] == id + 1) isl.mask[(size_t)(y - y0) * isl.w + (x - x0)] = 1;
                islands.push_back(std::move(isl));
            }

            // Each triangle belongs to the blob under its first corner (that pixel always touches it).
            for (const TriRef& r : bySource[sid]) {
                Vec2d p[3];
                tri_px(r, p);
                int x = (int)std::floor(p[0].x) - gx, y = (int)std::floor(p[0].y) - gy;
                int id = label[(size_t)y * gw + x] - 1;
                if (id < first) continue;  // cannot happen: the corner pixel is always covered
                islands[id].tris.push_back(r);
                const InMesh& mesh = scene.meshes[r.mesh];
                int& fi = faceIsland[r.mesh][mesh.triFace[r.tri]];
                if (fi >= 0 && fi != id) ++splitFaces;
                fi = id;
            }
        }
        if (splitFaces) say("  ! %zu faces span two islands (degenerate UVs?)\n", splitFaces);
    }

    // ---- 3. pack -------------------------------------------------------------------------------
    bool pack() {
        pad = (o.padding + 1) / 2;
        std::vector<PackItem> items;
        items.reserve(islands.size());
        double need = 0;
        for (const Island& isl : islands) {
            items.push_back(build_item(isl.mask, isl.w, isl.h, pad, o.cell));
            need += (double)items.back().rot[0].cells * o.cell * o.cell;
        }

        std::vector<std::pair<int, int>> sizes;
        if (o.sizeW) sizes.push_back({o.sizeW, o.sizeH});
        else {
            for (int w = 256; w <= o.maxSize; w *= 2) {
                sizes.push_back({w, w});
                if (w * 2 <= o.maxSize) sizes.push_back({w * 2, w});
            }
            std::stable_sort(sizes.begin(), sizes.end(), [](auto a, auto b) {
                return (double)a.first * a.second < (double)b.first * b.second;
            });
        }
        say("  footprint area %.1f Mpx (islands + padding)\n", need / 1e6);

        for (auto [w, h] : sizes) {
            if ((double)w * h < need) continue;
            if (w % o.cell || h % o.cell) { say("atlas size must divide by --cell\n"); return false; }
            auto t0 = Clock::now();
            std::vector<Placement> pl;
            int passes = 0;
            bool ok = pack_items(items, w / o.cell, h / o.cell, o.rotate, pl, &passes);
            double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            say("  try %dx%d: %s after %d pass%s (%.0f ms)\n", w, h, ok ? "fits" : "no fit", passes,
                passes == 1 ? "" : "es", ms);
            if (!ok) continue;
            AW = w; AH = h;
            for (size_t i = 0; i < islands.size(); ++i) {
                islands[i].rot = pl[i].rot;
                islands[i].ox = pl[i].x * o.cell + pad;
                islands[i].oy = pl[i].y * o.cell + pad;
            }
            return true;
        }
        say("islands do not fit (largest try %dx%d); raise --max-size / --size or shrink with --scale\n",
            sizes.empty() ? 0 : sizes.back().first, sizes.empty() ? 0 : sizes.back().second);
        return false;
    }

    // New UV of a tile-local source UV. Same math as the pixel copy, in continuous coordinates.
    Vec2d new_uv(const Island& isl, const Vec2d& uv, Tile t) const {
        const Source& s = sources[isl.source];
        Vec2d p = src_px(s, uv, t);
        double rx, ry;
        rot_pt(isl.rot, isl.w, isl.h, p.x - isl.bx, p.y - isl.by, rx, ry);
        return {(isl.ox + rx) / AW, 1.0 - (isl.oy + ry) / AH};
    }

    void dest_px(const Island& isl, int lx, int ly, int& X, int& Y) const {
        rot_px(isl.rot, isl.w, isl.h, lx, ly, X, Y);
        X += isl.ox;
        Y += isl.oy;
    }

    const Vec4d* material_constant(const Source& s, const std::string& channel) const {
        for (uint32_t m : s.materials)
            for (const auto& [name, c] : scene.materials[m].constants)
                if (name == channel) return &c;
        return nullptr;
    }

    void fill_value(const Source& s, const Channel& ch, uint32_t out[4]) const {
        const double maxv = ch.bps == 2 ? 65535.0 : 255.0;
        double v[4] = {0, 0, 0, 1};
        const Options::Fill* fill = nullptr;
        for (const Options::Fill& f : o.fills) {
            if (f.channel != ch.name) continue;
            bool hit = f.glob.empty();
            for (uint32_t m : s.materials) hit |= glob_match(f.glob, scene.materials[m].name);
            if (hit) fill = &f;
        }
        if (fill) {
            for (size_t i = 0; i < 4 && i < fill->values.size(); ++i) v[i] = fill->values[i] / 255.0;
            if (fill->values.size() == 1) v[1] = v[2] = v[0];
        } else if (const Vec4d* c = material_constant(s, ch.name)) {
            v[0] = (*c)[0]; v[1] = (*c)[1]; v[2] = (*c)[2];
        }
        uint32_t rgba[4];
        for (int i = 0; i < 4; ++i) rgba[i] = (uint32_t)std::lround(std::clamp(v[i], 0.0, 1.0) * maxv);
        switch (ch.c) {
            case 1: out[0] = rgba[0]; break;
            case 2: out[0] = rgba[0]; out[1] = rgba[3]; break;
            default: for (int i = 0; i < ch.c; ++i) out[i] = rgba[i]; break;
        }
    }

    // ---- 4. compose + bleed + verify + write, one channel at a time -------------------------
    bool compose(std::vector<uint8_t>& owned, BleedPlan& plan, std::vector<std::string>& files) {
        owned.assign((size_t)AW * AH, 0);
        size_t overlaps = 0;
        for (const Island& isl : islands)
            for (int ly = 0; ly < isl.h; ++ly)
                for (int lx = 0; lx < isl.w; ++lx) {
                    if (!isl.mask[(size_t)ly * isl.w + lx]) continue;
                    int X, Y;
                    dest_px(isl, lx, ly, X, Y);
                    uint8_t& m = owned[(size_t)Y * AW + X];
                    overlaps += m;
                    m = 1;
                }
        if (overlaps) { say("internal error: %zu atlas pixels claimed twice\n", overlaps); return false; }

        auto t0 = Clock::now();
        plan = make_bleed_plan(owned, AW, AH);
        say("  bleed plan: %zu background pixels, %.0f ms\n", plan.order.size(),
            std::chrono::duration<double, std::milli>(Clock::now() - t0).count());

        std::vector<std::vector<int>> islandsBySource(sources.size());
        for (size_t i = 0; i < islands.size(); ++i) islandsBySource[islands[i].source].push_back((int)i);

        for (size_t ci = 0; ci < channels.size(); ++ci) {
            const Channel& ch = channels[ci];
            t0 = Clock::now();
            Image atlas;
            atlas.alloc(AW, AH, ch.c, ch.bps);
            const uint32_t maxv = atlas.maxv();
            std::vector<Image> srcImg(sources.size());

            for (size_t sid = 0; sid < sources.size(); ++sid) {
                const Source& s = sources[sid];
                bool has = !s.nomap && s.tex[ci] >= 0;
                if (has && !source_image(s, (int)ci, srcImg[sid])) return false;
                uint32_t fill[4] = {0, 0, 0, 0};
                if (!has) fill_value(s, ch, fill);
                for (int ii : islandsBySource[sid]) {
                    const Island& isl = islands[ii];
                    for (int ly = 0; ly < isl.h; ++ly)
                        for (int lx = 0; lx < isl.w; ++lx) {
                            if (!isl.mask[(size_t)ly * isl.w + lx]) continue;
                            int X, Y;
                            dest_px(isl, lx, ly, X, Y);
                            uint32_t v[4] = {fill[0], fill[1], fill[2], fill[3]};
                            if (has) {
                                const Image& im = srcImg[sid];
                                size_t sp = (size_t)wrap(isl.by + ly, im.h) * im.w + wrap(isl.bx + lx, im.w);
                                promote_pixel(im, sp, ch.c, ch.bps, v);
                                swizzle_normal(ch.kind, isl.rot, v, maxv);
                            }
                            size_t dp = (size_t)Y * AW + X;
                            for (int c = 0; c < ch.c; ++c) atlas.set(dp, c, v[c]);
                        }
                }
            }
            double composeMs = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

            // Proof, not trust: sample points inside every triangle, look the texel up through the
            // old UV in the source map and through the new UV in the atlas, and compare.
            size_t samples = 0, mismatches = 0;
            std::mt19937 rng(1234);
            std::uniform_real_distribution<double> U(0.0, 1.0);
            for (const Island& isl : islands) {
                const Source& s = sources[isl.source];
                if (s.nomap || s.tex[ci] < 0) continue;
                const Image& im = srcImg[isl.source];
                for (const TriRef& r : isl.tris) {
                    const InMesh& mesh = scene.meshes[r.mesh];
                    Tile t = faceTile[r.mesh][mesh.triFace[r.tri]];
                    const Vec2d* uv[3];
                    for (int k = 0; k < 3; ++k) uv[k] = &mesh.uvs[mesh.cornerUv[mesh.tris[r.tri][k]]];
                    for (int n = 0; n < 6; ++n) {
                        double a = U(rng), b = U(rng);
                        if (a + b > 1) { a = 1 - a; b = 1 - b; }
                        double c = 1 - a - b;
                        Vec2d p{a * uv[0]->x + b * uv[1]->x + c * uv[2]->x, a * uv[0]->y + b * uv[1]->y + c * uv[2]->y};
                        Vec2d sp = src_px(s, p, t);
                        uint32_t want[4];
                        promote_pixel(im, (size_t)wrap((int)std::floor(sp.y), im.h) * im.w + wrap((int)std::floor(sp.x), im.w),
                                      ch.c, ch.bps, want);
                        swizzle_normal(ch.kind, isl.rot, want, maxv);
                        Vec2d q = new_uv(isl, p, t);
                        int X = (int)std::floor(q.x * AW), Y = (int)std::floor((1.0 - q.y) * AH);
                        ++samples;
                        bool same = X >= 0 && Y >= 0 && X < AW && Y < AH;
                        for (int k = 0; same && k < ch.c; ++k) same = atlas.get((size_t)Y * AW + X, k) == want[k];
                        mismatches += !same;
                    }
                }
            }
            srcImg.clear();

            t0 = Clock::now();
            apply_bleed(atlas, plan, ch.kind != KIND_COLOR);
            double bleedMs = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

            std::string file = o.prefix + "_" + safe_name(ch.name) + ".png";
            std::string err;
            t0 = Clock::now();
            if (!save_png(ustr(upath(o.outDir) / upath(file)), atlas, err)) {
                say("cannot write %s: %s\n", file.c_str(), err.c_str());
                return false;
            }
            double saveMs = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            files.push_back(file);
            say("  %-18s %dch %2d-bit  copy %5.0f ms  bleed %5.0f ms  png %6.0f ms  verify %zu samples, %zu mismatches\n",
                ch.name.c_str(), ch.c, ch.bps * 8, composeMs, bleedMs, saveMs, samples, mismatches);
        }
        return true;
    }

    // ---- 5. side outputs ----------------------------------------------------------------------
    void write_layout(const std::vector<uint8_t>& owned) {
        const int f = std::max(1, (std::max(AW, AH) + 2047) / 2048);
        Image img;
        img.alloc(AW / f, AH / f, 3, 1);
        for (int y = 0; y < img.h; ++y)
            for (int x = 0; x < img.w; ++x)
                for (int c = 0; c < 3; ++c) img.set((size_t)y * img.w + x, c, 24);
        for (size_t i = 0; i < islands.size(); ++i) {
            const Island& isl = islands[i];
            uint32_t hsh = (uint32_t)(isl.source * 2654435761u) ^ (uint32_t)(i * 40503u);
            uint8_t col[3] = {(uint8_t)(80 + (hsh & 0x7F)), (uint8_t)(80 + ((hsh >> 8) & 0x7F)), (uint8_t)(80 + ((hsh >> 16) & 0x7F))};
            for (int ly = 0; ly < isl.h; ++ly)
                for (int lx = 0; lx < isl.w; ++lx) {
                    if (!isl.mask[(size_t)ly * isl.w + lx]) continue;
                    int X, Y;
                    dest_px(isl, lx, ly, X, Y);
                    X /= f; Y /= f;
                    if (X < img.w && Y < img.h)
                        for (int c = 0; c < 3; ++c) img.set((size_t)Y * img.w + X, c, col[c]);
                }
        }
        (void)owned;
        std::string err;
        save_png(ustr(upath(o.outDir) / upath(o.prefix + "_layout.png")), img, err);
    }

    void write_obj(const std::string& previewFile) {
        const fs::path dir = upath(o.outDir);
        {
            FILE* m = _wfopen((dir / upath(o.prefix + ".mtl")).c_str(), L"wb");
            if (!m) return;
            std::fprintf(m, "newmtl atlas\nKd 1 1 1\n");
            if (!previewFile.empty()) std::fprintf(m, "map_Kd %s\n", previewFile.c_str());
            std::fclose(m);
        }
        FILE* f = _wfopen((dir / upath(o.prefix + ".obj")).c_str(), L"wb");
        if (!f) return;
        std::fprintf(f, "# udimpacker preview: packed UVs, one material\nmtllib %s.mtl\n", o.prefix.c_str());
        size_t vBase = 1, vtBase = 1, vnBase = 1;
        std::string buf;
        char line[256];
        for (size_t mi = 0; mi < scene.meshes.size(); ++mi) {
            const InMesh& mesh = scene.meshes[mi];
            bool any = false;
            for (int s : faceSource[mi]) any |= s >= 0;
            if (!any) continue;
            buf.clear();
            buf += "o " + mesh.name + "\nusemtl atlas\n";
            for (const Vec3d& p : mesh.positions) {
                std::snprintf(line, sizeof line, "v %.6f %.6f %.6f\n", p[0], p[1], p[2]);
                buf += line;
            }
            for (const Vec3d& n : mesh.normals) {
                std::snprintf(line, sizeof line, "vn %.6f %.6f %.6f\n", n[0], n[1], n[2]);
                buf += line;
            }
            std::unordered_map<uint64_t, uint32_t> vt;  // (uv index, island) -> vt index
            std::string faces;
            for (size_t fi = 0; fi < mesh.faces.size(); ++fi) {
                int isl = faceIsland[mi][fi];
                if (faceSource[mi][fi] < 0 || isl < 0) continue;
                const InFace& face = mesh.faces[fi];
                faces += "f";
                for (uint32_t c = 0; c < face.count; ++c) {
                    uint32_t corner = face.first + c;
                    uint32_t uvi = mesh.cornerUv[corner];
                    uint64_t key = ((uint64_t)uvi << 32) | (uint32_t)isl;
                    auto it = vt.find(key);
                    if (it == vt.end()) {
                        Vec2d q = new_uv(islands[isl], mesh.uvs[uvi], faceTile[mi][fi]);
                        it = vt.emplace(key, (uint32_t)vt.size()).first;
                        std::snprintf(line, sizeof line, "vt %.9f %.9f\n", q.x, q.y);
                        buf += line;
                    }
                    if (mesh.normals.empty())
                        std::snprintf(line, sizeof line, " %zu/%zu", vBase + mesh.cornerPos[corner], vtBase + it->second);
                    else
                        std::snprintf(line, sizeof line, " %zu/%zu/%zu", vBase + mesh.cornerPos[corner], vtBase + it->second,
                                      vnBase + mesh.cornerNrm[corner]);
                    faces += line;
                }
                faces += "\n";
            }
            buf += faces;
            std::fwrite(buf.data(), 1, buf.size(), f);
            vBase += mesh.positions.size();
            vnBase += mesh.normals.size();
            vtBase += vt.size();
        }
        std::fclose(f);
    }

    void write_manifest(const std::vector<std::string>& files) {
        FILE* f = _wfopen((upath(o.outDir) / upath(o.prefix + "_manifest.json")).c_str(), L"wb");
        if (!f) return;
        std::fprintf(f, "{\n  \"input\": %s,\n  \"atlas\": {\"width\": %d, \"height\": %d, \"padding\": %d},\n",
                     json_str(scene.sourcePath).c_str(), AW, AH, o.padding);
        if (unsubdivided) {
            std::fprintf(f, "  \"unsubdivide\": {\"levels\": %d, \"cage\": %s, \"faces_in\": %zu, \"faces_out\": %zu, "
                            "\"triangles_in\": %zu, \"triangles_out\": %zu, \"fit_rms\": %.6g, \"fit_max\": %.6g, \"meshes\": [",
                         o.unsubdivide, o.cage ? "true" : "false", sub.facesBefore, sub.facesAfter, sub.trisBefore, sub.trisAfter,
                         sub.fitRms, sub.fitMax);
            for (size_t i = 0; i < sub.meshes.size(); ++i) {
                const SubdivReport::Mesh& m = sub.meshes[i];
                std::fprintf(f, "%s{\"name\": %s, \"exact_levels\": %d, \"approx_levels\": %d, \"dissolved\": %zu, \"joined\": %zu, "
                                "\"faces_in\": %zu, \"faces_out\": %zu, \"kept_faces\": %zu}",
                             i ? ", " : "", json_str(m.name).c_str(), m.levels, m.approxLevels, m.dissolved, m.joined, m.facesBefore,
                             m.facesAfter, m.keptFaces);
            }
            std::fprintf(f, "]},\n");
        }
        if (decimated)
            std::fprintf(f, "  \"decimation\": {\"triangles_in\": %zu, \"triangles_out\": %zu, \"error\": %.6g, "
                            "\"normal_weight\": %g, \"uv_weight\": %.6g, \"lock_border\": %s, \"flaps_removed\": %zu},\n",
                         dec.before, dec.after, dec.error, o.normalWeight, dec.uvWeight, o.lockBorder ? "true" : "false",
                         dec.flaps);
        std::fprintf(f, "  \"channels\": [\n");
        for (size_t i = 0; i < channels.size(); ++i)
            std::fprintf(f, "    {\"name\": %s, \"file\": %s, \"kind\": \"%s\", \"channels\": %d, \"bits\": %d}%s\n",
                         json_str(channels[i].name).c_str(), json_str(files[i]).c_str(),
                         channels[i].kind == KIND_COLOR ? "color" : channels[i].kind == KIND_NORMAL_GL ? "normal-gl" : "normal-dx",
                         channels[i].c, channels[i].bps * 8, i + 1 < channels.size() ? "," : "");
        std::fprintf(f, "  ],\n  \"excluded_materials\": [");
        bool first = true;
        for (size_t i = 0; i < excluded.size(); ++i)
            if (excluded[i]) { std::fprintf(f, "%s%s", first ? "" : ", ", json_str(scene.materials[i].name).c_str()); first = false; }
        std::fprintf(f, "],\n  \"sources\": [\n");
        for (size_t si = 0; si < sources.size(); ++si) {
            const Source& s = sources[si];
            std::fprintf(f, "    {\"id\": %zu, \"size\": [%d, %d], \"scale\": %g, \"materials\": [", si, s.W, s.H, std::ldexp(1.0, -s.shift));
            for (size_t k = 0; k < s.materials.size(); ++k)
                std::fprintf(f, "%s%s", k ? ", " : "", json_str(scene.materials[s.materials[k]].name).c_str());
            std::fprintf(f, "], \"maps\": {");
            bool fm = true;
            for (size_t ch = 0; ch < channels.size(); ++ch) {
                if (s.tex[ch] < 0) continue;
                std::fprintf(f, "%s%s: %s", fm ? "" : ", ", json_str(channels[ch].name).c_str(), json_str(texEntries[s.tex[ch]].label).c_str());
                fm = false;
            }
            std::fprintf(f, "}}%s\n", si + 1 < sources.size() ? "," : "");
        }
        // uv_affine maps a tile-local UV (u - tileU, v - tileV) to the atlas UV:
        //   U = a*u + b*v + c,  V = d*u + e*v + f
        std::fprintf(f, "  ],\n  \"islands\": [\n");
        for (size_t i = 0; i < islands.size(); ++i) {
            const Island& isl = islands[i];
            Vec2d o0 = new_uv(isl, {0, 0}, {}), ou = new_uv(isl, {1, 0}, {}), ov = new_uv(isl, {0, 1}, {});
            std::fprintf(f,
                         "    {\"id\": %zu, \"source\": %d, \"src_rect\": [%d, %d, %d, %d], \"rot_cw\": %d, \"dst_origin\": [%d, %d], "
                         "\"triangles\": %zu, \"uv_affine\": [%.12g, %.12g, %.12g, %.12g, %.12g, %.12g]}%s\n",
                         i, isl.source, isl.bx, isl.by, isl.w, isl.h, isl.rot * 90, isl.ox, isl.oy, isl.tris.size(),
                         ou.x - o0.x, ov.x - o0.x, o0.x, ou.y - o0.y, ov.y - o0.y, o0.y, i + 1 < islands.size() ? "," : "");
        }
        std::fprintf(f, "  ]\n}\n");
        std::fclose(f);
    }

    void report_density() {
        struct Acc { double world = 0, px = 0; };
        std::map<uint32_t, Acc> acc;
        for (size_t mi = 0; mi < scene.meshes.size(); ++mi) {
            const InMesh& mesh = scene.meshes[mi];
            for (size_t ti = 0; ti < mesh.tris.size(); ++ti) {
                uint32_t fi = mesh.triFace[ti];
                int sid = faceSource[mi][fi];
                if (sid < 0) continue;
                const auto& tr = mesh.tris[ti];
                const Vec3d& a = mesh.positions[mesh.cornerPos[tr[0]]];
                const Vec3d& b = mesh.positions[mesh.cornerPos[tr[1]]];
                const Vec3d& c = mesh.positions[mesh.cornerPos[tr[2]]];
                double e1[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]}, e2[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
                double cx = e1[1] * e2[2] - e1[2] * e2[1], cy = e1[2] * e2[0] - e1[0] * e2[2], cz = e1[0] * e2[1] - e1[1] * e2[0];
                Vec2d p[3];
                for (int k = 0; k < 3; ++k) p[k] = src_px(sources[sid], mesh.uvs[mesh.cornerUv[tr[k]]], faceTile[mi][fi]);
                Acc& ac = acc[mesh.faces[fi].material];
                ac.world += 0.5 * std::sqrt(cx * cx + cy * cy + cz * cz);
                ac.px += 0.5 * std::abs((p[1].x - p[0].x) * (p[2].y - p[0].y) - (p[1].y - p[0].y) * (p[2].x - p[0].x));
            }
        }
        say("  texel density (px per scene unit, at the packed scale):\n");
        for (const auto& [m, a] : acc)
            if (a.world > 0) say("    %-32s %8.1f\n", scene.materials[m].name.c_str(), std::sqrt(a.px / a.world));
    }
};

// ------------------------------------------------------------------------------------------------

static void print_info(const InScene& scene) {
    std::printf("scene %s\n\nmeshes (%zu)\n", scene.sourcePath.c_str(), scene.meshes.size());
    std::vector<std::map<int, size_t>> tilesPerMat(scene.materials.size());
    std::vector<size_t> facesPerMat(scene.materials.size());
    for (const InMesh& m : scene.meshes) {
        std::set<uint32_t> mats;
        for (const InFace& f : m.faces) {
            double cu = 0, cv = 0;
            for (uint32_t c = 0; c < f.count; ++c) { cu += m.uvs[m.cornerUv[f.first + c]].x; cv += m.uvs[m.cornerUv[f.first + c]].y; }
            tilesPerMat[f.material][udim_number({(int)std::floor(cu / f.count), (int)std::floor(cv / f.count)})]++;
            facesPerMat[f.material]++;
            mats.insert(f.material);
        }
        std::printf("  %-40s faces %7zu  tris %7zu  uvs %7zu  materials %zu\n", m.name.c_str(), m.faces.size(), m.tris.size(),
                    m.uvs.size(), mats.size());
    }
    std::printf("\nmaterials (%zu)\n", scene.materials.size());
    for (size_t i = 0; i < scene.materials.size(); ++i) {
        const InMaterial& mat = scene.materials[i];
        std::printf("  [%zu] %s  faces %zu  tiles", i, mat.name.c_str(), facesPerMat[i]);
        for (const auto& [t, n] : tilesPerMat[i]) std::printf(" %d(%zu)", t, n);
        std::printf("\n");
        for (const auto& [ch, v] : mat.constants)
            std::printf("      %-22s constant %.3f %.3f %.3f\n", ch.c_str(), v[0], v[1], v[2]);
        for (const InTexture& t : mat.textures) {
            std::vector<uint8_t> bytes;
            ImageInfo info;
            const char* where = "MISSING";
            if (!t.path.empty() && read_file(t.path, bytes) && image_info(bytes, info)) where = t.path.c_str();
            else if (!t.embedded.empty() && image_info(t.embedded, info)) where = "(embedded)";
            std::printf("      %-22s %-36s %5dx%-5d %dch %2d-bit  %s\n", t.channel.c_str(), t.label.c_str(), info.w, info.h, info.c,
                        info.bps * 8, where);
        }
    }
}

int main(int argc, char** argv) {
    Packer P;
    if (!parse_args(argc, argv, P.o)) return 2;
    const auto tStart = Clock::now();

    std::string err;
    say("loading %s\n", P.o.input.c_str());
    if (!load_fbx(P.o.input, P.o.uvSet, P.scene, err)) { std::fprintf(stderr, "load failed: %s\n", err.c_str()); return 1; }
    if (P.o.info) { print_info(P.scene); return 0; }

    std::error_code ec;
    fs::create_directories(upath(P.o.outDir), ec);

    P.subdivision();
    if (P.decimating()) {
        say("decimating\n");
        P.decimate();
    }

    say("binding maps\n");
    if (!P.bind()) return 1;
    for (size_t si = 0; si < P.sources.size(); ++si) {
        const Source& s = P.sources[si];
        std::string mats;
        for (uint32_t m : s.materials) mats += (mats.empty() ? "" : ", ") + P.scene.materials[m].name;
        say("  source %zu: %dx%d%s  [%s]\n", si, s.W, s.H, s.nomap ? " (no maps, constant fill)" : "", mats.c_str());
    }

    say("cutting islands\n");
    P.build_islands();
    say("  %zu islands\n", P.islands.size());
    P.report_density();

    say("packing (padding %d px, cell %d px, rotation %s)\n", P.o.padding, P.o.cell, P.o.rotate ? "on" : "off");
    if (!P.pack()) return 1;

    say("composing %dx%d\n", P.AW, P.AH);
    std::vector<uint8_t> owned;
    BleedPlan plan;
    std::vector<std::string> files;
    if (!P.compose(owned, plan, files)) return 1;
    size_t used = std::count(owned.begin(), owned.end(), (uint8_t)1);
    say("  island coverage %.1f%% of the atlas\n", 100.0 * used / owned.size());

    P.write_layout(owned);
    P.write_manifest(files);
    if (P.o.obj) {
        std::string preview;
        for (size_t i = 0; i < P.channels.size(); ++i)
            if (P.channels[i].name == P.o.previewChannel) preview = files[i];
        P.write_obj(preview);
    }
    say("done in %.1f s -> %s\n", std::chrono::duration<double>(Clock::now() - tStart).count(), P.o.outDir.c_str());
    return 0;
}
