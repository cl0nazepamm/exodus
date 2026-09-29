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

#include "atlas.h"

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

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
        "  --no-symmetry                 with --unsubdivide: do not keep mirror-symmetric meshes symmetric\n"
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


static bool split_kv(const std::string& s, std::string& k, std::string& v) {
    size_t eq = s.find('=');
    if (eq == std::string::npos) return false;
    k = s.substr(0, eq);
    v = s.substr(eq + 1);
    return true;
}

static bool parse_args(int argc, char** argv, PackOptions& o) {
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
        else if (a == "--no-symmetry") o.noSymmetry = true;
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
