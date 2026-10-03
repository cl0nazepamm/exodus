// Exodus - the packing pipeline: bind the maps of every material, cut the UV islands out of
// their source pixels, pack them into one atlas, and compose the atlas channel by channel (copy,
// verify, bleed, write). Shared by the command-line tool and the 3ds Max plugin.
#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "bleed.h"
#include "decimate.h"
#include "image.h"
#include "pack.h"
#include "subdiv.h"
#include "exodus.h"

struct PackOptions {
    std::string input, outDir, prefix, uvSet;
    int sizeW = 0, sizeH = 0;  // 0 = smallest power-of-two size that fits
    bool fill = false;         // with a size: every island scaled by one factor to fill it (pack_fill)
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
    std::vector<int> tiles;  // UDIM tiles to pack (1001, ...); empty = all. Faces elsewhere are left out.
    std::string previewChannel = "DiffuseColor";
    // decimation: off unless --tris, --decimate or --max-error is given
    size_t tris = 0;        // triangle budget for the whole scene
    double keep = 0;        // fraction of triangles to keep
    double maxError = -1;   // relative to model size; < 0 = unbounded
    double normalWeight = 0.5, uvWeight = -1;  // uvWeight < 0 = automatic
    bool lockBorder = false;
    bool symmetricDecimate = false;  // decimate mirror-symmetric meshes symmetrically
    std::vector<std::string> noDecimate;       // material globs left at full detail
    // subdivision: runs before decimation
    int unsubdivide = 0;    // Catmull-Clark levels to reverse
    bool cage = false;      // recover the control cage positions instead of keeping the smoothed ones
    bool unsubExact = false;  // refuse meshes that are not a subdivision result instead of coarsening them
    bool noSymmetry = false;  // let coarsening break the symmetry of mirror-symmetric meshes
    bool quadDominant = false;  // approximate levels by grouping quads into blocks (mostly quads, stronger)
    int subdivide = 0;      // Catmull-Clark levels to apply (utility / round-trip testing)
};

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

int udim_number(Tile t);
// "1001-1004, 1010" -> 1001 1002 1003 1004 1010; false on anything else.
bool parse_tiles(const std::string& text, std::vector<int>& tiles);
bool glob_match(const std::string& pattern, const std::string& s);  // * and ?, case-insensitive

// Progress and warnings; printf-style. Goes to stdout unless a host sets `say_hook`.
void say(const char* fmt, ...);
inline void (*say_hook)(const char* text) = nullptr;

struct Packer {
    PackOptions o;
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
    size_t verifySamples = 0, verifyMismatches = 0;  // totals of compose's verify pass

    double scale_for(const std::string& material) const;

    int tex_entry(const InTexture& t, Tile tile);

    bool load_image(int entry, Image& img);

    // Source image for one channel at the source's cutting resolution.
    bool source_image(const Source& s, int ch, Image& out);

    std::vector<uint8_t> excluded_materials() const;

    // ---- 0a. subdivision (optional) -----------------------------------------------------------
    // Reverse subdivision runs first, so decimation and the atlas both see the coarse mesh.
    void subdivision();

    // ---- 0b. decimation (optional) ------------------------------------------------------------
    // Runs before anything is bound, so islands, the atlas and the verify pass all use the game mesh.
    bool decimating() const;

    void decimate();

    // ---- 1. bindings ------------------------------------------------------------------------
    bool bind();

    // ---- 2. islands: UV coverage in source pixel space, split into connected blobs -----------
    Vec2d src_px(const Source& s, const Vec2d& uv, Tile t) const;

    void build_islands();

    // ---- 3. pack -------------------------------------------------------------------------------
    bool pack();

    // Fills o.sizeW x o.sizeH: the cutting grid of every source scaled by the largest one factor
    // whose islands still pack (the maps are resampled to it), then packed. Islands are rebuilt.
    bool pack_fill();
    double fillScale = 1;  // the factor pack_fill found

    // New UV of a tile-local source UV. Same math as the pixel copy, in continuous coordinates.
    Vec2d new_uv(const Island& isl, const Vec2d& uv, Tile t) const;

    void dest_px(const Island& isl, int lx, int ly, int& X, int& Y) const;

    const Vec4d* material_constant(const Source& s, const std::string& channel) const;

    void fill_value(const Source& s, const Channel& ch, uint32_t out[4]) const;

    // ---- 4. compose + bleed + verify + write, one channel at a time -------------------------
    bool compose(std::vector<uint8_t>& owned, BleedPlan& plan, std::vector<std::string>& files);

    // ---- 5. side outputs ----------------------------------------------------------------------
    void write_layout(const std::vector<uint8_t>& owned);

    void write_obj(const std::string& previewFile);

    void write_manifest(const std::vector<std::string>& files);

    void report_density();
};
