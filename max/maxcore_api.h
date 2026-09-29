// Exodus for 3ds Max: the interface between the plugin (exodus.gup) and the DLL that does
// the work (exodus_maxcore.dll). Both are built by the same compiler with the same settings
// and the dynamic runtime, so standard containers may cross it; the plugin owns every object it
// keeps (plans live in modifier data), so the core DLL can be unloaded and reloaded at any time.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

class INode;
class MNMesh;
class Modifier;
class ModContext;

// Receives UTF-8 text from the core DLL; the plugin copies it before the call returns.
struct ExodusSink {
    void (*write)(void* ctx, const char* utf8, size_t length);
    void* ctx;
};

struct ExodusUnsubdivide {
    int levels = 1;
    bool symmetry = true;
    bool quadDominant = false;  // approximate levels by grouping quads into blocks (see subdiv.h)
};

struct ExodusDecimate {
    float percent = 50;       // triangles to keep, % of the input's triangulation
    bool lockBorder = false;  // keep open borders as they are
};

using ExodusPos = std::array<float, 3>;  // a vertex position, compared bit for bit

// What unsubdivide decided, in terms of the input mesh's corners: live faces in order, corners
// in face order. Every output corner is an input corner, so every output vertex is an input
// vertex at its input position and every map / normal value is an input value.
struct ExodusPlan {
    uint64_t signature = 0;          // input topology + settings + core build it was made for
    std::vector<uint32_t> faceFirst;  // output face f has corners faceFirst[f] .. faceFirst[f + 1] - 1
    std::vector<uint32_t> corner;     // per output corner: the input corner
    std::vector<uint32_t> vertex;     // per output vertex: the input vertex (in input order)
    std::string report;               // one line for the modifier panel
};

// Skin weights by vertex position, kept per node by each reducing modifier (Unsubdivide,
// Decimate) so a vertex that comes back gets its own weights again: Skin only remembers the
// vertices it currently has. Positions, because the modifiers never move a vertex and Skin itself
// matches vertices by position; that also keeps the copies valid when a modifier below changes.
struct ExodusSkinCopy {
    std::vector<INode*> bones;            // resolved from boneNames when null (after a load)
    std::vector<std::wstring> boneNames;
    std::vector<ExodusPos> pos;             // per copied vertex
    std::vector<uint32_t> first;          // per copied vertex: its weights are first[v] .. first[v + 1] - 1
    std::vector<uint32_t> bone;           // per weight: index into bones
    std::vector<float> weight;
    std::vector<ExodusPos> synced;          // the vertices Skin holds these weights for, in Skin's order (if inSync)
    bool inSync = false;
    bool empty() const { return pos.empty(); }
};

// Everything a reducing modifier keeps per node.
struct ExodusReduceState {
    ExodusPlan plan;
    std::vector<ExodusPos> current;  // the vertices of the current output, in order
    ExodusSkinCopy skin;
    bool pending = false;          // a Skin weight update is queued
};

using ExodusIsReducer = bool (*)(Modifier*);

// UDIM Pack: the maps are given per tile by hand (no guessing from materials). Tiles with at
// least one map are packed into one atlas; faces in other tiles keep their UVs.
constexpr int kExodusPackChannels = 6;

struct ExodusPackMap {
    int tile = 1001;     // UDIM number
    int channel = 0;     // 0 .. kExodusPackChannels - 1
    std::wstring file;
};

struct ExodusPackSettings {
    int sourceChannel = 1, targetChannel = 2;  // map channels: packed UVs are read from / written to
    int size = 0;                              // atlas width and height; 0 = smallest that fits
    int maxSize = 8192;
    int padding = 8;
    bool rotate = true;
    std::wstring channel[kExodusPackChannels];   // names (atlas file suffix); empty = unused
    int kind[kExodusPackChannels] = {};          // 0 color, 1 normal map OpenGL (Y+), 2 normal map DirectX (Y-)
    std::vector<ExodusPackMap> maps;
    std::wstring outDir, prefix;               // where Bake writes; outDir empty = <scene folder>\exodus
};

struct ExodusPackTile {
    int tile = 1001;
    int faces = 0;  // faces of all parts in this tile (source UVs)
};

struct ExodusCoreApi {
    int version;        // kExodusCoreApiVersion of the DLL; the plugin refuses any other
    const char* build;  // build stamp of the DLL
    // JSON summary of a node: its mesh at the input of Skin (or its final mesh without Skin),
    // map channels, normals, smoothing groups, material IDs and maps, Skin bones and weights.
    bool (*inspect)(INode* node, ExodusSink out);
    // A modifier's ModifyObject: the plan (cached in `state` per input topology and settings), the
    // result built from the input's own vertices, map channels, normals, material IDs and
    // smoothing groups, and the Skin weight copy. `node` may be null (Max passes none to object
    // space modifiers); the node found is returned in `nodeOut`. Returns true when the Skin above
    // needs its weights written once the evaluation is over (write_weights).
    // `isReducer` tells the plugin's reducing modifiers apart: only the one nearest the Skin (no
    // other reducer in between) looks after Skin's weights.
    bool (*unsubdivide_modify)(Modifier* self, ModContext& mc, INode* node, MNMesh& mesh, const ExodusUnsubdivide& settings,
                               ExodusReduceState& state, ExodusIsReducer isReducer, std::wstring& status, INode*& nodeOut);
    bool (*decimate_modify)(Modifier* self, ModContext& mc, INode* node, MNMesh& mesh, const ExodusDecimate& settings,
                            ExodusReduceState& state, ExodusIsReducer isReducer, std::wstring& status, INode*& nodeOut);
    // After the evaluation: writes the copied weights of the current vertices into the Skin above.
    // `below` are the weight copies of the reducing modifiers further down the stack, looked up for
    // a vertex this modifier's own copy does not have. Returns true when Skin was brought in step.
    bool (*write_weights)(INode* node, Modifier* self, ExodusReduceState& state, const std::vector<ExodusSkinCopy*>& below);

    // UDIM Pack's ModifyObject. One modifier instance is shared by all parts of a character, so
    // the layout is made from every node the modifier sits on (the others are evaluated below the
    // modifier when their input is not known yet) and kept in the core per modifier. Writes the
    // packed UVs of this node into the target channel; `tiles` gets the UDIM tiles in use. Returns
    // true when the layout changed, so the other nodes need to evaluate again.
    bool (*pack_modify)(Modifier* self, ModContext& mc, INode* node, MNMesh& mesh, const ExodusPackSettings& settings,
                        std::wstring& status, std::vector<ExodusPackTile>& tiles);
    // Writes the atlas maps, layout image and manifest. Re-reads every node first; `relayout` is
    // set when that changed the layout (the nodes then need to evaluate again). `log` gets the
    // whole report.
    bool (*pack_bake)(Modifier* self, const ExodusPackSettings& settings, ExodusSink log, std::wstring& status, bool& relayout);
    void (*pack_forget)(Modifier* self);  // the modifier is gone: drop its layout
};

constexpr int kExodusCoreApiVersion = 7;

// The one export of exodus_maxcore.dll.
extern "C" typedef const ExodusCoreApi* (*ExodusCoreApiFn)();
#define EXODUS_CORE_API_EXPORT "exodus_core_api"
