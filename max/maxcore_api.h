// udimpacker for 3ds Max: the interface between the plugin (udimpacker.gup) and the DLL that does
// the work (udimpacker_maxcore.dll). Both are built by the same compiler with the same settings
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
struct UdimSink {
    void (*write)(void* ctx, const char* utf8, size_t length);
    void* ctx;
};

struct UdimUnsubdivide {
    int levels = 1;
    bool symmetry = true;
};

struct UdimDecimate {
    float percent = 50;       // triangles to keep, % of the input's triangulation
    bool lockBorder = false;  // keep open borders as they are
};

using UdimPos = std::array<float, 3>;  // a vertex position, compared bit for bit

// What unsubdivide decided, in terms of the input mesh's corners: live faces in order, corners
// in face order. Every output corner is an input corner, so every output vertex is an input
// vertex at its input position and every map / normal value is an input value.
struct UdimPlan {
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
struct UdimSkinCopy {
    std::vector<INode*> bones;            // resolved from boneNames when null (after a load)
    std::vector<std::wstring> boneNames;
    std::vector<UdimPos> pos;             // per copied vertex
    std::vector<uint32_t> first;          // per copied vertex: its weights are first[v] .. first[v + 1] - 1
    std::vector<uint32_t> bone;           // per weight: index into bones
    std::vector<float> weight;
    std::vector<UdimPos> synced;          // the vertices Skin holds these weights for, in Skin's order (if inSync)
    bool inSync = false;
    bool empty() const { return pos.empty(); }
};

// Everything a reducing modifier keeps per node.
struct UdimReduceState {
    UdimPlan plan;
    std::vector<UdimPos> current;  // the vertices of the current output, in order
    UdimSkinCopy skin;
    bool pending = false;          // a Skin weight update is queued
};

using UdimIsReducer = bool (*)(Modifier*);

struct UdimCoreApi {
    int version;        // kUdimCoreApiVersion of the DLL; the plugin refuses any other
    const char* build;  // build stamp of the DLL
    // JSON summary of a node: its mesh at the input of Skin (or its final mesh without Skin),
    // map channels, normals, smoothing groups, material IDs and maps, Skin bones and weights.
    bool (*inspect)(INode* node, UdimSink out);
    // A modifier's ModifyObject: the plan (cached in `state` per input topology and settings), the
    // result built from the input's own vertices, map channels, normals, material IDs and
    // smoothing groups, and the Skin weight copy. `node` may be null (Max passes none to object
    // space modifiers); the node found is returned in `nodeOut`. Returns true when the Skin above
    // needs its weights written once the evaluation is over (write_weights).
    // `isReducer` tells the plugin's reducing modifiers apart: only the one nearest the Skin (no
    // other reducer in between) looks after Skin's weights.
    bool (*unsubdivide_modify)(Modifier* self, ModContext& mc, INode* node, MNMesh& mesh, const UdimUnsubdivide& settings,
                               UdimReduceState& state, UdimIsReducer isReducer, std::wstring& status, INode*& nodeOut);
    bool (*decimate_modify)(Modifier* self, ModContext& mc, INode* node, MNMesh& mesh, const UdimDecimate& settings,
                            UdimReduceState& state, UdimIsReducer isReducer, std::wstring& status, INode*& nodeOut);
    // After the evaluation: writes the copied weights of the current vertices into the Skin above.
    // `below` are the weight copies of the reducing modifiers further down the stack, looked up for
    // a vertex this modifier's own copy does not have. Returns true when Skin was brought in step.
    bool (*write_weights)(INode* node, Modifier* self, UdimReduceState& state, const std::vector<UdimSkinCopy*>& below);
};

constexpr int kUdimCoreApiVersion = 5;

// The one export of udimpacker_maxcore.dll.
extern "C" typedef const UdimCoreApi* (*UdimCoreApiFn)();
#define UDIM_CORE_API_EXPORT "udim_core_api"
