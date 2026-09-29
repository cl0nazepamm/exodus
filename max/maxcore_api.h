// udimpacker for 3ds Max: the interface between the plugin (udimpacker.gup) and the DLL that does
// the work (udimpacker_maxcore.dll). Both are built by the same compiler with the same settings
// and the dynamic runtime, so standard containers may cross it; the plugin owns every object it
// keeps (plans live in modifier data), so the core DLL can be unloaded and reloaded at any time.
#pragma once

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

// Skin weights of every input vertex, kept per node by the Unsubdivide modifier so a vertex that
// comes back gets its own weights again (Skin only remembers the vertices it currently has).
struct UdimSkinCopy {
    std::vector<INode*> bones;            // resolved from boneNames when null (after a load)
    std::vector<std::wstring> boneNames;
    std::vector<uint32_t> first;          // per input vertex: its weights are first[v] .. first[v + 1] - 1
    std::vector<uint32_t> bone;           // per weight: index into bones
    std::vector<float> weight;
    std::vector<uint32_t> synced;         // the input vertices Skin holds these weights for (if inSync)
    bool inSync = false;
    bool empty() const { return first.empty(); }
};

// Everything the Unsubdivide modifier keeps per node.
struct UdimUnsubdivideState {
    UdimPlan plan;
    std::vector<uint32_t> current;  // input vertices of the current output
    UdimSkinCopy skin;
    bool pending = false;           // a Skin weight update is queued
};

struct UdimCoreApi {
    int version;        // kUdimCoreApiVersion of the DLL; the plugin refuses any other
    const char* build;  // build stamp of the DLL
    // JSON summary of a node: its mesh at the input of Skin (or its final mesh without Skin),
    // map channels, normals, smoothing groups, material IDs and maps, Skin bones and weights.
    bool (*inspect)(INode* node, UdimSink out);
    // Topology signature of a mesh for the given settings; a plan is valid while it matches.
    uint64_t (*unsubdivide_signature)(MNMesh& in, const UdimUnsubdivide& settings);
    // Works out which faces and vertices remain.
    bool (*unsubdivide_plan)(MNMesh& in, const UdimUnsubdivide& settings, UdimPlan& plan);
    // Builds the result: the plan's faces with the input's vertices, map channels, normals,
    // material IDs and smoothing groups.
    bool (*apply_plan)(MNMesh& in, const UdimPlan& plan, MNMesh& out);
    // The Unsubdivide modifier's ModifyObject: plan, result, and the Skin weight copy. `node` may
    // be null (Max passes none to object space modifiers). Returns true when the Skin above needs
    // its weights written once the evaluation is over (unsubdivide_write_weights).
    bool (*unsubdivide_modify)(Modifier* self, ModContext& mc, INode* node, MNMesh& mesh, const UdimUnsubdivide& settings,
                               UdimUnsubdivideState& state, std::wstring& status, INode*& nodeOut);
    // After the evaluation: writes the copied weights of the current vertices into the Skin above.
    void (*unsubdivide_write_weights)(INode* node, Modifier* self, UdimUnsubdivideState& state);
};

constexpr int kUdimCoreApiVersion = 4;

// The one export of udimpacker_maxcore.dll.
extern "C" typedef const UdimCoreApi* (*UdimCoreApiFn)();
#define UDIM_CORE_API_EXPORT "udim_core_api"
