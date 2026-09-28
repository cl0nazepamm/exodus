// udimpacker for 3ds Max: the interface between the plugin (udimpacker.gup) and the DLL that does
// the work (udimpacker_maxcore.dll). Plain C types and function pointers only, so the core DLL
// can be unloaded and reloaded without the plugin holding anything it owns.
#pragma once

#include <cstddef>

class INode;

// Receives UTF-8 text from the core DLL; the plugin copies it before the call returns.
struct UdimSink {
    void (*write)(void* ctx, const char* utf8, size_t length);
    void* ctx;
};

struct UdimCoreApi {
    int version;        // kUdimCoreApiVersion of the DLL; the plugin refuses any other
    const char* build;  // build stamp of the DLL
    // JSON summary of a node: its mesh at the input of Skin (or its final mesh without Skin),
    // map channels, normals, smoothing groups, material IDs and maps, Skin bones and weights.
    bool (*inspect)(INode* node, UdimSink out);
};

constexpr int kUdimCoreApiVersion = 1;

// The one export of udimpacker_maxcore.dll.
extern "C" typedef const UdimCoreApi* (*UdimCoreApiFn)();
#define UDIM_CORE_API_EXPORT "udim_core_api"
