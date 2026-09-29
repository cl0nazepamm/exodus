// Exodus - Catmull-Clark subdivision, forward and reverse.
//
// Reverse subdivision recognises the structure one Catmull-Clark step leaves behind (every original
// face became a fan of quads around a face vertex, every original edge got a midpoint vertex) and
// removes it: the face and edge vertices go, the original vertices stay, and the original polygons
// come back with their winding. Each surviving corner keeps its own UV and normal index, so seams,
// hard edges and per-corner data are untouched. Positions are kept as they are in the input; with
// `solveCage` the control cage the input was subdivided from is recovered instead (least squares
// against the Catmull-Clark rules, exact when the input really is a subdivision result).
//
// Meshes are handled per edge-connected face set. A face set that is not a subdivision result
// (non-quads, no consistent vertex pattern, non-manifold fans) is left exactly as it is.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "exodus.h"

struct UnsubdivideOptions {
    int levels = 1;
    bool solveCage = false;    // recover the control cage positions instead of keeping the smoothed ones
    bool exactOnly = false;    // refuse meshes that are not a subdivision result instead of coarsening them
    bool symmetry = true;      // coarsening keeps a mirror-symmetric mesh symmetric (mirror found by itself)
    bool quadDominant = false; // coarsen into blocks of quads instead of dissolving every other vertex
    std::vector<uint8_t> keep; // per material: 1 = its faces and vertices are never touched by coarsening
};

// A level that cannot be reversed exactly is approximated. By default the way Blender's
// un-subdivide does it: two passes that each dissolve every other vertex in a checkerboard pattern;
// triangles appear where the pattern meets poles, and pairs of them that make a good quad are joined
// again. With quadDominant, fine quads are grouped into blocks of about four around centre vertices,
// cut at seams, and the block outlines become the coarse faces: mostly quads, and a stronger
// reduction on the second level. Either way surviving vertices keep their positions, UVs and normals.
struct SubdivReport {
    struct Mesh {
        std::string name;
        size_t facesBefore = 0, facesAfter = 0;
        int levels = 0;                 // levels reversed exactly (or applied, for the forward pass)
        int approxLevels = 0;              // levels approximated (dissolve, or quad-dominant coarsening)
        size_t removed = 0;                // vertices removed by the approximation
        size_t paired = 0;                 // odd faces (triangles, pentagons) merged into quads afterwards
        size_t triangles = 0;              // triangles left after the last coarsened level
        size_t unpaired = 0;               // merges left out to stay symmetric
        int mirrorAxis = -1;               // mirror found at the last approximate level (-1 = none)
        double mirrored = 0;               // share of its vertices with a mirror image
        size_t keptFaces = 0;           // faces left untouched at the last exact level (not a subdivision result)
        size_t keptSets = 0;            // face sets those faces belong to
        std::string reason;             // why the last attempted exact level did not (fully) reverse; empty = fine
        std::string stop;               // why coarsening found nothing to do; empty = fine
        double fitRms = 0, fitMax = 0;  // solveCage: cage re-subdivided vs input, relative to scene size
    };
    std::vector<Mesh> meshes;
    size_t facesBefore = 0, facesAfter = 0;
    size_t trisBefore = 0, trisAfter = 0;
    size_t materialMismatch = 0;  // rebuilt faces whose sub-quads disagreed on material (first one wins)
    double fitRms = 0, fitMax = 0;
    double ms = 0;
};

SubdivReport unsubdivide_scene(InScene& scene, const UnsubdivideOptions& opt);

// Forward Catmull-Clark, `levels` times. Positions, UVs and normals are all subdivided (UVs and
// normals as face-varying data: UV seams and hard normal edges act as boundaries).
SubdivReport subdivide_scene(InScene& scene, int levels);
