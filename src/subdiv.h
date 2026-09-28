// udimpacker - Catmull-Clark subdivision, forward and reverse.
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

#include "udimpacker.h"

struct UnsubdivideOptions {
    int levels = 1;
    bool solveCage = false;    // recover the control cage positions instead of keeping the smoothed ones
    bool exactOnly = false;    // refuse meshes that are not a subdivision result instead of coarsening them
    std::vector<uint8_t> keep; // per material: 1 = its faces and vertices are never touched by coarsening
};

// A level that cannot be reversed exactly is approximated by polychord collapse: whole quad
// strips (the dual of edge loops) are removed, cheapest first and never two touching ones in a
// round, until the mesh has a quarter of its faces. The mesh stays a quad mesh with its edge
// flow; merged vertices slide along their edge to the point of least quadric error and their
// UVs and normals are interpolated at the same parameter, corner by corner.
struct SubdivReport {
    struct Mesh {
        std::string name;
        size_t facesBefore = 0, facesAfter = 0;
        int levels = 0;                 // levels reversed exactly (or applied, for the forward pass)
        int approxLevels = 0;           // levels approximated by chord collapse
        size_t chords = 0, rounds = 0;  // chord collapse work
        size_t keptFaces = 0;           // faces left untouched at the last exact level (not a subdivision result)
        size_t keptSets = 0;            // face sets those faces belong to
        std::string reason;             // why the last attempted exact level did not (fully) reverse; empty = fine
        std::string stop;               // why chord collapse stopped short of its target; empty = reached
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
