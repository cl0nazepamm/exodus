// udimpacker - game-mesh decimation through meshoptimizer. Runs before islands are cut, so the
// atlas is packed from the decimated triangles. Collapses only move onto existing vertices, so
// every UV in the result is an original UV and the texel copy stays exact.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "udimpacker.h"

struct DecimateOptions {
    size_t targetTris = 0;     // triangle budget for the whole scene; 0 = as far as maxError allows
    double maxError = 0;       // relative to scene size (0.01 = 1%); < 0 = unbounded
    double normalWeight = 0.5;
    double uvWeight = -1;      // < 0 = automatic: 1 / sqrt(mean UV triangle area)
    bool lockBorder = false;   // keep open mesh borders fixed
    std::vector<uint8_t> keep; // per material: 1 = leave its triangles untouched (may be empty)
};

struct DecimateResult {
    struct Mesh { std::string name; size_t before = 0, after = 0; };
    std::vector<Mesh> meshes;
    size_t before = 0, after = 0;  // triangles
    size_t vertices = 0;           // simplifier vertices (corners split by normal / UV / material)
    double error = 0;              // meshopt result error, relative to scene size (position + attributes)
    double uvWeight = 0;           // weight actually used
    size_t mixedTris = 0;          // triangles whose corners disagree on mesh or material (expected 0)
    size_t flaps = 0;              // back-to-back triangle pairs removed after simplification
};

// All meshes are simplified together, so one error scale and one budget cover the scene. Faces
// whose material is flagged in `drop` are removed first. Material borders, UV seams and normal
// splits are kept crack-free: corners that differ in any of them are separate vertices at one
// position, which the simplifier treats as a seam. Every mesh comes back as triangles
// (one InFace per triangle) with compacted position/normal/UV arrays.
DecimateResult decimate_scene(InScene& scene, const std::vector<uint8_t>& drop, const DecimateOptions& opt);
