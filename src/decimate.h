// Exodus - game-mesh decimation through meshoptimizer. Runs before islands are cut, so the
// atlas is packed from the decimated triangles. Collapses only move onto existing vertices, so
// every UV in the result is an original UV and the texel copy stays exact.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "exodus.h"

struct DecimateOptions {
    size_t targetTris = 0;     // triangle budget for the whole scene; 0 = as far as maxError allows
    double maxError = 0;       // relative to scene size (0.01 = 1%); < 0 = unbounded
    double normalWeight = 0.5;
    double uvWeight = -1;      // < 0 = automatic: 1 / sqrt(mean UV triangle area)
    bool lockBorder = false;   // keep open mesh borders fixed
    bool symmetry = false;     // mirror-symmetric meshes stay symmetric (mirror found as unsubdivide finds it)
    std::vector<uint8_t> keep; // per material: 1 = leave its triangles untouched (may be empty)
    std::vector<std::vector<uint8_t>> lockPos;  // per mesh, per position: 1 = stays where it is (may be empty)
};

struct DecimateResult {
    struct Mesh {
        std::string name;
        size_t before = 0, after = 0;
        int mirrorAxis = -1;  // symmetry: the mirror axis (-1 = none)
        double mirrored = 0;  // symmetry: share of the faces decimated as mirror pairs
    };
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
//
// With `symmetry`, the faces of a mesh with a mirror pair up with their mirror images. One face of
// each pair is simplified and the other side is rebuilt as the mirror image of the result, corner
// for corner from its own data (positions, UVs, normals, materials), so seams of both sides are
// kept. Vertices on the mirror plane collapse only along it; faces without a mirror image are
// simplified as usual, and the positions where they meet the rebuilt side stay.
DecimateResult decimate_scene(InScene& scene, const std::vector<uint8_t>& drop, const DecimateOptions& opt);

// Positions on an edge between two faces in different groups that share no bit (3ds Max smoothing
// groups: a border between groups that do not smooth together; faces in no group are one group
// here, since a mesh with explicit normals often has none), for DecimateOptions::lockPos.
std::vector<uint8_t> group_border_positions(const InMesh& m, const std::vector<uint32_t>& groups);
