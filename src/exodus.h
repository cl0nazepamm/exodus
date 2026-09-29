// Exodus - shared, DCC-neutral scene types.
#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

// UTF-8 std::string <-> filesystem path (C++20 made u8string() return std::u8string).
inline std::filesystem::path upath(const std::string& s) { return std::filesystem::path(std::u8string(s.begin(), s.end())); }
inline std::string ustr(const std::filesystem::path& p) { std::u8string u = p.u8string(); return std::string(u.begin(), u.end()); }

struct Vec2d { double x = 0, y = 0; };
using Vec3d = std::array<double, 3>;
using Vec4d = std::array<double, 4>;

// One texture bound to one material channel. `channel` is the material property name
// the texture is attached to (FBX: "DiffuseColor", "NormalMap", "TransparentColor", ...).
struct InTexture {
    std::string channel;
    std::string path;               // resolved file on disk, empty if only embedded data exists
    std::string label;              // file name for reports / embedded identity
    std::vector<uint8_t> embedded;  // embedded file bytes (used only when `path` is empty)
};

struct InMaterial {
    std::string name;
    std::vector<InTexture> textures;
    // Constant property value per channel name, used to fill a channel this material has no map for.
    std::vector<std::pair<std::string, Vec4d>> constants;
};

struct InFace {
    uint32_t first = 0;     // first corner
    uint32_t count = 0;     // corner count
    uint32_t material = 0;  // index into InScene::materials
};

struct InMesh {
    std::string name;
    std::vector<Vec3d> positions;  // world space, indexed by cornerPos
    std::vector<Vec3d> normals;    // world space, indexed by cornerNrm (may be empty)
    std::vector<Vec2d> uvs;        // indexed by cornerUv
    std::vector<uint32_t> cornerPos, cornerNrm, cornerUv;
    std::vector<uint32_t> cornerSrc;   // optional: the input corner each corner comes from (kept by unsubdivide and decimate)
    std::vector<uint32_t> cornerSeam;  // optional: corners with different values never merge in decimation (extra seams)
    std::vector<InFace> faces;
    std::vector<std::array<uint32_t, 3>> tris;  // corner indices
    std::vector<uint32_t> triFace;              // owning face per triangle
};

struct InScene {
    std::string sourcePath;
    std::vector<InMaterial> materials;
    std::vector<InMesh> meshes;
};

bool load_fbx(const std::string& path, const std::string& uvSet, InScene& out, std::string& err);
