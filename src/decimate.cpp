// Exodus - game-mesh decimation through meshoptimizer.
#include "decimate.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cfloat>
#include <cmath>
#include <unordered_map>

#include "meshoptimizer.h"

namespace {

constexpr uint32_t kNone = UINT32_MAX;

using Q3 = std::array<int64_t, 3>;

struct Q3Hash {
    size_t operator()(const Q3& q) const {
        uint64_t h = 1469598103934665603ull;
        for (int64_t v : q) { h ^= (uint64_t)v; h *= 1099511628211ull; h ^= h >> 29; }
        return (size_t)h;
    }
};

// One simplifier vertex. Corners share a vertex only when mesh, position, normal, UV, material and
// seam key all match; anything else is a separate vertex at the same position, i.e. a seam.
struct VKey {
    uint32_t mesh, pos, nrm, uv, material, seam;
    bool operator==(const VKey&) const = default;
};

struct VKeyHash {
    size_t operator()(const VKey& k) const {
        return Q3Hash{}({(int64_t)((uint64_t)k.mesh << 32 | k.material), (int64_t)((uint64_t)k.seam << 32 | k.pos),
                         (int64_t)((uint64_t)k.nrm << 32 | k.uv)});
    }
};

// Canonical attribute index per value, so meshes that store one normal/UV per corner (FBX
// ByPolygonVertex) still weld into a connected surface. Normals match within 1e-4 per component,
// UVs match exactly (the texel copy depends on them).
std::vector<uint32_t> weld_normals(const std::vector<Vec3d>& n) {
    std::unordered_map<Q3, uint32_t, Q3Hash> first;
    std::vector<uint32_t> id(n.size());
    for (size_t i = 0; i < n.size(); ++i) {
        double len = std::sqrt(n[i][0] * n[i][0] + n[i][1] * n[i][1] + n[i][2] * n[i][2]);
        if (len < 1e-12) len = 1;
        Q3 q;
        for (int k = 0; k < 3; ++k) q[k] = std::llround(n[i][k] / len * 1e4);
        id[i] = first.try_emplace(q, (uint32_t)i).first->second;
    }
    return id;
}

std::vector<uint32_t> weld_uvs(const std::vector<Vec2d>& uv) {
    std::unordered_map<Q3, uint32_t, Q3Hash> first;
    std::vector<uint32_t> id(uv.size());
    for (size_t i = 0; i < uv.size(); ++i) {
        Q3 q{std::bit_cast<int64_t>(uv[i].x + 0.0), std::bit_cast<int64_t>(uv[i].y + 0.0), 0};  // +0.0: -0 == 0
        id[i] = first.try_emplace(q, (uint32_t)i).first->second;
    }
    return id;
}

// Edge collapses occasionally leave a flap: two triangles on the same three positions with
// opposite winding. Both are dropped; each of their edges loses one use in each direction, so no
// hole opens. Returns the number of pairs removed.
size_t remove_flaps(std::vector<uint32_t>& idx, const std::vector<VKey>& verts) {
    std::unordered_map<Q3, std::vector<uint32_t>, Q3Hash> byKey;  // triangle << 1 | mirrored
    for (uint32_t t = 0; t < idx.size() / 3; ++t) {
        const VKey* v[3] = {&verts[idx[t * 3]], &verts[idx[t * 3 + 1]], &verts[idx[t * 3 + 2]]};
        const uint32_t p[3] = {v[0]->pos, v[1]->pos, v[2]->pos};
        const int r = p[0] < p[1] ? (p[0] < p[2] ? 0 : 2) : (p[1] < p[2] ? 1 : 2);  // rotate smallest first
        const uint32_t a = p[r], b = p[(r + 1) % 3], c = p[(r + 2) % 3];
        byKey[{(int64_t)((uint64_t)v[0]->mesh << 32 | a), std::min(b, c), std::max(b, c)}].push_back(t << 1 | (b > c));
    }
    std::vector<uint8_t> dead(idx.size() / 3, 0);
    size_t pairs = 0;
    for (const auto& [key, list] : byKey) {
        if (list.size() < 2) continue;
        std::vector<uint32_t> pending[2];  // unmatched triangles per winding
        for (uint32_t e : list) {
            std::vector<uint32_t>& other = pending[!(e & 1)];
            if (other.empty()) { pending[e & 1].push_back(e); continue; }
            dead[other.back() >> 1] = dead[e >> 1] = 1;
            other.pop_back();
            ++pairs;
        }
    }
    size_t w = 0;
    for (size_t t = 0; t < dead.size(); ++t)
        if (!dead[t])
            for (int k = 0; k < 3; ++k) idx[w++] = idx[t * 3 + k];
    idx.resize(w);
    return pairs;
}

}  // namespace

DecimateResult decimate_scene(InScene& scene, const std::vector<uint8_t>& drop, const DecimateOptions& opt) {
    DecimateResult res;
    constexpr int kAttr = 5;  // nx ny nz u v

    // ---- one indexed triangle list for the whole scene -------------------------------------
    std::vector<VKey> verts;
    std::vector<uint32_t> vertCorner;  // per simplifier vertex: the first corner with its key
    std::unordered_map<VKey, uint32_t, VKeyHash> lookup;
    std::vector<float> pos, attr;
    std::vector<uint32_t> indices;
    double uvArea = 0;

    for (uint32_t mi = 0; mi < scene.meshes.size(); ++mi) {
        const InMesh& m = scene.meshes[mi];
        const std::vector<uint32_t> nrmId = weld_normals(m.normals), uvId = weld_uvs(m.uvs);
        size_t before = 0;
        for (size_t ti = 0; ti < m.tris.size(); ++ti) {
            const uint32_t material = m.faces[m.triFace[ti]].material;
            if (drop[material]) continue;
            const Vec2d* uv[3];
            for (int k = 0; k < 3; ++k) {
                const uint32_t c = m.tris[ti][k];
                const VKey key{mi, m.cornerPos[c], m.cornerNrm.empty() ? kNone : nrmId[m.cornerNrm[c]], uvId[m.cornerUv[c]],
                               material, m.cornerSeam.empty() ? 0 : m.cornerSeam[c]};
                auto [it, fresh] = lookup.try_emplace(key, (uint32_t)verts.size());
                if (fresh) {
                    verts.push_back(key);
                    vertCorner.push_back(c);
                    const Vec3d& p = m.positions[key.pos];
                    pos.insert(pos.end(), {(float)p[0], (float)p[1], (float)p[2]});
                    Vec3d n = {0, 0, 0};
                    if (key.nrm != kNone) {
                        n = m.normals[key.nrm];
                        double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
                        if (len > 1e-12) for (double& x : n) x /= len;
                    }
                    const Vec2d& t = m.uvs[key.uv];
                    attr.insert(attr.end(), {(float)n[0], (float)n[1], (float)n[2], (float)t.x, (float)t.y});
                }
                indices.push_back(it->second);
                uv[k] = &m.uvs[key.uv];
            }
            uvArea += 0.5 * std::abs((uv[1]->x - uv[0]->x) * (uv[2]->y - uv[0]->y) - (uv[1]->y - uv[0]->y) * (uv[2]->x - uv[0]->x));
            ++before;
        }
        res.meshes.push_back({m.name, before, 0});
        res.before += before;
    }
    res.vertices = verts.size();

    // ---- simplify ------------------------------------------------------------------------------
    // Automatic UV weight per the meshoptimizer README: 1 / sqrt(mean UV triangle area).
    res.uvWeight = opt.uvWeight >= 0 ? opt.uvWeight : (uvArea > 0 && res.before ? 1.0 / std::sqrt(uvArea / res.before) : 0.0);
    const float nw = (float)opt.normalWeight, uw = (float)res.uvWeight;
    const float weights[kAttr] = {nw, nw, nw, uw, uw};

    // Kept materials: every vertex locked. meshopt also locks the other wedges at those positions,
    // so neighbouring materials cannot pull away from the shared border.
    std::vector<uint8_t> lock;
    for (size_t i = 0; i < verts.size(); ++i)
        if (verts[i].material < opt.keep.size() && opt.keep[verts[i].material]) {
            lock.resize(verts.size(), 0);
            lock[i] = meshopt_SimplifyVertex_Lock;
        }

    std::vector<uint32_t> out(indices.size());
    float err = 0;
    if (!indices.empty()) {
        const size_t target = std::min(opt.targetTris * 3, indices.size());
        const float limit = opt.maxError < 0 ? FLT_MAX : (float)opt.maxError;
        out.resize(meshopt_simplifyWithAttributes(out.data(), indices.data(), indices.size(), pos.data(), verts.size(),
                                                  3 * sizeof(float), attr.data(), kAttr * sizeof(float), weights, kAttr,
                                                  lock.empty() ? nullptr : lock.data(), target, limit,
                                                  opt.lockBorder ? meshopt_SimplifyLockBorder : 0, &err));
        res.flaps = remove_flaps(out, verts);
        meshopt_optimizeVertexCache(out.data(), out.data(), out.size(), verts.size());
    }
    res.error = err;

    // ---- split back into meshes, compacting the attribute arrays -----------------------------
    const size_t nm = scene.meshes.size();
    std::vector<InMesh> rebuilt(nm);
    std::vector<std::vector<uint32_t>> posMap(nm), nrmMap(nm), uvMap(nm);  // old index -> new
    for (size_t i = 0; i < nm; ++i) {
        rebuilt[i].name = scene.meshes[i].name;
        posMap[i].assign(scene.meshes[i].positions.size(), kNone);
        nrmMap[i].assign(scene.meshes[i].normals.size(), kNone);
        uvMap[i].assign(scene.meshes[i].uvs.size(), kNone);
    }
    auto remap = [](std::vector<uint32_t>& map, uint32_t old, auto& dst, const auto& src) {
        if (map[old] == kNone) { map[old] = (uint32_t)dst.size(); dst.push_back(src[old]); }
        return map[old];
    };
    for (size_t t = 0; t + 2 < out.size(); t += 3) {
        const VKey* v[3] = {&verts[out[t]], &verts[out[t + 1]], &verts[out[t + 2]]};
        const uint32_t mi = v[0]->mesh;
        if (v[1]->mesh != mi || v[2]->mesh != mi) { ++res.mixedTris; continue; }  // cannot index one mesh
        if (v[1]->material != v[0]->material || v[2]->material != v[0]->material) ++res.mixedTris;
        const InMesh& src = scene.meshes[mi];
        InMesh& dst = rebuilt[mi];
        const uint32_t first = (uint32_t)dst.cornerPos.size();
        dst.triFace.push_back((uint32_t)dst.faces.size());
        dst.faces.push_back({first, 3, v[0]->material});
        for (int k = 0; k < 3; ++k) {
            dst.cornerPos.push_back(remap(posMap[mi], v[k]->pos, dst.positions, src.positions));
            if (!src.cornerNrm.empty()) dst.cornerNrm.push_back(remap(nrmMap[mi], v[k]->nrm, dst.normals, src.normals));
            dst.cornerUv.push_back(remap(uvMap[mi], v[k]->uv, dst.uvs, src.uvs));
            if (!src.cornerSrc.empty()) dst.cornerSrc.push_back(src.cornerSrc[vertCorner[out[t + k]]]);
        }
        dst.tris.push_back({first, first + 1, first + 2});
        ++res.meshes[mi].after;
        ++res.after;
    }

    scene.meshes.clear();
    for (InMesh& m : rebuilt)
        if (!m.faces.empty()) scene.meshes.push_back(std::move(m));
    return res;
}
