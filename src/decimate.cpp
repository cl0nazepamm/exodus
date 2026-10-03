// Exodus - game-mesh decimation through meshoptimizer.
#include "decimate.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cfloat>
#include <cmath>
#include <map>
#include <numeric>
#include <unordered_map>

#include "meshoptimizer.h"
#include "subdiv.h"

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
// seam key all match; anything else is a separate vertex at the same position, i.e. a seam. A corner
// of a face that is simplified for its mirror image too also carries the values of the matching
// corner there (m*; kNone otherwise), so the seams of both sides are kept.
struct VKey {
    uint32_t mesh, pos, nrm, uv, material, seam;
    uint32_t mnrm = kNone, muv = kNone, mmaterial = kNone, mseam = kNone;
    bool operator==(const VKey&) const = default;
};

struct VKeyHash {
    size_t operator()(const VKey& k) const {
        const size_t h = Q3Hash{}({(int64_t)((uint64_t)k.mesh << 32 | k.material), (int64_t)((uint64_t)k.seam << 32 | k.pos),
                                   (int64_t)((uint64_t)k.nrm << 32 | k.uv)});
        if (k.muv == kNone) return h;
        return h ^ Q3Hash{}({(int64_t)((uint64_t)k.mnrm << 32 | k.muv), (int64_t)((uint64_t)k.mmaterial << 32 | k.mseam), 1});
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

// Edges of the faces `use` accepts, as (smaller position << 32 | larger, face), sorted: the faces
// of one edge are adjacent.
template <class Use>
std::vector<std::pair<uint64_t, uint32_t>> edges_of(const InMesh& m, Use&& use) {
    std::vector<std::pair<uint64_t, uint32_t>> e;
    for (uint32_t f = 0; f < m.faces.size(); ++f) {
        if (!use(f)) continue;
        const InFace& F = m.faces[f];
        for (uint32_t k = 0; k < F.count; ++k) {
            const uint32_t a = m.cornerPos[F.first + k], b = m.cornerPos[F.first + (k + 1) % F.count];
            if (a != b) e.push_back({(uint64_t)std::min(a, b) << 32 | std::max(a, b), f});
        }
    }
    std::sort(e.begin(), e.end());
    return e;
}

// Calls fn(a, b, first, last) for each edge a-b with its run [first, last) in edges_of's list.
template <class Fn>
void for_each_edge(const std::vector<std::pair<uint64_t, uint32_t>>& e, Fn&& fn) {
    for (size_t i = 0, j; i < e.size(); i = j) {
        for (j = i + 1; j < e.size() && e[j].first == e[i].first; ++j) {}
        fn((uint32_t)(e[i].first >> 32), (uint32_t)e[i].first, i, j);
    }
}

// Face roles in symmetric decimation.
enum : uint8_t {
    kFree,     // simplified as usual (no mirror image, its own mirror image, or no symmetry)
    kHalf,     // simplified, and its mirror image rebuilt from the result
    kImage,    // left out: rebuilt as the mirror image of the half
    kDropped,  // material dropped
};

// How one mesh is decimated: the role of each face, the positions that must stay, and for faces
// of the half the matching corners of their mirror images.
struct MeshPlan {
    std::vector<uint8_t> role;       // per face
    std::vector<uint32_t> faceMir;   // per face: its mirror image (kNone = none)
    std::vector<uint32_t> image;     // per corner of a kHalf face: the matching corner of its mirror image
    std::vector<uint32_t> mir;       // per position: mirror image, itself on the plane, kNone (empty = no mirror)
    std::vector<uint8_t> lock;       // per position: 1 = stays (empty = none)
    int axis = -1;
    size_t paired = 0;               // faces in mirror pairs
    bool mid(uint32_t p) const { return !mir.empty() && mir[p] == p; }
};

// The faces of a mesh with a mirror pair up: a face and the face on the mirror images of its
// positions, wound the other way. One of each pair joins the half, and the half is chosen per
// part so it stays in one piece: pairs connected without crossing the mirror plane (an edge between
// two positions on it) go the same way, the side with the larger coordinate on the axis. The half
// and the faces kept on their own are simplified together; the other side is rebuilt from the half.
// That meets the rest without cracks only where both sides do the same, so these positions stay:
//   - where a rebuilt face meets a face kept on its own,
//   - where the half meets the rebuilt side off the plane,
//   - positions on the plane whose open edge in the simplified part leads off the plane (a
//     position there may only slide along the plane, which the mirror side does alike),
//   - all positions of faces that are their own mirror image,
// and every locked position locks its mirror image too.
MeshPlan plan_mesh(const InMesh& m, const std::vector<uint8_t>& drop, const std::vector<uint8_t>* lockPos, const DecimateOptions& opt) {
    MeshPlan p;
    const uint32_t nf = (uint32_t)m.faces.size(), nv = (uint32_t)m.positions.size();
    p.role.assign(nf, kFree);
    p.faceMir.assign(nf, kNone);
    for (uint32_t f = 0; f < nf; ++f)
        if (drop[m.faces[f].material]) p.role[f] = kDropped;
    std::vector<uint8_t> lock(nv, 0);
    if (lockPos)
        for (uint32_t v = 0; v < nv && v < lockPos->size(); ++v) lock[v] = (*lockPos)[v];
    auto pos = [&](uint32_t f, uint32_t k) { return m.cornerPos[m.faces[f].first + k]; };
    auto live = [&](uint32_t f) { return p.role[f] != kDropped; };

    if (opt.symmetry) {
        p.mir = mirror_positions(m, p.axis);
        if (p.axis < 0) p.mir.clear();
    }
    if (!p.mir.empty()) {
        const std::vector<uint32_t>& mir = p.mir;
        // faces by their sorted positions (kNone where two faces share them)
        std::map<std::vector<uint32_t>, uint32_t> byVerts;
        std::vector<uint32_t> key;
        for (uint32_t f = 0; f < nf; ++f) {
            if (!live(f)) continue;
            key.clear();
            for (uint32_t k = 0; k < m.faces[f].count; ++k) key.push_back(pos(f, k));
            std::sort(key.begin(), key.end());
            if (auto [it, fresh] = byVerts.try_emplace(key, f); !fresh) it->second = kNone;
        }
        std::vector<uint32_t> fm(nf, kNone), shift(nf, 0);  // shift: corner of the image on corner 0's image
        for (uint32_t f = 0; f < nf; ++f) {
            if (!live(f)) continue;
            const uint32_t n = m.faces[f].count;
            key.clear();
            for (uint32_t k = 0; k < n && key.size() == k; ++k)
                if (mir[pos(f, k)] != kNone) key.push_back(mir[pos(f, k)]);
            if (key.size() != n) continue;
            std::sort(key.begin(), key.end());
            const auto it = byVerts.find(key);
            if (it == byVerts.end() || it->second == kNone || m.faces[it->second].count != n) continue;
            const uint32_t g = it->second;
            uint32_t j = 0;
            while (j < n && pos(g, j) != mir[pos(f, 0)]) ++j;
            bool wound = j < n;
            for (uint32_t k = 1; k < n && wound; ++k) wound = pos(g, (j + n - k) % n) == mir[pos(f, k)];
            if (wound) { fm[f] = g; shift[f] = j; }
        }
        for (uint32_t f = 0; f < nf; ++f)
            if (fm[f] != kNone && fm[fm[f]] == f) p.faceMir[f] = fm[f];
        for (uint32_t f = 0; f < nf; ++f)
            if (p.faceMir[f] == f)
                for (uint32_t k = 0; k < m.faces[f].count; ++k) lock[pos(f, k)] = 1;
        auto paired = [&](uint32_t f) { return p.faceMir[f] != kNone && p.faceMir[f] != f; };

        // parts: pairs connected without crossing the plane
        const auto edges = edges_of(m, live);
        std::vector<uint32_t> parent(nf);
        std::iota(parent.begin(), parent.end(), 0u);
        auto root = [&](uint32_t f) {
            while (parent[f] != f) f = parent[f] = parent[parent[f]];
            return f;
        };
        for_each_edge(edges, [&](uint32_t a, uint32_t b, size_t i, size_t j) {
            if (j - i != 2 || (p.mid(a) && p.mid(b))) return;
            const uint32_t f = edges[i].second, g = edges[i + 1].second;
            if (paired(f) && paired(g)) parent[root(f)] = root(g);
        });
        std::vector<double> cen(nf, 0.0), sum(nf, 0.0);
        std::vector<uint32_t> cnt(nf, 0);
        for (uint32_t f = 0; f < nf; ++f) {
            if (!paired(f)) continue;
            for (uint32_t k = 0; k < m.faces[f].count; ++k) cen[f] += m.positions[pos(f, k)][p.axis] / m.faces[f].count;
            sum[root(f)] += cen[f];
            ++cnt[root(f)];
        }
        auto in_half = [&](uint32_t f, uint32_t g) {  // f rather than its mirror image g
            const uint32_t rf = root(f), rg = root(g);
            if (rf != rg) {
                const double a = sum[rf] / cnt[rf], b = sum[rg] / cnt[rg];
                return a != b ? a > b : rf < rg;
            }
            return cen[f] != cen[g] ? cen[f] > cen[g] : f < g;
        };
        p.image.assign(m.cornerPos.size(), kNone);
        for (uint32_t f = 0; f < nf; ++f) {
            const uint32_t g = p.faceMir[f];
            if (!paired(f) || f > g) continue;
            const uint32_t h = in_half(f, g) ? f : g, i = h == f ? g : f;
            p.role[h] = kHalf;
            p.role[i] = kImage;
            p.paired += 2;
            const uint32_t n = m.faces[h].count, j = shift[h];
            for (uint32_t k = 0; k < n; ++k) p.image[m.faces[h].first + k] = m.faces[i].first + (j + n - k) % n;
        }

        std::vector<uint8_t> uses(nv, 0);  // per position: 1 << role of each face on it
        for (uint32_t f = 0; f < nf; ++f)
            if (live(f))
                for (uint32_t k = 0; k < m.faces[f].count; ++k) uses[pos(f, k)] |= 1 << p.role[f];
        for (uint32_t v = 0; v < nv; ++v) {
            const bool image = uses[v] & 1 << kImage, half = uses[v] & 1 << kHalf, free = uses[v] & 1 << kFree;
            if (image && (free || (half && !p.mid(v)))) lock[v] = 1;
        }
        for_each_edge(edges, [&](uint32_t a, uint32_t b, size_t i, size_t j) {
            size_t simplified = 0;
            for (size_t e = i; e < j; ++e) simplified += p.role[edges[e].second] != kImage;
            if (simplified != 1) return;
            if (p.mid(a) && !p.mid(b)) lock[a] = 1;
            if (p.mid(b) && !p.mid(a)) lock[b] = 1;
        });
        // without symmetry meshopt locks the borders itself; here the half has the plane as a border
        if (opt.lockBorder)
            for_each_edge(edges, [&](uint32_t a, uint32_t b, size_t i, size_t j) {
                if (j - i == 1) lock[a] = lock[b] = 1;
            });
        for (uint32_t f = 0; f < nf; ++f)
            if (live(f) && m.faces[f].material < opt.keep.size() && opt.keep[m.faces[f].material])
                for (uint32_t k = 0; k < m.faces[f].count; ++k) lock[pos(f, k)] = 1;
        for (uint32_t v = 0; v < nv; ++v)
            if (lock[v] && mir[v] != kNone) lock[mir[v]] = 1;
    } else if (opt.symmetry && opt.lockBorder) {  // no mirror: borders as meshopt would lock them
        for_each_edge(edges_of(m, live), [&](uint32_t a, uint32_t b, size_t i, size_t j) {
            if (j - i == 1) lock[a] = lock[b] = 1;
        });
    }
    if (std::find(lock.begin(), lock.end(), 1) != lock.end()) p.lock = std::move(lock);
    return p;
}

}  // namespace

std::vector<uint8_t> group_border_positions(const InMesh& m, const std::vector<uint32_t>& groups) {
    std::vector<uint8_t> lock(m.positions.size(), 0);
    const auto edges = edges_of(m, [&](uint32_t f) { return f < groups.size(); });
    for_each_edge(edges, [&](uint32_t a, uint32_t b, size_t i, size_t j) {
        for (size_t x = i; x < j; ++x)
            for (size_t y = x + 1; y < j; ++y) {
                const uint32_t g = groups[edges[x].second], h = groups[edges[y].second];
                if (g != h && !(g & h)) lock[a] = lock[b] = 1;
            }
    });
    return lock;
}

DecimateResult decimate_scene(InScene& scene, const std::vector<uint8_t>& drop, const DecimateOptions& opt) {
    DecimateResult res;
    constexpr int kAttr = 5;  // nx ny nz u v

    // ---- one indexed triangle list for the whole scene -------------------------------------
    std::vector<VKey> verts;
    std::vector<uint32_t> vertCorner;  // per simplifier vertex: the first corner with its key
    std::vector<uint32_t> vertImage;   // per simplifier vertex: the matching corner of the mirror image (kNone = none)
    std::unordered_map<VKey, uint32_t, VKeyHash> lookup;
    std::vector<float> pos, attr;
    std::vector<uint32_t> indices;
    std::vector<uint8_t> lock;
    double uvArea = 0;
    size_t imageTris = 0;  // triangles rebuilt from the mirror image of the half

    std::vector<MeshPlan> plans(scene.meshes.size());
    for (uint32_t mi = 0; mi < scene.meshes.size(); ++mi) {
        const InMesh& m = scene.meshes[mi];
        const std::vector<uint8_t>* lockPos = mi < opt.lockPos.size() && !opt.lockPos[mi].empty() ? &opt.lockPos[mi] : nullptr;
        const MeshPlan& plan = plans[mi] = plan_mesh(m, drop, lockPos, opt);
        const std::vector<uint32_t> nrmId = weld_normals(m.normals), uvId = weld_uvs(m.uvs);
        auto nrm_of = [&](uint32_t c) { return m.cornerNrm.empty() ? kNone : nrmId[m.cornerNrm[c]]; };
        auto seam_of = [&](uint32_t c) { return m.cornerSeam.empty() ? 0 : m.cornerSeam[c]; };
        size_t before = 0;
        for (size_t ti = 0; ti < m.tris.size(); ++ti) {
            const uint32_t face = m.triFace[ti], material = m.faces[face].material;
            const uint8_t role = plan.role[face];
            if (role == kDropped) continue;
            ++before;
            if (role == kImage) { ++imageTris; continue; }
            const Vec2d* uv[3];
            for (int k = 0; k < 3; ++k) {
                const uint32_t c = m.tris[ti][k];
                VKey key{mi, m.cornerPos[c], nrm_of(c), uvId[m.cornerUv[c]], material, seam_of(c)};
                const uint32_t ic = role == kHalf ? plan.image[c] : kNone;
                if (ic != kNone) {
                    key.mnrm = nrm_of(ic);
                    key.muv = uvId[m.cornerUv[ic]];
                    key.mmaterial = m.faces[plan.faceMir[face]].material;
                    key.mseam = seam_of(ic);
                }
                auto [it, fresh] = lookup.try_emplace(key, (uint32_t)verts.size());
                if (fresh) {
                    verts.push_back(key);
                    vertCorner.push_back(c);
                    vertImage.push_back(ic);
                    if (!plan.lock.empty() && plan.lock[key.pos]) {
                        lock.resize(verts.size(), 0);
                        lock.back() = meshopt_SimplifyVertex_Lock;
                    }
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
        }
        DecimateResult::Mesh mr{m.name, before, 0};
        if (!plan.mir.empty()) {
            mr.mirrorAxis = plan.axis;
            mr.mirrored = m.faces.empty() ? 0.0 : (double)plan.paired / (double)m.faces.size();
        }
        res.meshes.push_back(mr);
        res.before += before;
    }
    const size_t simplified = res.before - imageTris;  // triangles the simplifier sees
    res.vertices = verts.size();

    // ---- simplify ------------------------------------------------------------------------------
    // Automatic UV weight per the meshoptimizer README: 1 / sqrt(mean UV triangle area).
    res.uvWeight = opt.uvWeight >= 0 ? opt.uvWeight : (uvArea > 0 && simplified ? 1.0 / std::sqrt(uvArea / simplified) : 0.0);
    const float nw = (float)opt.normalWeight, uw = (float)res.uvWeight;
    const float weights[kAttr] = {nw, nw, nw, uw, uw};

    // Kept materials: every vertex locked. meshopt also locks the other wedges at those positions,
    // so neighbouring materials cannot pull away from the shared border.
    for (size_t i = 0; i < verts.size(); ++i)
        if (verts[i].material < opt.keep.size() && opt.keep[verts[i].material]) {
            lock.resize(verts.size(), 0);
            lock[i] = meshopt_SimplifyVertex_Lock;
        }

    std::vector<uint32_t> out(indices.size());
    float err = 0;
    if (!lock.empty()) lock.resize(verts.size(), 0);
    if (!indices.empty()) {
        // Each triangle of the half comes out twice: the target shrinks so the result meets it
        // when the half and the rest reduce alike.
        const size_t targetTris = imageTris ? (size_t)std::llround((double)opt.targetTris * (double)simplified / (double)res.before) : opt.targetTris;
        const size_t target = std::min(targetTris * 3, indices.size());
        const float limit = opt.maxError < 0 ? FLT_MAX : (float)opt.maxError;
        out.resize(meshopt_simplifyWithAttributes(out.data(), indices.data(), indices.size(), pos.data(), verts.size(),
                                                  3 * sizeof(float), attr.data(), kAttr * sizeof(float), weights, kAttr,
                                                  lock.empty() ? nullptr : lock.data(), target, limit,
                                                  opt.lockBorder && !opt.symmetry ? meshopt_SimplifyLockBorder : 0, &err));
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
    // One output triangle; `image` builds the mirror image of a triangle of the half instead
    // (from the matching corners there, wound the other way).
    auto emit = [&](size_t t, bool image) {
        const VKey* v[3] = {&verts[out[t]], &verts[out[t + 1]], &verts[out[t + 2]]};
        const uint32_t mi = v[0]->mesh;
        const InMesh& src = scene.meshes[mi];
        InMesh& dst = rebuilt[mi];
        const uint32_t first = (uint32_t)dst.cornerPos.size();
        dst.triFace.push_back((uint32_t)dst.faces.size());
        dst.faces.push_back({first, 3, image ? v[0]->mmaterial : v[0]->material});
        for (int k : image ? std::array<int, 3>{0, 2, 1} : std::array<int, 3>{0, 1, 2}) {
            const uint32_t p = image ? plans[mi].mir[v[k]->pos] : v[k]->pos;
            dst.cornerPos.push_back(remap(posMap[mi], p, dst.positions, src.positions));
            if (!src.cornerNrm.empty()) dst.cornerNrm.push_back(remap(nrmMap[mi], image ? v[k]->mnrm : v[k]->nrm, dst.normals, src.normals));
            dst.cornerUv.push_back(remap(uvMap[mi], image ? v[k]->muv : v[k]->uv, dst.uvs, src.uvs));
            if (!src.cornerSrc.empty()) dst.cornerSrc.push_back(src.cornerSrc[(image ? vertImage : vertCorner)[out[t + k]]]);
        }
        dst.tris.push_back({first, first + 1, first + 2});
        ++res.meshes[mi].after;
        ++res.after;
    };
    for (size_t t = 0; t + 2 < out.size(); t += 3) {
        const VKey* v[3] = {&verts[out[t]], &verts[out[t + 1]], &verts[out[t + 2]]};
        const uint32_t mi = v[0]->mesh;
        if (v[1]->mesh != mi || v[2]->mesh != mi) { ++res.mixedTris; continue; }  // cannot index one mesh
        if (v[1]->material != v[0]->material || v[2]->material != v[0]->material) ++res.mixedTris;
        const int half = (v[0]->muv != kNone) + (v[1]->muv != kNone) + (v[2]->muv != kNone);
        if (half == 3) {
            const MeshPlan& plan = plans[mi];
            if (plan.mid(v[0]->pos) && plan.mid(v[1]->pos) && plan.mid(v[2]->pos)) { ++res.flaps; continue; }  // back to back with its own image
            if (v[1]->mmaterial != v[0]->mmaterial || v[2]->mmaterial != v[0]->mmaterial) ++res.mixedTris;
            emit(t, false);
            emit(t, true);
        } else {
            if (half) ++res.mixedTris;  // half and the rest in one triangle: left without its image
            emit(t, false);
        }
    }

    scene.meshes.clear();
    for (InMesh& m : rebuilt)
        if (!m.faces.empty()) scene.meshes.push_back(std::move(m));
    return res;
}
