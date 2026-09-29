// Exodus - Catmull-Clark subdivision, forward and reverse.
#include "subdiv.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <map>
#include <set>
#include <type_traits>
#include <unordered_map>
#include <utility>

namespace {

constexpr uint32_t kNone = UINT32_MAX;
using Clock = std::chrono::steady_clock;

// ------------------------------------------------------------------------------------------------
// Polygon topology over one index channel (positions, UVs or normals). Corners are numbered in
// face order; edges are keyed by their two vertices and carry the corners that start them.

struct FaceSet {  // the faces of a mesh, corners renumbered contiguously in face order
    std::vector<uint32_t> faceFirst;  // nf + 1
    std::vector<uint32_t> srcCorner;  // new corner -> InMesh corner
};

FaceSet faces_of(const InMesh& m) {
    FaceSet fs;
    fs.faceFirst.reserve(m.faces.size() + 1);
    fs.faceFirst.push_back(0);
    for (const InFace& f : m.faces) {
        for (uint32_t c = 0; c < f.count; ++c) fs.srcCorner.push_back(f.first + c);
        fs.faceFirst.push_back((uint32_t)fs.srcCorner.size());
    }
    return fs;
}

struct Topo {
    uint32_t nv = 0;
    std::vector<uint32_t> faceFirst;               // nf + 1 corner offsets
    std::vector<uint32_t> idx;                     // corner -> vertex
    std::vector<uint32_t> faceOf;                  // corner -> face
    std::vector<uint32_t> cornerEdge;              // corner -> edge to the next corner of its face
    std::vector<uint32_t> edgeA, edgeB;            // edge endpoints, A < B
    std::vector<uint32_t> edgeFirst, edgeCorners;  // CSR edge -> corners starting it (one per face on the edge)
    std::vector<uint32_t> vertFirst, vertCorners;  // CSR vertex -> corners

    uint32_t nf() const { return (uint32_t)faceFirst.size() - 1; }
    uint32_t ne() const { return (uint32_t)edgeA.size(); }
    uint32_t deg(uint32_t f) const { return faceFirst[f + 1] - faceFirst[f]; }
    uint32_t next(uint32_t c) const { const uint32_t f = faceOf[c]; return c + 1 < faceFirst[f + 1] ? c + 1 : faceFirst[f]; }
    uint32_t prev(uint32_t c) const { const uint32_t f = faceOf[c]; return c > faceFirst[f] ? c - 1 : faceFirst[f + 1] - 1; }
    uint32_t edgeFaces(uint32_t e) const { return edgeFirst[e + 1] - edgeFirst[e]; }
    uint32_t vertFaces(uint32_t v) const { return vertFirst[v + 1] - vertFirst[v]; }
    uint32_t other(uint32_t e, uint32_t v) const { return edgeA[e] == v ? edgeB[e] : edgeA[e]; }

    void build() {
        const uint32_t nc = (uint32_t)idx.size();
        faceOf.resize(nc);
        for (uint32_t f = 0; f < nf(); ++f)
            for (uint32_t c = faceFirst[f]; c < faceFirst[f + 1]; ++c) faceOf[c] = f;

        std::vector<std::pair<uint64_t, uint32_t>> keyed(nc);
        for (uint32_t c = 0; c < nc; ++c) {
            uint32_t a = idx[c], b = idx[next(c)];
            if (a > b) std::swap(a, b);
            keyed[c] = {(uint64_t)a << 32 | b, c};
        }
        std::sort(keyed.begin(), keyed.end());
        cornerEdge.resize(nc);
        edgeCorners.resize(nc);
        edgeFirst.clear();
        edgeA.clear();
        edgeB.clear();
        for (uint32_t i = 0; i < nc; ++i) {
            if (i == 0 || keyed[i].first != keyed[i - 1].first) {
                edgeFirst.push_back(i);
                edgeA.push_back((uint32_t)(keyed[i].first >> 32));
                edgeB.push_back((uint32_t)keyed[i].first);
            }
            cornerEdge[keyed[i].second] = (uint32_t)edgeA.size() - 1;
            edgeCorners[i] = keyed[i].second;
        }
        edgeFirst.push_back(nc);

        vertFirst.assign(nv + 1, 0);
        for (uint32_t v : idx) ++vertFirst[v + 1];
        for (uint32_t v = 0; v < nv; ++v) vertFirst[v + 1] += vertFirst[v];
        vertCorners.resize(nc);
        std::vector<uint32_t> fill(vertFirst.begin(), vertFirst.end() - 1);
        for (uint32_t c = 0; c < nc; ++c) vertCorners[fill[idx[c]]++] = c;
    }
};

Topo topo_for(const FaceSet& fs, const std::vector<uint32_t>& channel, size_t nv) {
    Topo t;
    t.nv = (uint32_t)nv;
    t.faceFirst = fs.faceFirst;
    t.idx.resize(fs.srcCorner.size());
    for (size_t c = 0; c < t.idx.size(); ++c) t.idx[c] = channel[fs.srcCorner[c]];
    t.build();
    return t;
}

// ------------------------------------------------------------------------------------------------
// Catmull-Clark stencils: one sparse row per fine vertex (coarse vertices first, then one per edge,
// then one per face) in terms of the coarse vertices, plus the fine quads. Per coarse corner one
// quad (corner, next edge point, face point, previous edge point) in the coarse winding.
// Boundaries use the crease rules (edge midpoint; 1/8 3/4 1/8 along the boundary, corners too).

struct Stencil {
    std::vector<uint32_t> first{0}, col;
    std::vector<double> w;
    size_t rows() const { return first.size() - 1; }
};

struct RowBuilder {
    std::vector<std::pair<uint32_t, double>> row;
    void add(uint32_t c, double weight) { row.push_back({c, weight}); }
    void flush(Stencil& s) {
        std::sort(row.begin(), row.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
        for (const auto& [c, weight] : row) {
            if (s.col.size() > s.first.back() && s.col.back() == c) s.w.back() += weight;
            else { s.col.push_back(c); s.w.push_back(weight); }
        }
        s.first.push_back((uint32_t)s.col.size());
        row.clear();
    }
};

void cc_stencils(const Topo& t, Stencil& S, std::vector<uint32_t>& quads) {
    const uint32_t nv = t.nv, ne = t.ne(), nf = t.nf();
    S = Stencil{};
    RowBuilder rb;
    std::vector<uint32_t> edges;

    for (uint32_t v = 0; v < nv; ++v) {  // vertex points
        edges.clear();
        for (uint32_t k = t.vertFirst[v]; k < t.vertFirst[v + 1]; ++k) {
            const uint32_t c = t.vertCorners[k];
            edges.push_back(t.cornerEdge[c]);
            edges.push_back(t.cornerEdge[t.prev(c)]);
        }
        std::sort(edges.begin(), edges.end());
        edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
        const uint32_t n = (uint32_t)edges.size(), nfaces = t.vertFaces(v);
        uint32_t boundary = 0;
        for (uint32_t e : edges) boundary += t.edgeFaces(e) == 1;
        if (n >= 3 && boundary == 0) {
            rb.add(v, (double)(n - 2) / n);
            for (uint32_t e : edges) rb.add(t.other(e, v), 1.0 / ((double)n * n));
            for (uint32_t k = t.vertFirst[v]; k < t.vertFirst[v + 1]; ++k) {
                const uint32_t f = t.faceOf[t.vertCorners[k]];
                const double wf = 1.0 / ((double)n * nfaces * t.deg(f));
                for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c) rb.add(t.idx[c], wf);
            }
        } else if (boundary == 2) {
            rb.add(v, 0.75);
            for (uint32_t e : edges)
                if (t.edgeFaces(e) == 1) rb.add(t.other(e, v), 0.125);
        } else {
            rb.add(v, 1.0);  // isolated, corner of a single edge, or non-manifold: stays
        }
        rb.flush(S);
    }
    for (uint32_t e = 0; e < ne; ++e) {  // edge points
        if (t.edgeFaces(e) == 2) {
            rb.add(t.edgeA[e], 0.25);
            rb.add(t.edgeB[e], 0.25);
            for (uint32_t k = t.edgeFirst[e]; k < t.edgeFirst[e + 1]; ++k) {
                const uint32_t f = t.faceOf[t.edgeCorners[k]];
                const double wf = 0.25 / t.deg(f);
                for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c) rb.add(t.idx[c], wf);
            }
        } else {
            rb.add(t.edgeA[e], 0.5);
            rb.add(t.edgeB[e], 0.5);
        }
        rb.flush(S);
    }
    for (uint32_t f = 0; f < nf; ++f) {  // face points
        const double wf = 1.0 / t.deg(f);
        for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c) rb.add(t.idx[c], wf);
        rb.flush(S);
    }
    quads.clear();
    quads.reserve(t.idx.size() * 4);
    for (uint32_t f = 0; f < nf; ++f)
        for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c) {
            quads.push_back(t.idx[c]);
            quads.push_back(nv + t.cornerEdge[c]);
            quads.push_back(nv + ne + f);
            quads.push_back(nv + t.cornerEdge[t.prev(c)]);
        }
}

template <size_t D>
std::vector<std::array<double, D>> apply(const Stencil& S, const std::vector<std::array<double, D>>& in) {
    std::vector<std::array<double, D>> out(S.rows());
    for (size_t r = 0; r < S.rows(); ++r) {
        std::array<double, D> acc{};
        for (uint32_t k = S.first[r]; k < S.first[r + 1]; ++k)
            for (size_t d = 0; d < D; ++d) acc[d] += S.w[k] * in[S.col[k]][d];
        out[r] = acc;
    }
    return out;
}

// ------------------------------------------------------------------------------------------------
// Triangulation matching the loader (ufbx): quads split along the shorter diagonal unless one
// split would fold, larger polygons as a fan.

void triangulate(InMesh& m) {
    m.tris.clear();
    m.triFace.clear();
    for (uint32_t fi = 0; fi < m.faces.size(); ++fi) {
        const InFace& f = m.faces[fi];
        auto tri = [&](uint32_t a, uint32_t b, uint32_t c) {
            m.tris.push_back({f.first + a, f.first + b, f.first + c});
            m.triFace.push_back(fi);
        };
        if (f.count == 4) {
            const Vec3d* v[4];
            for (int k = 0; k < 4; ++k) v[k] = &m.positions[m.cornerPos[f.first + k]];
            auto sub = [](const Vec3d& a, const Vec3d& b) { return Vec3d{a[0] - b[0], a[1] - b[1], a[2] - b[2]}; };
            auto cross = [](const Vec3d& a, const Vec3d& b) {
                return Vec3d{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
            };
            auto dot = [](const Vec3d& a, const Vec3d& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
            auto unit = [&](Vec3d a) { double l = std::sqrt(dot(a, a)); if (l > 0) for (double& x : a) x /= l; return a; };
            const Vec3d a = sub(*v[2], *v[0]), b = sub(*v[3], *v[1]);
            const double na = dot(unit(cross(a, sub(*v[1], *v[0]))), unit(cross(a, sub(*v[0], *v[3]))));
            const double nb = dot(unit(cross(b, sub(*v[1], *v[0]))), unit(cross(b, sub(*v[2], *v[1]))));
            bool splitA = dot(a, a) <= dot(b, b);
            if (na < 0 || nb < 0) splitA = na >= nb;
            if (splitA) { tri(0, 1, 2); tri(0, 2, 3); }
            else { tri(0, 1, 3); tri(1, 2, 3); }
        } else {
            for (uint32_t k = 1; k + 1 < f.count; ++k) tri(0, k, k + 1);
        }
    }
}

// Drop unreferenced positions / normals / UVs and renumber the corners.
void compact(InMesh& m) {
    auto pack = [](auto& values, std::vector<uint32_t>& corners) {
        std::vector<uint32_t> map(values.size(), kNone);
        std::remove_reference_t<decltype(values)> out;
        for (uint32_t& c : corners) {
            if (map[c] == kNone) { map[c] = (uint32_t)out.size(); out.push_back(values[c]); }
            c = map[c];
        }
        values = std::move(out);
    };
    pack(m.positions, m.cornerPos);
    pack(m.uvs, m.cornerUv);
    if (!m.cornerNrm.empty()) pack(m.normals, m.cornerNrm);
}

double scene_diagonal(const InScene& scene) {
    Vec3d lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
    for (const InMesh& m : scene.meshes)
        for (const Vec3d& p : m.positions)
            for (int k = 0; k < 3; ++k) { lo[k] = std::min(lo[k], p[k]); hi[k] = std::max(hi[k], p[k]); }
    double d = 0;
    for (int k = 0; k < 3; ++k) d += hi[k] > lo[k] ? (hi[k] - lo[k]) * (hi[k] - lo[k]) : 0;
    return d > 0 ? std::sqrt(d) : 1.0;
}

// ------------------------------------------------------------------------------------------------
// Forward subdivision of one mesh: positions, UVs and normals through their own topologies. The
// three topologies share the face structure, so their fine quads line up corner for corner.

void subdivide_mesh(InMesh& m) {
    const FaceSet fs = faces_of(m);
    Stencil S;
    std::vector<uint32_t> q;
    cc_stencils(topo_for(fs, m.cornerPos, m.positions.size()), S, q);
    InMesh out;
    out.name = m.name;
    out.positions = apply<3>(S, m.positions);
    out.cornerPos = std::move(q);
    cc_stencils(topo_for(fs, m.cornerUv, m.uvs.size()), S, q);
    {
        std::vector<std::array<double, 2>> in(m.uvs.size());
        for (size_t i = 0; i < in.size(); ++i) in[i] = {m.uvs[i].x, m.uvs[i].y};
        const auto fine = apply<2>(S, in);
        out.uvs.resize(fine.size());
        for (size_t i = 0; i < fine.size(); ++i) out.uvs[i] = {fine[i][0], fine[i][1]};
    }
    out.cornerUv = std::move(q);
    if (!m.cornerNrm.empty()) {
        cc_stencils(topo_for(fs, m.cornerNrm, m.normals.size()), S, q);
        out.normals = apply<3>(S, m.normals);
        for (Vec3d& n : out.normals) {
            const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
            if (len > 1e-12) for (double& x : n) x /= len;
        }
        out.cornerNrm = std::move(q);
    }
    uint32_t first = 0;
    for (uint32_t f = 0; f < m.faces.size(); ++f)
        for (uint32_t k = 0; k < m.faces[f].count; ++k, first += 4) out.faces.push_back({first, 4, m.faces[f].material});
    triangulate(out);
    m = std::move(out);
}

// ------------------------------------------------------------------------------------------------
// Reverse subdivision of one level on one mesh.
//
// Vertex labels: O = original vertex, E = edge midpoint, F = face vertex. Every quad of a
// subdivided mesh reads (O, E, F, E) around its corners and every edge joins an E to an O or an F,
// so one labelled quad fixes the labels of every quad it touches. Each edge-connected face set is
// labelled from one seed quad, trying the four rotations of the pattern; the labelling is then
// checked (E vertices join exactly two O's and 2k faces for k F's; F vertices sit inside a closed,
// manifold fan of quads) and the fan around each F vertex is walked to rebuild the original face.

enum : uint8_t { L_NONE = 0, L_O = 1, L_E = 2, L_F = 3 };

struct Level {
    // rebuilt faces: corners are FaceSet corner ids, mid[i] is the E vertex between corner i and i+1
    std::vector<uint32_t> cfFirst{0}, cfCorner, cfMid, cfCenter, cfMaterial;
    std::vector<uint8_t> faceKept;  // per fine face
    size_t keptFaces = 0, keptSets = 0, materialMismatch = 0;
    std::string reason;
};

struct Fit { double sumSq = 0, worst = 0; size_t rows = 0; };
Fit solve_cage(const InMesh& m, const FaceSet& fs, const Level& R, std::vector<Vec3d>* write);

void reverse_level(const InMesh& m, const FaceSet& fs, const Topo& t, Level& R) {
    const uint32_t nf = t.nf();
    std::vector<uint8_t> label(t.nv, L_NONE), state(nf, 0), vis(nf, 0), vmark(t.nv, 0);
    std::vector<uint32_t> comp, compVerts, queue, edges, walkStamp(nf, kNone), vstamp(t.nv, kNone);
    R.faceKept.assign(nf, 0);
    uint32_t walkId = 0;
    std::string attempt;  // why the current labelling attempt failed

    auto fail = [&](const char* why) {
        if (attempt.empty()) attempt = why;
        return false;
    };

    // Label the component from `seed` with the pattern rotated by `rot`.
    auto propagate = [&](uint32_t seed, int rot) {
        for (uint32_t v : compVerts) label[v] = L_NONE;
        for (uint32_t f : comp) vis[f] = 0;
        static const uint8_t pattern[4] = {L_O, L_E, L_F, L_E};
        for (int j = 0; j < 4; ++j) {
            uint8_t& l = label[t.idx[t.faceFirst[seed] + j]];
            const uint8_t want = pattern[(j + rot) & 3];
            if (l != L_NONE && l != want) return false;
            l = want;
        }
        queue.clear();
        queue.push_back(seed);
        vis[seed] = 1;
        for (size_t qi = 0; qi < queue.size(); ++qi) {
            const uint32_t f = queue[qi], base = t.faceFirst[f];
            if (f != seed) {
                int k = -1;
                for (int j = 0; j < 4 && k < 0; ++j)
                    if (label[t.idx[base + j]] == L_O || label[t.idx[base + j]] == L_F) k = j;
                if (k < 0) return false;
                uint8_t want[4];
                want[k] = label[t.idx[base + k]];
                want[(k + 2) & 3] = want[k] == L_O ? L_F : L_O;
                want[(k + 1) & 3] = want[(k + 3) & 3] = L_E;
                for (int j = 0; j < 4; ++j) {
                    uint8_t& l = label[t.idx[base + j]];
                    if (l != L_NONE && l != want[j]) return false;
                    l = want[j];
                }
            }
            for (uint32_t c = base; c < base + 4; ++c) {
                const uint32_t e = t.cornerEdge[c];
                for (uint32_t k = t.edgeFirst[e]; k < t.edgeFirst[e + 1]; ++k) {
                    const uint32_t g = t.faceOf[t.edgeCorners[k]];
                    if (!vis[g]) { vis[g] = 1; queue.push_back(g); }
                }
            }
        }
        return true;
    };

    // Check the labelling and rebuild the faces of the component. Appends to R on success.
    auto rebuild = [&]() {
        for (uint32_t v : compVerts) {
            if (label[v] != L_E) continue;
            edges.clear();
            for (uint32_t k = t.vertFirst[v]; k < t.vertFirst[v + 1]; ++k) {
                const uint32_t c = t.vertCorners[k];
                edges.push_back(t.cornerEdge[c]);
                edges.push_back(t.cornerEdge[t.prev(c)]);
            }
            std::sort(edges.begin(), edges.end());
            edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
            uint32_t nO = 0, nF = 0;
            for (uint32_t e : edges) {
                const uint8_t l = label[t.other(e, v)];
                nO += l == L_O;
                nF += l == L_F;
            }
            if (nO != 2 || nF == 0 || t.vertFaces(v) != 2 * nF) return fail("edge vertices do not join two original vertices");
        }
        for (uint32_t v : compVerts) {
            if (label[v] != L_F) continue;
            const uint32_t n = t.vertFaces(v);
            if (n < 3) return fail("face vertex with fewer than three quads");
            ++walkId;
            uint32_t c = t.vertCorners[t.vertFirst[v]];
            const uint32_t c0 = c;
            const uint32_t material = m.faces[t.faceOf[c]].material;
            bool mismatch = false;
            for (uint32_t step = 0; step < n; ++step) {
                const uint32_t f = t.faceOf[c], base = t.faceFirst[f];
                if (walkStamp[f] == walkId) return fail("quads around a face vertex do not form one fan");
                walkStamp[f] = walkId;
                mismatch |= m.faces[f].material != material;
                const uint32_t o = base + ((c - base + 2) & 3), before = t.prev(c);
                const uint32_t ov = t.idx[o];
                if (vstamp[ov] == walkId) return fail("rebuilt face repeats a vertex");
                vstamp[ov] = walkId;
                R.cfCorner.push_back(o);
                R.cfMid.push_back(t.idx[before]);
                const uint32_t e = t.cornerEdge[before];  // edge (E, F) shared with the next quad of the fan
                if (t.edgeFaces(e) != 2) return fail("non-manifold edge at a face vertex");
                uint32_t g = kNone;
                for (uint32_t k = t.edgeFirst[e]; k < t.edgeFirst[e + 1]; ++k)
                    if (t.faceOf[t.edgeCorners[k]] != f) g = t.faceOf[t.edgeCorners[k]];
                if (g == kNone) return fail("edge used twice by one quad");
                c = kNone;
                for (uint32_t gc = t.faceFirst[g]; gc < t.faceFirst[g + 1]; ++gc)
                    if (t.idx[gc] == v) c = gc;
                if (c == kNone) return fail("quads around a face vertex do not close");
            }
            if (c != c0) return fail("quads around a face vertex do not close");
            R.cfFirst.push_back((uint32_t)R.cfCorner.size());
            R.cfCenter.push_back(v);
            R.cfMaterial.push_back(material);
            R.materialMismatch += mismatch;
        }
        return true;
    };

    for (uint32_t seed = 0; seed < nf; ++seed) {
        if (state[seed]) continue;
        // the edge-connected face set around `seed`
        comp.clear();
        comp.push_back(seed);
        state[seed] = 1;
        bool quads = true, degenerate = false;
        for (size_t qi = 0; qi < comp.size(); ++qi) {
            const uint32_t f = comp[qi];
            if (t.deg(f) != 4) quads = false;
            for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c) {
                const uint32_t e = t.cornerEdge[c];
                degenerate |= t.edgeA[e] == t.edgeB[e];
                for (uint32_t k = t.edgeFirst[e]; k < t.edgeFirst[e + 1]; ++k) {
                    const uint32_t g = t.faceOf[t.edgeCorners[k]];
                    if (!state[g]) { state[g] = 1; comp.push_back(g); }
                }
            }
        }
        compVerts.clear();
        for (uint32_t f : comp)
            for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c)
                if (!vmark[t.idx[c]]) { vmark[t.idx[c]] = 1; compVerts.push_back(t.idx[c]); }
        for (uint32_t v : compVerts) vmark[v] = 0;

        bool ok = false;
        attempt.clear();
        if (!quads) fail("not all quads");
        else if (degenerate) fail("degenerate faces");
        else {
            // Every valid labelling is a candidate. A closed face set also validates with O and F
            // swapped (that rebuilds the dual mesh, which subdivides to the same topology), and a
            // regular grid with the pattern shifted, so when several rotations pass, the one whose
            // rebuilt cage reproduces the input under the Catmull-Clark rules best wins.
            const size_t nFace = R.cfFirst.size(), nCorner = R.cfCorner.size();
            const size_t mismatch = R.materialMismatch;
            std::vector<Level> cands;
            for (int rot = 0; rot < 4; ++rot) {
                attempt.clear();
                if (propagate(seed, rot) && rebuild()) {
                    Level c;
                    for (size_t i = nFace; i < R.cfFirst.size(); ++i) c.cfFirst.push_back(R.cfFirst[i] - (uint32_t)nCorner);
                    c.cfCorner.assign(R.cfCorner.begin() + nCorner, R.cfCorner.end());
                    c.cfMid.assign(R.cfMid.begin() + nCorner, R.cfMid.end());
                    c.cfCenter.assign(R.cfCenter.begin() + (nFace - 1), R.cfCenter.end());
                    c.cfMaterial.assign(R.cfMaterial.begin() + (nFace - 1), R.cfMaterial.end());
                    c.materialMismatch = R.materialMismatch - mismatch;
                    cands.push_back(std::move(c));
                }
                R.cfFirst.resize(nFace);
                R.cfCenter.resize(nFace - 1);
                R.cfMaterial.resize(nFace - 1);
                R.cfCorner.resize(nCorner);
                R.cfMid.resize(nCorner);
                R.materialMismatch = mismatch;
            }
            if (!cands.empty()) {
                size_t best = 0;
                if (cands.size() > 1) {
                    double bestSq = 0;
                    for (size_t i = 0; i < cands.size(); ++i) {
                        const double sq = solve_cage(m, fs, cands[i], nullptr).sumSq;
                        if (i == 0 || sq < bestSq) { bestSq = sq; best = i; }
                    }
                }
                const Level& c = cands[best];
                for (size_t i = 1; i < c.cfFirst.size(); ++i) R.cfFirst.push_back(c.cfFirst[i] + (uint32_t)nCorner);
                R.cfCorner.insert(R.cfCorner.end(), c.cfCorner.begin(), c.cfCorner.end());
                R.cfMid.insert(R.cfMid.end(), c.cfMid.begin(), c.cfMid.end());
                R.cfCenter.insert(R.cfCenter.end(), c.cfCenter.begin(), c.cfCenter.end());
                R.cfMaterial.insert(R.cfMaterial.end(), c.cfMaterial.begin(), c.cfMaterial.end());
                R.materialMismatch += c.materialMismatch;
                ok = true;
            } else if (attempt.empty()) attempt = "no consistent vertex pattern";
        }
        if (!ok) {
            if (R.reason.empty()) R.reason = attempt;
            for (uint32_t f : comp) R.faceKept[f] = 1;
            R.keptFaces += comp.size();
            ++R.keptSets;
        }
        for (uint32_t v : compVerts) label[v] = L_NONE;
    }
}

// Recover the control cage positions: least squares fit of the Catmull-Clark rules on the rebuilt
// topology to the fine positions (conjugate gradients on the normal equations, warm-started from
// the smoothed positions, which are already close). Writes the result over the original vertices.
// Returns the sum of squared residuals, the largest residual (scene units) and the row count. The
// solved positions go to `write` (the mesh's own position array, or null to only measure the fit).
Fit solve_cage(const InMesh& m, const FaceSet& fs, const Level& R, std::vector<Vec3d>* write) {
    const size_t nCoarse = R.cfFirst.size() - 1;
    std::vector<uint32_t> fineOfO, coarseId(m.positions.size(), kNone);
    Topo ct;
    ct.faceFirst = R.cfFirst;
    ct.idx.resize(R.cfCorner.size());
    for (size_t i = 0; i < R.cfCorner.size(); ++i) {
        const uint32_t v = m.cornerPos[fs.srcCorner[R.cfCorner[i]]];
        if (coarseId[v] == kNone) { coarseId[v] = (uint32_t)fineOfO.size(); fineOfO.push_back(v); }
        ct.idx[i] = coarseId[v];
    }
    ct.nv = (uint32_t)fineOfO.size();
    ct.build();
    Stencil S;
    std::vector<uint32_t> quads;
    cc_stencils(ct, S, quads);

    std::vector<uint32_t> fineOf(S.rows(), kNone);
    for (uint32_t i = 0; i < ct.nv; ++i) fineOf[i] = fineOfO[i];
    for (size_t i = 0; i < R.cfCorner.size(); ++i) {
        uint32_t& slot = fineOf[ct.nv + ct.cornerEdge[i]];
        if (slot == kNone) slot = R.cfMid[i];
    }
    for (size_t f = 0; f < nCoarse; ++f) fineOf[ct.nv + ct.ne() + f] = R.cfCenter[f];

    const size_t rows = S.rows(), cols = ct.nv;
    std::vector<Vec3d> b(rows), x(cols), r(cols), p(cols), q(rows), sq(cols);
    for (size_t i = 0; i < rows; ++i) b[i] = m.positions[fineOf[i]];
    for (size_t i = 0; i < cols; ++i) x[i] = m.positions[fineOfO[i]];
    auto mul = [&](const std::vector<Vec3d>& in, std::vector<Vec3d>& out) {
        for (size_t i = 0; i < rows; ++i) {
            Vec3d acc{};
            for (uint32_t k = S.first[i]; k < S.first[i + 1]; ++k)
                for (int d = 0; d < 3; ++d) acc[d] += S.w[k] * in[S.col[k]][d];
            out[i] = acc;
        }
    };
    auto mulT = [&](const std::vector<Vec3d>& in, std::vector<Vec3d>& out) {
        for (Vec3d& v : out) v = {};
        for (size_t i = 0; i < rows; ++i)
            for (uint32_t k = S.first[i]; k < S.first[i + 1]; ++k)
                for (int d = 0; d < 3; ++d) out[S.col[k]][d] += S.w[k] * in[i][d];
    };
    auto dot = [](const std::vector<Vec3d>& a, const std::vector<Vec3d>& c) {
        double s = 0;
        for (size_t i = 0; i < a.size(); ++i) s += a[i][0] * c[i][0] + a[i][1] * c[i][1] + a[i][2] * c[i][2];
        return s;
    };
    // r = S^T (b - S x)
    mul(x, q);
    for (size_t i = 0; i < rows; ++i) for (int d = 0; d < 3; ++d) q[i][d] = b[i][d] - q[i][d];
    mulT(q, r);
    p = r;
    double rr = dot(r, r);
    const double tol = std::max(1e-28, rr * 1e-24);
    for (int it = 0; it < 500 && rr > tol; ++it) {
        mul(p, q);
        const double qq = dot(q, q);
        if (qq <= 0) break;
        const double alpha = rr / qq;
        for (size_t i = 0; i < cols; ++i) for (int d = 0; d < 3; ++d) x[i][d] += alpha * p[i][d];
        mulT(q, sq);
        for (size_t i = 0; i < cols; ++i) for (int d = 0; d < 3; ++d) r[i][d] -= alpha * sq[i][d];
        const double rrNew = dot(r, r);
        const double beta = rrNew / rr;
        rr = rrNew;
        for (size_t i = 0; i < cols; ++i) for (int d = 0; d < 3; ++d) p[i][d] = r[i][d] + beta * p[i][d];
    }
    mul(x, q);
    double sumSq = 0, worst = 0;
    for (size_t i = 0; i < rows; ++i) {
        double e2 = 0;
        for (int d = 0; d < 3; ++d) e2 += (q[i][d] - b[i][d]) * (q[i][d] - b[i][d]);
        sumSq += e2;
        worst = std::max(worst, e2);
    }
    if (write)
        for (size_t i = 0; i < cols; ++i) (*write)[fineOfO[i]] = x[i];
    return {sumSq, std::sqrt(worst), rows};
}

// Assemble the coarser mesh: untouched faces as they were, rebuilt faces from their surviving
// corners (position, UV and normal indices copied from the fine corners).
void rebuild_mesh(InMesh& m, const FaceSet& fs, const Level& R) {
    InMesh out;
    out.name = m.name;
    out.positions = std::move(m.positions);
    out.normals = std::move(m.normals);
    out.uvs = std::move(m.uvs);
    const bool hasN = !m.cornerNrm.empty(), hasSrc = !m.cornerSrc.empty();
    auto corner = [&](uint32_t c) {
        const uint32_t oc = fs.srcCorner[c];
        out.cornerPos.push_back(m.cornerPos[oc]);
        out.cornerUv.push_back(m.cornerUv[oc]);
        if (hasN) out.cornerNrm.push_back(m.cornerNrm[oc]);
        if (hasSrc) out.cornerSrc.push_back(m.cornerSrc[oc]);
    };
    for (uint32_t f = 0; f < m.faces.size(); ++f) {
        if (!R.faceKept[f]) continue;
        out.faces.push_back({(uint32_t)out.cornerPos.size(), fs.faceFirst[f + 1] - fs.faceFirst[f], m.faces[f].material});
        for (uint32_t c = fs.faceFirst[f]; c < fs.faceFirst[f + 1]; ++c) corner(c);
    }
    for (size_t f = 0; f + 1 < R.cfFirst.size(); ++f) {
        out.faces.push_back({(uint32_t)out.cornerPos.size(), R.cfFirst[f + 1] - R.cfFirst[f], R.cfMaterial[f]});
        for (uint32_t c = R.cfFirst[f]; c < R.cfFirst[f + 1]; ++c) corner(R.cfCorner[c]);
    }
    compact(out);
    triangulate(out);
    m = std::move(out);
}

// ------------------------------------------------------------------------------------------------
// Mirror symmetry. A character's topology is usually symmetric even where its shape is not quite
// (morphs, sculpting), so the mirror is found on the topology: a face is matched to its mirror
// image by position, and the match is walked across the mesh face by face with the winding
// reversed. Every part not reached from an earlier seed (eyes, lashes) gets a seed of its own.
// Positions only pick the seed and confirm the walk (most of its vertices must land within half an
// edge of their mirror image); after that the topology decides, so a posed or sculpted side still
// pairs up. A vertex reached with two different images has none. The three axes are tried through
// the mean vertex; the one that pairs the most vertices wins. Returns the mirror of every position
// (itself on the plane, kNone where there is none).
std::vector<uint32_t> find_mirror(const InMesh& m, int& axisOut) {
    const FaceSet fs = faces_of(m);
    const Topo t = topo_for(fs, m.cornerPos, m.positions.size());
    const uint32_t nf = t.nf(), nv = t.nv;
    const std::vector<Vec3d>& P = m.positions;
    auto dist = [](const Vec3d& a, const Vec3d& b) {
        return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
    };
    axisOut = -1;
    std::vector<uint32_t> best(nv, kNone);
    if (!nf || !t.ne()) return best;

    // local edge length per vertex and face, face centres, and a grid over the centres
    std::vector<double> vlen(nv, 0.0), flen(nf, 0.0);
    std::vector<uint32_t> vcnt(nv, 0);
    double lenSum = 0;
    for (uint32_t e = 0; e < t.ne(); ++e) {
        const double l = dist(P[t.edgeA[e]], P[t.edgeB[e]]);
        vlen[t.edgeA[e]] += l;
        vlen[t.edgeB[e]] += l;
        ++vcnt[t.edgeA[e]];
        ++vcnt[t.edgeB[e]];
        lenSum += l;
    }
    for (uint32_t v = 0; v < nv; ++v)
        if (vcnt[v]) vlen[v] /= vcnt[v];
    std::vector<Vec3d> cen(nf, Vec3d{0, 0, 0});
    for (uint32_t f = 0; f < nf; ++f)
        for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c) {
            for (int d = 0; d < 3; ++d) cen[f][d] += P[t.idx[c]][d] / t.deg(f);
            flen[f] += vlen[t.idx[c]] / t.deg(f);
        }
    const double cell = std::max(2 * lenSum / t.ne(), 1e-12);
    auto key = [&](const Vec3d& p, int dx, int dy, int dz) {
        const int64_t x = (int64_t)std::floor(p[0] / cell) + dx, y = (int64_t)std::floor(p[1] / cell) + dy,
                      z = (int64_t)std::floor(p[2] / cell) + dz;
        return (uint64_t)(x & 0x1FFFFF) << 42 | (uint64_t)(y & 0x1FFFFF) << 21 | (uint64_t)(z & 0x1FFFFF);
    };
    std::vector<std::pair<uint64_t, uint32_t>> grid(nf);
    for (uint32_t f = 0; f < nf; ++f) grid[f] = {key(cen[f], 0, 0, 0), f};
    std::sort(grid.begin(), grid.end());

    // the corner that starts the same edge the other way round in the face across, or kNone
    auto across = [&](uint32_t c) {
        const uint32_t e = t.cornerEdge[c];
        if (t.edgeFaces(e) != 2) return kNone;
        for (uint32_t k = t.edgeFirst[e]; k < t.edgeFirst[e + 1]; ++k) {
            const uint32_t c2 = t.edgeCorners[k];
            if (t.faceOf[c2] != t.faceOf[c] && t.idx[c2] == t.idx[t.next(c)]) return c2;
        }
        return kNone;
    };

    size_t bestCount = 0;
    std::vector<uint32_t> fimg(nf), vimg(nv), queue, comp, touched, clashed, res(nv);
    std::vector<uint8_t> tried(nf), bad(nv);
    for (int axis = 0; axis < 3; ++axis) {
        double plane = 0;
        size_t used = 0;
        for (uint32_t v = 0; v < nv; ++v)
            if (vcnt[v]) { plane += P[v][axis]; ++used; }
        plane /= (double)std::max<size_t>(used, 1);
        auto mirror = [&](Vec3d p) { p[axis] = 2 * plane - p[axis]; return p; };
        std::fill(fimg.begin(), fimg.end(), kNone);
        std::fill(vimg.begin(), vimg.end(), kNone);
        std::fill(tried.begin(), tried.end(), 0);
        std::fill(bad.begin(), bad.end(), 0);
        for (uint32_t f = 0; f < nf; ++f) {
            if (fimg[f] != kNone || tried[f]) continue;
            tried[f] = 1;
            // seed: the face and corner that match the mirror image of f best, within half an edge
            const Vec3d mc = mirror(cen[f]);
            const uint32_t n0 = t.deg(f);
            double bestErr = 0.5 * flen[f];
            uint32_t seed = kNone;
            for (int dx = -1; dx <= 1; ++dx)
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dz = -1; dz <= 1; ++dz) {
                        const uint64_t k = key(mc, dx, dy, dz);
                        auto it = std::lower_bound(grid.begin(), grid.end(), std::pair<uint64_t, uint32_t>{k, 0});
                        for (; it != grid.end() && it->first == k; ++it) {
                            const uint32_t g = it->second;
                            if (t.deg(g) != n0) continue;
                            for (uint32_t j = 0; j < n0; ++j) {
                                double err = 0;
                                for (uint32_t i = 0; i < n0 && err < bestErr; ++i)
                                    err = std::max(err, dist(mirror(P[t.idx[t.faceFirst[f] + i]]), P[t.idx[t.faceFirst[g] + (j + n0 - i) % n0]]));
                                if (err < bestErr) { bestErr = err; seed = t.faceFirst[g] + j; }
                            }
                        }
                    }
            if (seed == kNone) continue;
            // walk: corner c of a face maps to corner d of its image, next(c) to prev(d)
            comp.assign(1, f);
            touched.clear();
            clashed.clear();
            fimg[f] = t.faceOf[seed];
            queue.assign({t.faceFirst[f], seed});
            for (size_t qi = 0; qi < queue.size(); qi += 2) {
                uint32_t c = queue[qi], d = queue[qi + 1];
                const uint32_t n = t.deg(t.faceOf[c]);
                for (uint32_t k = 0; k < n; ++k, c = t.next(c), d = t.prev(d)) {
                    const uint32_t a = t.idx[c], b = t.idx[d];
                    if (vimg[a] == kNone) { vimg[a] = b; touched.push_back(a); }
                    else if (vimg[a] != b && !bad[a]) { bad[a] = 1; clashed.push_back(a); }
                    const uint32_t c2 = across(c), d2 = across(t.prev(d));
                    if (c2 == kNone || d2 == kNone) continue;
                    const uint32_t f2 = t.faceOf[c2], g2 = t.faceOf[d2];
                    if (fimg[f2] != kNone || t.deg(f2) != t.deg(g2)) continue;
                    fimg[f2] = g2;
                    comp.push_back(f2);
                    queue.push_back(c2);
                    queue.push_back(t.next(d2));
                }
            }
            size_t near = 0;
            for (uint32_t a : touched) near += dist(mirror(P[a]), P[vimg[a]]) <= 0.5 * vlen[a];
            if (2 * near < touched.size()) {  // a wrong seed: undo the walk
                for (uint32_t g : comp) { fimg[g] = kNone; tried[g] = 1; }
                for (uint32_t a : touched) vimg[a] = kNone;
                for (uint32_t a : clashed) bad[a] = 0;
            }
        }
        size_t count = 0;
        for (uint32_t v = 0; v < nv; ++v) {
            const uint32_t w = vimg[v];
            res[v] = w != kNone && !bad[v] && !bad[w] && vimg[w] == v ? w : kNone;
            count += res[v] != kNone;
        }
        if (count > bestCount) {
            bestCount = count;
            best = res;
            axisOut = axis;
        }
    }
    return best;
}

// ------------------------------------------------------------------------------------------------
// Coarsening: the approximate reverse of one subdivision level, for meshes that are not a
// subdivision result (an artist's cage, or a level below it). Only input vertices are kept, at
// their input positions, and every corner keeps an input corner's UV and normal.
//
// A quad mesh coarsened by two reads like a subdivision result backwards: each coarse face is a
// block of fine quads around a centre vertex, the block corners stay, the centres and the vertices
// halfway along the block sides go. So every vertex is labelled corner, side or centre:
// - The vertex graph of a quad mesh has two colour classes: corners and centres take one, sides
//   the other. Of the two ways round, the one with more poles (valence other than 4) and fixed
//   vertices among corners and centres wins. Faces with an odd corner count (a coarser level's
//   triangles) break the classes; they are paired up along short paths, and only there may two
//   neighbours share a class.
// - Corners and centres alternate across quad diagonals. Around a side vertex with an odd number
//   of quads they cannot; those vertices are paired up along short paths too, and only there may
//   a diagonal join two corners or two centres. Then flood fill, improve vertex by vertex, and of
//   the two ways round take the one with more poles and fixed vertices as corners.
// - Each quad joins the block of its centre corner. A quad with two centres joins both blocks into
//   one, a quad with none joins a neighbouring block, and where the classes break two quads glue
//   into a half block. Blocks are cut at walls (mesh borders, UV seams, hard edges, material
//   borders), so a wall never runs through a coarse face; a wall along centres leaves half blocks
//   on both sides, which are still quads.
// - Coarse vertices are those where three or more blocks meet, plus the fixed ones (where a wall
//   ends, turns or meets another). Every other vertex goes.
// Poles and odd wall lengths still leave faces with an odd corner count. Neighbouring odd faces
// are merged and cut again into quads, best quads first, also across one face in between; what is
// left keeps a triangle. No cut runs along an existing edge or turns a face over. On a mirror-
// symmetric mesh (find_mirror) labels are set per mirror pair and every cut is made on both sides
// (the mirror side cut the mirrored way), so the result is symmetric too.

struct CoarsenResult {
    size_t removed = 0;    // vertices removed
    size_t paired = 0;     // odd faces merged with a neighbour into quads
    size_t triangles = 0;  // triangles left
    size_t unpaired = 0;   // merges left out because the mirror side could not follow
    int axis = -1;         // mirror axis (-1 = none)
    double mirrored = 0;   // share of the vertices with a mirror image
    std::string stop;      // why nothing was coarsened (empty = fine)
};

Vec3d vsub(const Vec3d& a, const Vec3d& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
double vdot(const Vec3d& a, const Vec3d& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
Vec3d vcross(const Vec3d& a, const Vec3d& b) { return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}; }
double vangle(const Vec3d& a, const Vec3d& b) {
    const double l = std::sqrt(vdot(a, a) * vdot(b, b));
    return l > 0 ? std::acos(std::clamp(vdot(a, b) / l, -1.0, 1.0)) : 0.0;
}
constexpr double kHalfPi = 1.57079632679489661923;

// How good a quad is, 0 .. 1: 0 when concave, folded or degenerate. The worst corner's distance
// from a right angle and the bend between the two halves (either way of cutting it) both count.
double quad_quality(const Vec3d* p[4]) {
    Vec3d n[4], sum{0, 0, 0};
    double worst = 0;
    for (int i = 0; i < 4; ++i) {
        const Vec3d a = vsub(*p[(i + 1) % 4], *p[i]), b = vsub(*p[(i + 3) % 4], *p[i]);
        n[i] = vcross(a, b);
        for (int k = 0; k < 3; ++k) sum[k] += n[i][k];
        worst = std::max(worst, std::abs(vangle(a, b) - kHalfPi));
    }
    if (vdot(sum, sum) <= 0) return 0;
    for (const Vec3d& x : n)
        if (vdot(x, sum) <= 0) return 0;
    const double bend = std::max(vangle(vcross(vsub(*p[1], *p[0]), vsub(*p[2], *p[0])), vcross(vsub(*p[2], *p[0]), vsub(*p[3], *p[0]))),
                                 vangle(vcross(vsub(*p[2], *p[1]), vsub(*p[3], *p[1])), vcross(vsub(*p[3], *p[1]), vsub(*p[0], *p[1]))));
    return std::max(0.0, 1 - worst / kHalfPi) * std::max(0.0, 1 - bend / kHalfPi);
}

// How good a triangle is, 0 .. 1 (1 = equilateral): its smallest angle against 60 degrees.
double tri_quality(const Vec3d* p[3]) {
    if (vdot(vcross(vsub(*p[1], *p[0]), vsub(*p[2], *p[0])), vcross(vsub(*p[1], *p[0]), vsub(*p[2], *p[0]))) <= 0) return 0;
    double least = kHalfPi;
    for (int i = 0; i < 3; ++i) least = std::min(least, vangle(vsub(*p[(i + 1) % 3], *p[i]), vsub(*p[(i + 2) % 3], *p[i])));
    return least / (kHalfPi * 2 / 3);
}

// The best way to cut a polygon into quads and at most `maxTris` triangles, without new corners.
// Faces are corner positions in the polygon (a triangle's fourth is -1). Best = the worst face as
// good as possible, then the sum. A triangle counts at half its quality, so quads win when they can.
struct Split {
    double worst = -1, sum = 0;
    int tris = 0;
    std::vector<std::array<int, 4>> faces;
};

// What a cut has to respect: chords it may not make because they are an edge of another face
// already (n x n, 1 = not allowed), and which way the surface faces. A new face must agree with
// the surface normals at its corners or with the polygon's own normal: the first fails on thin
// shells (nails), where the normals of both sides cancel, the second on polygons that curve round.
struct CutRules {
    std::vector<uint8_t> blocked;
    std::vector<Vec3d> up;  // per polygon corner (empty: no check)
    Vec3d own{0, 0, 0};     // the polygon's normal

    bool allows(const std::vector<const Vec3d*>& p, const std::array<int, 4>& face) const {
        const int n = (int)p.size(), m = face[3] < 0 ? 3 : 4;
        Vec3d normal{0, 0, 0}, surface{0, 0, 0};
        for (int k = 0; k < m; ++k) {
            const int x = face[k], y = face[(k + 1) % m], w = face[(k + m - 1) % m];
            if (y != (x + 1) % n && !blocked.empty() && blocked[(size_t)x * n + y]) return false;
            const Vec3d c = vcross(vsub(*p[y], *p[x]), vsub(*p[w], *p[x]));
            for (int d = 0; d < 3; ++d) normal[d] += c[d];
            if (!up.empty())
                for (int d = 0; d < 3; ++d) surface[d] += up[x][d];
        }
        return up.empty() || vdot(normal, surface) > 0 || vdot(normal, own) > 0;
    }
};

Split best_split(const std::vector<const Vec3d*>& p, int maxTris, const CutRules* rules = nullptr) {
    const int n = (int)p.size();
    std::map<std::array<int, 3>, Split> memo;
    // the part of the polygon between corners i and j (i < j) that lies beyond the chord i - j
    auto solve = [&](auto&& self, int i, int j, int trisLeft) -> Split {
        Split none;
        if (j - i == 1) { none.worst = 2; return none; }  // just an edge: nothing to cut
        const std::array<int, 3> key{i, j, trisLeft};
        if (auto it = memo.find(key); it != memo.end()) return it->second;
        Split best;
        auto consider = [&](std::array<int, 4> face, double q, int tris, std::initializer_list<std::pair<int, int>> parts) {
            if (q <= 0 || tris > trisLeft || (rules && !rules->allows(p, face))) return;
            Split s;
            s.worst = q;
            s.sum = q;
            s.tris = tris;
            s.faces.push_back(face);
            for (auto [a, b] : parts) {
                const Split r = self(self, a, b, trisLeft - s.tris);
                if (r.worst < 0) return;
                s.worst = std::min(s.worst, r.worst);
                s.sum += r.sum;
                s.tris += r.tris;
                s.faces.insert(s.faces.end(), r.faces.begin(), r.faces.end());
            }
            if (s.tris > trisLeft) return;
            if (s.worst > best.worst + 1e-12 || (std::abs(s.worst - best.worst) <= 1e-12 && s.sum > best.sum)) best = std::move(s);
        };
        for (int a = i + 1; a < j; ++a) {
            const Vec3d* tri[3] = {p[i], p[a], p[j]};
            consider({i, a, j, -1}, 0.5 * tri_quality(tri), 1, {{i, a}, {a, j}});
            for (int b = a + 1; b < j; ++b) {
                const Vec3d* quad[4] = {p[i], p[a], p[b], p[j]};
                consider({i, a, b, j}, quad_quality(quad), 0, {{i, a}, {a, b}, {b, j}});
            }
        }
        memo[key] = best;
        return best;
    };
    if (n < 3) return {};
    return solve(solve, 0, n - 1, maxTris);
}

// Every way to cut a polygon as best_split does (for small polygons that need a cut with a
// property the best one may lack).
std::vector<Split> every_split(const std::vector<const Vec3d*>& p, int maxTris, const CutRules* rules = nullptr) {
    const int n = (int)p.size();
    std::map<std::array<int, 3>, std::vector<Split>> memo;
    auto solve = [&](auto&& self, int i, int j, int trisLeft) -> std::vector<Split> {
        if (j - i == 1) {
            Split edge;
            edge.worst = 2;
            return {edge};
        }
        const std::array<int, 3> key{i, j, trisLeft};
        if (auto it = memo.find(key); it != memo.end()) return it->second;
        std::vector<Split> all;
        auto combine = [&](std::array<int, 4> face, double q, int tris, std::initializer_list<std::pair<int, int>> parts) {
            if (q <= 0 || tris > trisLeft || (rules && !rules->allows(p, face))) return;
            std::vector<Split> acc(1);
            acc[0].worst = acc[0].sum = q;
            acc[0].tris = tris;
            acc[0].faces.push_back(face);
            for (auto [a, b] : parts) {
                std::vector<Split> next;
                for (const Split& x : acc)
                    for (const Split& y : self(self, a, b, trisLeft - x.tris)) {
                        if (x.tris + y.tris > trisLeft) continue;
                        Split z = x;
                        z.worst = std::min(x.worst, y.worst);
                        z.sum += y.sum;
                        z.tris += y.tris;
                        z.faces.insert(z.faces.end(), y.faces.begin(), y.faces.end());
                        next.push_back(std::move(z));
                    }
                acc.swap(next);
            }
            all.insert(all.end(), acc.begin(), acc.end());
        };
        for (int a = i + 1; a < j; ++a) {
            const Vec3d* tri[3] = {p[i], p[a], p[j]};
            combine({i, a, j, -1}, 0.5 * tri_quality(tri), 1, {{i, a}, {a, j}});
            for (int b = a + 1; b < j; ++b) {
                const Vec3d* quad[4] = {p[i], p[a], p[b], p[j]};
                combine({i, a, b, j}, quad_quality(quad), 0, {{i, a}, {a, b}, {b, j}});
            }
        }
        memo[key] = all;
        return all;
    };
    if (n < 3) return {};
    return solve(solve, 0, n - 1, maxTris);
}

// Pairs up the `odd` nodes of a graph along shortest paths: a flood from all of them at once, the
// closest meetings first. adj: node -> (neighbour, link between them). mirror(node): its mirror
// image (the node itself on the mirror plane, kNone without one); mirror_link(link): the mirror
// image of a link, or kNone. A pair's mirror image is paired too, along the mirror image of the
// same path, so the result stays symmetric. Returns, per link, whether a path runs over it.
template <class Mirror, class MirrorLink>
std::vector<uint8_t> pair_up(size_t nodes, size_t links, const std::vector<std::vector<std::pair<uint32_t, uint32_t>>>& adj,
                             std::vector<uint32_t> odd, Mirror&& mirror, MirrorLink&& mirror_link) {
    std::vector<uint8_t> onPath(links, 0);
    auto orb = [&](uint32_t v) {
        const uint32_t w = mirror(v);
        return w == kNone ? v : std::min(v, w);
    };
    std::sort(odd.begin(), odd.end(), [&](uint32_t a, uint32_t b) { return std::make_pair(orb(a), a) < std::make_pair(orb(b), b); });
    std::vector<uint32_t> dist(nodes, kNone), src(nodes, kNone), via(nodes, kNone), from(nodes, kNone), layer = odd, next;
    for (uint32_t v : odd) {
        dist[v] = 0;
        src[v] = v;
    }
    for (uint32_t d = 0; !layer.empty(); ++d) {
        next.clear();
        for (uint32_t x : layer)
            for (auto [y, l] : adj[x]) {
                if (dist[y] == kNone) {
                    dist[y] = d + 1;
                    next.push_back(y);
                } else if (dist[y] != d + 1 || orb(src[x]) >= orb(src[y])) {
                    continue;  // reached before, or by a source that wins ties
                }
                src[y] = src[x];
                via[y] = l;
                from[y] = x;
            }
        layer.swap(next);
    }
    struct Meet { uint32_t len, lo, hi, x, y, l; };
    std::vector<Meet> meets;
    for (uint32_t x = 0; x < nodes; ++x) {
        if (dist[x] == kNone) continue;
        for (auto [y, l] : adj[x]) {
            if (y < x || dist[y] == kNone || src[x] == src[y]) continue;
            const uint32_t a = orb(src[x]), b = orb(src[y]);
            meets.push_back({dist[x] + dist[y] + 1, std::min(a, b), std::max(a, b), x, y, l});
        }
    }
    std::sort(meets.begin(), meets.end(), [](const Meet& a, const Meet& b) { return std::tie(a.len, a.lo, a.hi, a.x) < std::tie(b.len, b.lo, b.hi, b.x); });
    auto mark = [&](const Meet& mt, bool mirrorToo) {
        std::vector<uint32_t> path{mt.l};
        for (uint32_t v : {mt.x, mt.y})
            for (; from[v] != kNone; v = from[v]) path.push_back(via[v]);
        for (uint32_t l : path) {
            onPath[l] = 1;
            if (mirrorToo)
                if (const uint32_t ml = mirror_link(l); ml != kNone) onPath[ml] = 1;
        }
    };
    std::vector<uint8_t> matched(nodes, 0);
    for (const Meet& mt : meets) {
        const uint32_t p = src[mt.x], q = src[mt.y];
        if (matched[p] || matched[q]) continue;
        const uint32_t mp = mirror(p), mq = mirror(q);
        const bool pOn = mp == p, qOn = mq == q, pNone = mp == kNone, qNone = mq == kNone;
        if (mp == q || (pNone && qNone) || (pOn && qOn)) {  // its own mirror image, or no mirror at all
            mark(mt, false);
            matched[p] = matched[q] = 1;
            continue;
        }
        if (pOn || qOn || pNone || qNone || matched[mp] || matched[mq]) continue;
        mark(mt, true);
        matched[p] = matched[q] = matched[mp] = matched[mq] = 1;
    }
    return onPath;
}

CoarsenResult coarsen_level(InMesh& m, const std::vector<uint8_t>& lockedMaterial, bool symmetric) {
    CoarsenResult res;
    const FaceSet fs = faces_of(m);
    const Topo t = topo_for(fs, m.cornerPos, m.positions.size());
    const uint32_t nf = t.nf(), nv = t.nv, ne = t.ne();
    const std::vector<Vec3d>& P = m.positions;
    const bool hasN = !m.cornerNrm.empty(), hasSrc = !m.cornerSrc.empty();
    auto locked = [&](uint32_t f) {
        const uint32_t mat = m.faces[f].material;
        return mat < lockedMaterial.size() && lockedMaterial[mat] != 0;
    };
    const std::vector<uint32_t> mir = symmetric ? find_mirror(m, res.axis) : std::vector<uint32_t>(nv, kNone);
    {
        size_t paired = 0;
        for (uint32_t v : mir) paired += v != kNone;
        res.mirrored = nv ? (double)paired / nv : 0;
    }
    auto twin = [&](uint32_t v) { return mir[v] == kNone ? v : mir[v]; };  // the mirror image, or itself

    // ---- walls: edges a coarse face may not cross ----
    std::vector<uint8_t> wall(ne, 0);
    for (uint32_t e = 0; e < ne; ++e) {
        if (t.edgeFaces(e) != 2) { wall[e] = 1; continue; }
        const uint32_t c1 = t.edgeCorners[t.edgeFirst[e]], c2 = t.edgeCorners[t.edgeFirst[e] + 1];
        const uint32_t f1 = t.faceOf[c1], f2 = t.faceOf[c2];
        if (f1 == f2 || locked(f1) || locked(f2) || m.faces[f1].material != m.faces[f2].material) { wall[e] = 1; continue; }
        auto same = [&](uint32_t a, uint32_t b) {  // two corners at one vertex carry the same UV and normal
            const uint32_t sa = fs.srcCorner[a], sb = fs.srcCorner[b];
            return m.cornerUv[sa] == m.cornerUv[sb] && (!hasN || m.cornerNrm[sa] == m.cornerNrm[sb]);
        };
        const uint32_t a1 = c1, b1 = t.next(c1);
        const bool along = t.idx[c2] == t.idx[a1];  // the other face runs the edge the same way (flipped face)
        const uint32_t a2 = along ? c2 : t.next(c2), b2 = along ? t.next(c2) : c2;
        if (!same(a1, a2) || !same(b1, b2)) wall[e] = 1;
    }

    // ---- per vertex: its edges, poles and fixed vertices ----
    std::vector<uint32_t> vertEdgeFirst(nv + 1, 0), vertEdges;
    {
        std::vector<std::vector<uint32_t>> lists(nv);
        for (uint32_t e = 0; e < ne; ++e) { lists[t.edgeA[e]].push_back(e); lists[t.edgeB[e]].push_back(e); }
        for (uint32_t v = 0; v < nv; ++v) {
            vertEdges.insert(vertEdges.end(), lists[v].begin(), lists[v].end());
            vertEdgeFirst[v + 1] = (uint32_t)vertEdges.size();
        }
    }
    std::vector<uint8_t> fixed(nv, 0), pole(nv, 0);
    for (uint32_t v = 0; v < nv; ++v) {
        uint32_t walls = 0, border = 0, w[2] = {kNone, kNone};
        bool manifold = true;
        for (uint32_t k = vertEdgeFirst[v]; k < vertEdgeFirst[v + 1]; ++k) {
            const uint32_t e = vertEdges[k];
            if (t.edgeFaces(e) > 2) manifold = false;
            border += t.edgeFaces(e) == 1;
            if (wall[e]) { if (walls < 2) w[walls] = e; ++walls; }
        }
        const uint32_t valence = vertEdgeFirst[v + 1] - vertEdgeFirst[v];
        bool anyLocked = false;
        for (uint32_t k = t.vertFirst[v]; k < t.vertFirst[v + 1]; ++k) anyLocked |= locked(t.faceOf[t.vertCorners[k]]);
        bool straight = false;  // a wall passing straight through
        if (walls == 2 && manifold) {
            if (border) {
                straight = border == 2 && t.vertFaces(v) == 2 && valence == 3;
            } else {
                straight = valence == 4;
                for (uint32_t k = t.vertFirst[v]; k < t.vertFirst[v + 1] && straight; ++k) {
                    const uint32_t c = t.vertCorners[k], in = t.cornerEdge[t.prev(c)], out = t.cornerEdge[c];
                    if ((in == w[0] && out == w[1]) || (in == w[1] && out == w[0])) straight = false;  // the wall turns here
                }
            }
        }
        fixed[v] = anyLocked || !manifold || (walls != 0 && !straight);
        pole[v] = !walls && valence != 4;
    }

    // ---- labels: class (0 corner or centre, 1 side), then corner (0) or centre (1) ----
    // Flood fills set a vertex and its mirror image together, so the labels come out symmetric.
    auto weight = [&](uint32_t v, bool asCorner) {  // how much a vertex wants to be kept
        if (fixed[v]) return 3.0;
        if (!pole[v]) return 0.0;
        return asCorner ? 1.0 : 0.5;  // a pole as a centre gives an odd face, but at least not a defect
    };
    // Faces with an odd number of corners (triangles, pentagons) break the two classes around them.
    // They are paired up along short paths over the faces, and only the edges crossed on the way
    // may join two vertices of one class.
    std::vector<uint8_t> classCut;
    {
        std::vector<std::vector<std::pair<uint32_t, uint32_t>>> faceAdj(nf);
        for (uint32_t e = 0; e < ne; ++e) {
            if (t.edgeFaces(e) != 2) continue;
            const uint32_t f1 = t.faceOf[t.edgeCorners[t.edgeFirst[e]]], f2 = t.faceOf[t.edgeCorners[t.edgeFirst[e] + 1]];
            if (f1 == f2 || locked(f1) || locked(f2)) continue;
            faceAdj[f1].push_back({f2, e});
            faceAdj[f2].push_back({f1, e});
        }
        std::vector<uint32_t> odd;
        for (uint32_t f = 0; f < nf; ++f)
            if (!locked(f) && t.deg(f) % 2) odd.push_back(f);
        std::vector<uint32_t> mirFace(nf, kNone);
        if (!odd.empty()) {
            std::map<std::vector<uint32_t>, uint32_t> byVerts;
            for (uint32_t f = 0; f < nf; ++f) {
                std::vector<uint32_t> vs(t.idx.begin() + t.faceFirst[f], t.idx.begin() + t.faceFirst[f + 1]);
                std::sort(vs.begin(), vs.end());
                byVerts[vs] = f;
            }
            for (uint32_t f = 0; f < nf; ++f) {
                std::vector<uint32_t> vs;
                for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c) vs.push_back(mir[t.idx[c]]);
                if (std::find(vs.begin(), vs.end(), kNone) != vs.end()) continue;
                std::sort(vs.begin(), vs.end());
                if (auto it = byVerts.find(vs); it != byVerts.end()) mirFace[f] = it->second;
            }
        }
        auto edge_between = [&](uint32_t a, uint32_t b) {
            for (uint32_t k = vertEdgeFirst[a]; k < vertEdgeFirst[a + 1]; ++k)
                if (t.other(vertEdges[k], a) == b) return vertEdges[k];
            return kNone;
        };
        classCut = pair_up(nf, ne, faceAdj, odd, [&](uint32_t f) { return mirFace[f]; }, [&](uint32_t e) {
            const uint32_t a = mir[t.edgeA[e]], b = mir[t.edgeB[e]];
            return a == kNone || b == kNone ? kNone : edge_between(a, b);
        });
    }
    std::vector<uint8_t> cls(nv, 2);
    {
        std::vector<uint32_t> queue, comp;
        for (uint32_t s = 0; s < nv; ++s) {
            if (cls[s] != 2 || vertEdgeFirst[s + 1] == vertEdgeFirst[s]) continue;
            cls[s] = cls[twin(s)] = 0;
            queue.assign({s, twin(s)});
            comp = queue;
            for (size_t qi = 0; qi < queue.size(); ++qi) {
                const uint32_t v = queue[qi];
                for (uint32_t k = vertEdgeFirst[v]; k < vertEdgeFirst[v + 1]; ++k) {
                    const uint32_t u = t.other(vertEdges[k], v);
                    if (cls[u] != 2 || classCut[vertEdges[k]]) continue;
                    cls[u] = cls[v] ^ 1;
                    if (cls[twin(u)] == 2) cls[twin(u)] = cls[u];
                    queue.push_back(u);
                    queue.push_back(twin(u));
                    comp.push_back(u);
                    comp.push_back(twin(u));
                }
            }
            double score = 0;  // for the classes as they are, against flipped
            for (uint32_t v : comp) score += cls[v] == 0 ? weight(v, false) : -weight(v, false);
            if (score < 0) {  // flip, once per vertex (comp can hold one twice)
                for (uint32_t v : comp) if (cls[v] < 2) cls[v] = 2 + (cls[v] ^ 1);
                for (uint32_t v : comp) if (cls[v] >= 2) cls[v] -= 2;
            }
        }
    }
    // corners and centres across quad diagonals: per vertex its diagonal neighbours and their quads
    std::vector<std::vector<std::pair<uint32_t, uint32_t>>> diag(nv);
    auto alternating = [&](uint32_t f) {  // a quad whose corners alternate between the classes: its first class-0 corner
        if (t.deg(f) != 4 || locked(f)) return kNone;
        const uint32_t c = t.faceFirst[f];
        const uint8_t k0 = cls[t.idx[c]], k1 = cls[t.idx[c + 1]], k2 = cls[t.idx[c + 2]], k3 = cls[t.idx[c + 3]];
        if (k0 == k1 || k1 == k2 || k2 == k3 || k3 == k0) return kNone;
        return k0 == 0 ? c : c + 1;
    };
    for (uint32_t f = 0; f < nf; ++f) {
        const uint32_t c = alternating(f);
        if (c == kNone) continue;
        const uint32_t a = t.idx[c], b = t.idx[t.next(t.next(c))];
        diag[a].push_back({b, f});
        diag[b].push_back({a, f});
    }
    // Around a side vertex with an odd number of quads, corners and centres cannot alternate. Those
    // vertices are paired up along short paths over the side vertices (neighbours across a quad),
    // and the quads on the paths are the only ones whose corner-centre diagonal may clash.
    std::vector<uint8_t> cut;
    {
        std::vector<std::vector<std::pair<uint32_t, uint32_t>>> sideDiag(nv);
        for (uint32_t f = 0; f < nf; ++f) {
            const uint32_t c = alternating(f);
            if (c == kNone) continue;
            const uint32_t x = t.idx[t.next(c)], y = t.idx[t.prev(c)];
            sideDiag[x].push_back({y, f});
            sideDiag[y].push_back({x, f});
        }
        std::vector<uint32_t> odd;
        for (uint32_t v = 0; v < nv; ++v) {
            if (cls[v] != 1) continue;
            bool closed = t.vertFaces(v) > 0;
            for (uint32_t k = vertEdgeFirst[v]; k < vertEdgeFirst[v + 1]; ++k) closed &= t.edgeFaces(vertEdges[k]) == 2;
            for (uint32_t k = t.vertFirst[v]; k < t.vertFirst[v + 1]; ++k) closed &= alternating(t.faceOf[t.vertCorners[k]]) != kNone;
            if (closed && t.vertFaces(v) % 2) odd.push_back(v);
        }
        auto quad_with_diagonal = [&](uint32_t a, uint32_t b) {
            for (uint32_t k = t.vertFirst[a]; k < t.vertFirst[a + 1]; ++k) {
                const uint32_t c = t.vertCorners[k];
                if (t.deg(t.faceOf[c]) == 4 && t.idx[t.next(t.next(c))] == b) return t.faceOf[c];
            }
            return kNone;
        };
        cut = pair_up(nv, nf, sideDiag, odd, [&](uint32_t v) { return mir[v]; }, [&](uint32_t q) {
            const uint32_t c = alternating(q);
            const uint32_t a = mir[t.idx[c]], b = mir[t.idx[t.next(t.next(c))]];
            return a == kNone || b == kNone ? kNone : quad_with_diagonal(a, b);
        });
    }
    std::vector<uint8_t> col(nv, 2);
    {
        std::vector<uint32_t> queue;
        for (uint32_t s = 0; s < nv; ++s) {
            if (cls[s] != 0 || col[s] != 2) continue;
            col[s] = col[twin(s)] = 0;
            queue.assign({s, twin(s)});
            for (size_t qi = 0; qi < queue.size(); ++qi)
                for (auto [u, f] : diag[queue[qi]]) {
                    if (col[u] != 2 || cut[f]) continue;
                    col[u] = col[queue[qi]] ^ 1;
                    if (col[twin(u)] == 2) col[twin(u)] = col[u];
                    queue.push_back(u);
                    queue.push_back(twin(u));
                }
        }
        // improve: flip a vertex (with its mirror image) when more of its diagonal neighbours agree with it
        for (int sweep = 0; sweep < 32; ++sweep) {
            size_t flips = 0;
            for (uint32_t v = 0; v < nv; ++v) {
                if (cls[v] != 0 || twin(v) < v) continue;
                int same = 0, other = 0;
                for (uint32_t x : {v, twin(v)}) {
                    for (auto [u, f] : diag[x])
                        if (u != v && u != twin(v)) (col[u] == col[x] ? same : other) += 1;
                    if (x == twin(v)) break;
                }
                const double keepGain = weight(v, col[v] == 0) - weight(v, col[v] != 0);  // the pole's preference
                if (same > other || (same == other && keepGain < 0)) {
                    col[v] ^= 1;
                    if (twin(v) != v) col[twin(v)] = col[v];
                    ++flips;
                }
            }
            if (!flips) break;
        }
        // per connected part of the diagonal graph: corners or centres the other way round?
        std::vector<uint8_t> seen(nv, 0);
        std::vector<uint32_t> comp;
        for (uint32_t s = 0; s < nv; ++s) {
            if (cls[s] != 0 || seen[s]) continue;
            comp.clear();
            auto add = [&](uint32_t v) {  // a vertex and its mirror image
                for (uint32_t x : {v, twin(v)})
                    if (!seen[x]) { seen[x] = 1; comp.push_back(x); }
            };
            add(s);
            for (size_t qi = 0; qi < comp.size(); ++qi)
                for (auto [w, f] : diag[comp[qi]]) add(w);
            double score = 0;
            for (uint32_t v : comp) score += weight(v, col[v] == 0) - weight(v, col[v] != 0);
            if (score < 0)
                for (uint32_t v : comp) col[v] ^= 1;
        }
    }

    // ---- blocks: each quad joins its centre's block; blocks are cut at walls ----
    std::vector<uint32_t> key(nf, kNone), uf(nf);
    for (uint32_t f = 0; f < nf; ++f) {
        uf[f] = f;
        if (locked(f)) continue;
        const uint32_t c = alternating(f);
        key[f] = nv + f;  // a block of its own unless it has a centre
        if (c == kNone) continue;
        const uint32_t a = t.idx[c], b = t.idx[t.next(t.next(c))];
        const bool ca = col[a] == 1, cb = col[b] == 1;
        if (ca && cb) key[f] = std::min(a, twin(a)) <= std::min(b, twin(b)) ? a : b;  // two centres: the same pick on both sides
        else if (ca) key[f] = a;
        else if (cb) key[f] = b;
    }
    auto find = [&](uint32_t f) {
        while (uf[f] != f) f = uf[f] = uf[uf[f]];
        return f;
    };
    // Where the classes break (two corners or two sides next to each other, on a mesh with triangles),
    // a row of quads has its two sides across one edge: two such quads glued there make a domino of
    // half a block, whose long sides have the sides as midpoints.
    auto crossing = [&](uint32_t f) {  // a quad with classes corner, corner, side, side: its side-side edge
        if (t.deg(f) != 4 || locked(f) || alternating(f) != kNone) return kNone;
        uint32_t found = kNone, sides = 0, corners = 0;
        for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c) {
            const uint8_t a = cls[t.idx[c]], b = cls[t.idx[t.next(c)]];
            if (a == 1 && b == 1) { found = t.cornerEdge[c]; ++sides; }
            else if (a == 0 && b == 0) ++corners;
        }
        return sides == 1 && corners == 1 ? found : kNone;
    };
    for (uint32_t e = 0; e < ne; ++e) {
        if (wall[e]) continue;
        const uint32_t f1 = t.faceOf[t.edgeCorners[t.edgeFirst[e]]], f2 = t.faceOf[t.edgeCorners[t.edgeFirst[e] + 1]];
        if (key[f1] != kNone && key[f1] == key[f2]) uf[find(f1)] = find(f2);
        else if (crossing(f1) == e && crossing(f2) == e) uf[find(f1)] = find(f2);
    }
    // Where corners and centres clash: a quad with two centres joins both their blocks into one, and
    // a quad without one joins a neighbouring block (the same pick on both sides of the mirror).
    // Left on their own they force extra corners on every block around them.
    {
        for (uint32_t f = 0; f < nf; ++f) {
            if (locked(f)) continue;
            const uint32_t c = alternating(f);
            const bool isOrphan = key[f] >= nv;
            bool twoCentres = false;
            uint32_t other = kNone;
            if (c != kNone) {
                const uint32_t a = t.idx[c], b = t.idx[t.next(t.next(c))];
                twoCentres = col[a] == 1 && col[b] == 1;
                other = key[f] == a ? b : a;
            }
            uint32_t bestOrb = kNone;
            std::vector<uint32_t> best;  // the neighbouring blocks with the lowest mirror pair: both of a pair when it ties
            for (uint32_t cc = t.faceFirst[f]; cc < t.faceFirst[f + 1]; ++cc) {
                const uint32_t e = t.cornerEdge[cc];
                if (wall[e] || t.edgeFaces(e) != 2) continue;
                const uint32_t g = t.faceOf[t.edgeCorners[t.edgeFirst[e]]] == f ? t.faceOf[t.edgeCorners[t.edgeFirst[e] + 1]] : t.faceOf[t.edgeCorners[t.edgeFirst[e]]];
                if (locked(g) || key[g] >= nv) continue;
                if (twoCentres && key[g] == other) uf[find(f)] = find(g);
                if (isOrphan) {
                    const uint32_t o = std::min(key[g], twin(key[g]));
                    if (o < bestOrb) { bestOrb = o; best.assign(1, g); }
                    else if (o == bestOrb) best.push_back(g);
                }
            }
            for (uint32_t g : best) uf[find(f)] = find(g);
        }
    }

    // ---- coarse faces: the outline of each block through its coarse vertices ----
    struct Poly {
        std::vector<uint32_t> corner;  // topology corners (their vertex, UV and normal)
        std::vector<uint8_t> wallSide; // side i (corner i to i + 1) lies on a wall
        uint32_t material = 0;
        bool dead = false;
    };
    std::vector<Poly> polys;
    std::vector<uint8_t> keepV(nv, 0);
    std::vector<uint8_t> forced(nv, 0);  // kept because a block came out bad
    for (int attempt = 0;; ++attempt) {
        auto coarse = [&](uint32_t e) {
            if (wall[e]) return true;
            return find(t.faceOf[t.edgeCorners[t.edgeFirst[e]]]) != find(t.faceOf[t.edgeCorners[t.edgeFirst[e] + 1]]);
        };
        for (uint32_t v = 0; v < nv; ++v) {
            uint32_t n = 0;
            for (uint32_t k = vertEdgeFirst[v]; k < vertEdgeFirst[v + 1]; ++k) n += coarse(vertEdges[k]);
            keepV[v] = fixed[v] || forced[v] || n >= 3 || n == 1;
        }
        std::vector<std::vector<uint32_t>> members(nf);
        for (uint32_t f = 0; f < nf; ++f)
            if (!locked(f)) members[find(f)].push_back(f);
        polys.clear();
        bool bad = false;
        std::unordered_map<uint32_t, uint32_t> out;  // outline: vertex -> the corner leaving it
        for (uint32_t r = 0; r < nf; ++r) {
            if (members[r].empty()) continue;
            out.clear();
            bool ok = true;
            size_t count = 0;
            for (uint32_t f : members[r])
                for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c)
                    if (coarse(t.cornerEdge[c])) {
                        ok &= out.emplace(t.idx[c], c).second;  // twice from one vertex: the block pinches
                        ++count;
                    }
            Poly p;
            p.material = m.faces[members[r][0]].material;
            uint32_t start = kNone;  // the kept vertex with the lowest number starts the outline
            for (const auto& [v, c] : out)
                if (keepV[v] && (start == kNone || v < t.idx[start])) start = c;
            if (ok && start != kNone) {
                bool sideWall = false;
                size_t steps = 0;
                uint32_t c = start;
                do {
                    if (keepV[t.idx[c]]) {
                        if (!p.corner.empty()) p.wallSide.push_back(sideWall);
                        p.corner.push_back(c);
                        sideWall = false;
                    }
                    sideWall |= wall[t.cornerEdge[c]] != 0;
                    auto it = out.find(t.idx[t.next(c)]);
                    if (it == out.end()) { ok = false; break; }
                    c = it->second;
                    ++steps;
                } while (c != start && steps <= count);
                ok &= c == start && steps == count;  // one closed outline through every outline edge
                p.wallSide.push_back(sideWall);
                ok &= p.corner.size() >= 3;
            } else {
                ok = false;
            }
            if (!ok && attempt >= 8) {  // still not a disc: its faces stay as they are
                for (uint32_t f : members[r]) {
                    Poly q;
                    q.material = m.faces[f].material;
                    for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c) {
                        q.corner.push_back(c);
                        q.wallSide.push_back(wall[t.cornerEdge[c]]);
                    }
                    polys.push_back(std::move(q));
                }
                continue;
            }
            if (!ok) {  // not a disc: its faces become blocks of their own, with all their corners kept
                bad = true;
                for (uint32_t f : members[r]) {
                    uf[f] = f;
                    key[f] = nv + f;
                    if (members[r].size() == 1 || attempt > 2)
                        for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c) forced[t.idx[c]] = 1;
                }
                continue;
            }
            polys.push_back(std::move(p));
        }
        if (!bad || attempt >= 8) break;
    }
    if (polys.empty() || polys.size() * 10 > (size_t)nf * 9) res.stop = "nothing to merge";

    // the surface normal at each vertex (area-weighted, from the fine faces around it)
    std::vector<Vec3d> vnormal(nv, Vec3d{0, 0, 0});
    for (uint32_t f = 0; f < nf; ++f)
        for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c) {
            const Vec3d& o = P[t.idx[c]];
            const Vec3d n = vcross(vsub(P[t.idx[t.next(c)]], o), vsub(P[t.idx[t.prev(c)]], o));
            for (int d = 0; d < 3; ++d) vnormal[t.idx[c]][d] += n[d];
        }

    // ---- odd faces: merge neighbours and cut the result into quads ----
    // Every cut is made on a polygon and on its mirror image together (the image cut the mirrored
    // way, not chosen again), and a polygon that is its own mirror image only gets a symmetric cut.
    auto vert = [&](uint32_t c) { return t.idx[c]; };
    auto pos = [&](uint32_t c) { return &P[t.idx[c]]; };
    std::map<std::vector<uint32_t>, uint32_t> byVerts;  // sorted vertices -> polygon (for mirror look-ups)
    std::map<std::pair<uint32_t, uint32_t>, std::vector<std::pair<uint32_t, uint32_t>>> sides;  // edge -> (polygon, side)
    auto sorted_verts = [&](const Poly& p) {
        std::vector<uint32_t> s;
        for (uint32_t c : p.corner) s.push_back(vert(c));
        std::sort(s.begin(), s.end());
        return s;
    };
    auto index = [&]() {
        byVerts.clear();
        sides.clear();
        for (uint32_t i = 0; i < polys.size(); ++i) {
            const Poly& p = polys[i];
            if (p.dead) continue;
            byVerts[sorted_verts(p)] = i;
            for (uint32_t k = 0; k < p.corner.size(); ++k) {
                uint32_t a = vert(p.corner[k]), b = vert(p.corner[(k + 1) % p.corner.size()]);
                if (a > b) std::swap(a, b);
                sides[{a, b}].push_back({i, k});
            }
        }
    };
    auto neighbour = [&](uint32_t i, uint32_t k) {  // the polygon across side k, when it is not a wall and has one
        const Poly& p = polys[i];
        if (p.wallSide[k]) return kNone;
        uint32_t a = vert(p.corner[k]), b = vert(p.corner[(k + 1) % p.corner.size()]);
        if (a > b) std::swap(a, b);
        const auto& list = sides[{a, b}];
        if (list.size() != 2) return kNone;
        const uint32_t j = list[0].first == i ? list[1].first : list[0].first;
        return polys[j].dead || j == i ? kNone : j;
    };
    // Polygons p and q glued along p's side from vertex a to b (q runs it from b to a).
    auto glue = [&](const Poly& p, const Poly& q, uint32_t a, uint32_t b, Poly& u) {
        const size_t np = p.corner.size(), nq = q.corner.size();
        size_t ib = np, ja = nq;
        for (size_t k = 0; k < np; ++k) if (vert(p.corner[k]) == b && vert(p.corner[(k + np - 1) % np]) == a) ib = k;
        for (size_t k = 0; k < nq; ++k) if (vert(q.corner[k]) == a && vert(q.corner[(k + nq - 1) % nq]) == b) ja = k;
        if (ib == np || ja == nq) return false;
        u = {};
        u.material = p.material;
        for (size_t k = 0; k < np; ++k) {  // b round p to a
            u.corner.push_back(p.corner[(ib + k) % np]);
            u.wallSide.push_back(k + 1 < np ? p.wallSide[(ib + k) % np] : q.wallSide[ja]);
        }
        for (size_t k = 1; k + 1 < nq; ++k) {  // then round q, strictly between a and b
            u.corner.push_back(q.corner[(ja + k) % nq]);
            u.wallSide.push_back(q.wallSide[(ja + k) % nq]);
        }
        const std::vector<uint32_t> vs = sorted_verts(u);
        return std::adjacent_find(vs.begin(), vs.end()) == vs.end();  // simple: no vertex twice
    };
    // A group of neighbours glued into one polygon, in order (each one next to one before it).
    auto union_of = [&](const std::vector<uint32_t>& parts, Poly& u) {
        u = polys[parts[0]];
        for (size_t k = 1; k < parts.size(); ++k) {
            const Poly& q = polys[parts[k]];
            bool glued = false;
            for (uint32_t s = 0; s < u.corner.size() && !glued; ++s) {
                Poly w;
                if (glue(u, q, vert(u.corner[s]), vert(u.corner[(s + 1) % u.corner.size()]), w)) { u = std::move(w); glued = true; }
            }
            if (!glued) return false;
        }
        return true;
    };
    auto points = [&](const Poly& u) {
        std::vector<const Vec3d*> pts;
        for (uint32_t c : u.corner) pts.push_back(pos(c));
        return pts;
    };
    auto face_verts = [&](const Poly& u, const std::array<int, 4>& f) {
        std::vector<uint32_t> v;
        for (int k = 0; k < (f[3] < 0 ? 3 : 4); ++k) v.push_back(vert(u.corner[f[k]]));
        std::sort(v.begin(), v.end());
        return v;
    };
    auto symmetric_cut = [&](const Poly& u, const Split& s) {  // u is its own mirror image; so must the cut be
        std::set<std::vector<uint32_t>> faces;
        for (const auto& f : s.faces) faces.insert(face_verts(u, f));
        for (const auto& f : s.faces) {
            std::vector<uint32_t> v = face_verts(u, f);
            for (uint32_t& x : v) {
                if (mir[x] == kNone) return false;
                x = mir[x];
            }
            std::sort(v.begin(), v.end());
            if (!faces.count(v)) return false;
        }
        return true;
    };
    // The best cut of u; for a polygon that is its own mirror image, the best symmetric one.
    auto choose = [&](const Poly& u, int maxTris, bool self, const CutRules& rules) {
        if (!self) return best_split(points(u), maxTris, &rules);
        Split best;
        if (u.corner.size() > 10) return best;
        for (Split& s : every_split(points(u), maxTris, &rules))
            if (symmetric_cut(u, s) && (s.worst > best.worst + 1e-12 || (std::abs(s.worst - best.worst) <= 1e-12 && s.sum > best.sum)))
                best = std::move(s);
        return best;
    };
    // The cut s of u made on u's mirror image mu.
    auto mirrored = [&](const Poly& u, const Split& s, const Poly& mu, Split& ms) {
        std::unordered_map<uint32_t, int> at;
        for (int k = 0; k < (int)mu.corner.size(); ++k) at[vert(mu.corner[k])] = k;
        ms = s;
        for (auto& f : ms.faces) {
            const int n = f[3] < 0 ? 3 : 4;
            std::array<int, 4> g{-1, -1, -1, -1};
            for (int k = 0; k < n; ++k) {  // reversed: the mirror turns the winding round
                const uint32_t w = mir[vert(u.corner[f[n - 1 - k]])];
                auto it = w == kNone ? at.end() : at.find(w);
                if (it == at.end()) return false;
                g[k] = it->second;
            }
            f = g;
        }
        return true;
    };
    auto apply_split = [&](const Poly& u, const Split& s) {
        for (const auto& f : s.faces) {
            Poly q;
            q.material = u.material;
            const int n = f[3] < 0 ? 3 : 4;
            for (int k = 0; k < n; ++k) {
                q.corner.push_back(u.corner[f[k]]);
                const int next = f[(k + 1) % n];
                q.wallSide.push_back(next == (f[k] + 1) % (int)u.corner.size() ? u.wallSide[f[k]] : 0);  // a new cut is no wall
            }
            const uint32_t id = (uint32_t)polys.size();  // keep the look-ups current for the cuts after this one
            byVerts[sorted_verts(q)] = id;
            for (uint32_t k = 0; k < q.corner.size(); ++k) {
                uint32_t a = vert(q.corner[k]), b = vert(q.corner[(k + 1) % q.corner.size()]);
                if (a > b) std::swap(a, b);
                sides[{a, b}].push_back({id, k});
            }
            polys.push_back(std::move(q));
        }
    };
    // The rules for cutting u, made of the live polygons in `parts`: no chord along an edge of any
    // other live polygon, and every face turned the way u is.
    auto rules_for = [&](const Poly& u, const std::vector<uint32_t>& parts) {
        CutRules r;
        const size_t n = u.corner.size();
        r.blocked.assign(n * n, 0);
        for (size_t x = 0; x < n; ++x)
            for (size_t y = 0; y < n; ++y) {
                if (x == y) continue;
                uint32_t a = vert(u.corner[x]), b = vert(u.corner[y]);
                if (a > b) std::swap(a, b);
                auto it = sides.find({a, b});
                if (it == sides.end()) continue;
                for (auto [pi, side] : it->second)
                    if (!polys[pi].dead && std::find(parts.begin(), parts.end(), pi) == parts.end()) r.blocked[x * n + y] = 1;
            }
        for (size_t k = 0; k < n; ++k) {
            r.up.push_back(vnormal[vert(u.corner[k])]);
            const Vec3d& o = *pos(u.corner[k]);
            const Vec3d c = vcross(vsub(*pos(u.corner[(k + 1) % n]), o), vsub(*pos(u.corner[(k + n - 1) % n]), o));
            for (int d = 0; d < 3; ++d) r.own[d] += c[d];
        }
        return r;
    };
    auto mirror_poly = [&](uint32_t i) {  // the live polygon on the mirror images of i's corners, or kNone
        std::vector<uint32_t> s;
        for (uint32_t c : polys[i].corner) {
            if (mir[vert(c)] == kNone) return kNone;
            s.push_back(mir[vert(c)]);
        }
        std::sort(s.begin(), s.end());
        auto it = byVerts.find(s);
        return it == byVerts.end() || polys[it->second].dead ? kNone : it->second;
    };
    // Would the faces of cut s of u run an edge the same way as a live polygon outside `parts`?
    // (Two faces on the same side of an edge: they overlap.)
    auto overlaps = [&](const Poly& u, const Split& s, const std::vector<uint32_t>& parts) {
        for (const auto& f : s.faces) {
            const int n = f[3] < 0 ? 3 : 4;
            for (int k = 0; k < n; ++k) {
                const uint32_t a = vert(u.corner[f[k]]), b = vert(u.corner[f[(k + 1) % n]]);
                auto it = sides.find({std::min(a, b), std::max(a, b)});
                if (it == sides.end()) continue;
                for (auto [pi, side] : it->second) {
                    const Poly& p = polys[pi];
                    if (p.dead || std::find(parts.begin(), parts.end(), pi) != parts.end()) continue;
                    if (vert(p.corner[side]) == a && vert(p.corner[(side + 1) % p.corner.size()]) == b) return true;
                }
            }
        }
        return false;
    };
    // Merges a group of neighbours, cuts it with at most maxTris triangles, no face worse than
    // `least`, and does the same to the group's mirror image. 1: done without a mirror image or as
    // its own; 2: done with the image; 0: no cut good enough; -1: the mirror side cannot follow.
    auto process = [&](const std::vector<uint32_t>& parts, int maxTris, double least, bool anyCut = false) {
        Poly u;
        if (!union_of(parts, u)) return 0;
        std::vector<uint32_t> image;
        size_t imaged = 0;
        for (uint32_t p : parts) {
            image.push_back(anyCut ? kNone : mirror_poly(p));
            imaged += image.back() != kNone;
        }
        const bool single = imaged == 0;
        const bool self = !single && std::is_permutation(image.begin(), image.end(), parts.begin());
        if (!single && !self) {
            if (imaged != parts.size()) return -1;
            for (uint32_t p : image)
                if (std::find(parts.begin(), parts.end(), p) != parts.end()) return -1;
        }
        const Split s = choose(u, maxTris, self, rules_for(u, parts));
        if (s.worst < least || s.worst <= 0) return self ? -1 : 0;
        if (overlaps(u, s, parts)) return 0;
        Poly mu;
        Split ms;
        if (!single && !self) {
            if (!union_of(image, mu) || !mirrored(u, s, mu, ms)) return -1;
            const CutRules mr = rules_for(mu, image);
            for (const auto& f : ms.faces)
                if (!mr.allows(points(mu), f)) return -1;
            if (overlaps(mu, ms, image)) return -1;
            std::set<std::pair<uint32_t, uint32_t>> runs;  // and the two cuts may not overlap each other (on the mirror plane)
            for (const auto& f : s.faces)
                for (int k = 0, n = f[3] < 0 ? 3 : 4; k < n; ++k) runs.insert({vert(u.corner[f[k]]), vert(u.corner[f[(k + 1) % n]])});
            for (const auto& f : ms.faces)
                for (int k = 0, n = f[3] < 0 ? 3 : 4; k < n; ++k)
                    if (runs.count({vert(mu.corner[f[k]]), vert(mu.corner[f[(k + 1) % n]])})) return -1;
        }
        for (uint32_t p : parts) polys[p].dead = true;
        apply_split(u, s);
        if (single || self) return 1;
        for (uint32_t p : image) polys[p].dead = true;
        apply_split(mu, ms);
        return 2;
    };
    constexpr double kGoodEnough = 0.3;

    index();
    for (uint32_t i = 0, n = (uint32_t)polys.size(); i < n; ++i)  // even polygons over four corners: cut into quads
        if (!polys[i].dead && polys[i].corner.size() >= 6 && polys[i].corner.size() % 2 == 0) process({i}, 0, 1e-9);
    struct Merge { double worst, sum; std::vector<uint32_t> parts; };
    for (int round = 0; round < 2; ++round) {
        index();
        std::vector<Merge> merges;
        for (uint32_t i = 0; i < polys.size(); ++i) {
            if (polys[i].dead || polys[i].corner.size() % 2 == 0) continue;
            for (uint32_t k = 0; k < polys[i].corner.size(); ++k) {
                const uint32_t j = neighbour(i, k);
                if (j == kNone) continue;
                const uint32_t a = vert(polys[i].corner[k]), b = vert(polys[i].corner[(k + 1) % polys[i].corner.size()]);
                Poly u;
                if (!glue(polys[i], polys[j], a, b, u)) continue;
                if (round == 0) {  // two odd neighbours
                    if (polys[j].corner.size() % 2 == 0 || j < i) continue;
                    const CutRules rules = rules_for(u, {i, j});
                    const Split s = best_split(points(u), 0, &rules);
                    if (s.worst >= kGoodEnough) merges.push_back({s.worst, s.sum, {i, j}});
                    continue;
                }
                if (polys[j].corner.size() % 2) continue;  // across one even polygon to another odd one
                for (uint32_t k2 = 0; k2 < polys[j].corner.size(); ++k2) {
                    const uint32_t l = neighbour(j, k2);
                    if (l == kNone || l <= i || polys[l].corner.size() % 2 == 0) continue;
                    Poly u2;
                    if (!glue(u, polys[l], vert(polys[j].corner[k2]), vert(polys[j].corner[(k2 + 1) % polys[j].corner.size()]), u2)) continue;
                    const CutRules rules = rules_for(u2, {i, j, l});
                    const Split s = best_split(points(u2), 0, &rules);
                    if (s.worst >= kGoodEnough) merges.push_back({s.worst, s.sum, {i, j, l}});
                }
            }
        }
        std::sort(merges.begin(), merges.end(), [](const Merge& x, const Merge& y) {
            return x.worst != y.worst ? x.worst > y.worst : x.sum > y.sum;
        });
        for (const Merge& mg : merges) {
            bool live = true;
            for (uint32_t p : mg.parts) live &= !polys[p].dead;
            if (!live) continue;
            const int r = process(mg.parts, 0, kGoodEnough);
            if (r > 0) res.paired += 2 * r;
            else if (r < 0) ++res.unpaired;
        }
    }
    index();
    for (uint32_t i = 0, n = (uint32_t)polys.size(); i < n; ++i) {  // what is left over four corners: cut, a triangle when odd
        if (polys[i].dead || polys[i].corner.size() <= 4) continue;
        const int odd = (int)(polys[i].corner.size() % 2);
        if (process({i}, odd, 1e-9) == 0) process({i}, (int)polys[i].corner.size(), 1e-9);
        if (!polys[i].dead && mirror_poly(i) == i && process({i}, (int)polys[i].corner.size(), 1e-9, true) > 0)
            ++res.unpaired;  // on the mirror plane with no symmetric cut: an asymmetric one beats a big polygon
    }

    // ---- the new mesh ----
    InMesh out;
    out.name = m.name;
    std::vector<uint8_t> used(nv, 0);
    auto emit = [&](uint32_t material, auto&& corners) {
        const uint32_t first = (uint32_t)out.cornerPos.size();
        uint32_t n = 0;
        for (uint32_t c : corners) {
            const uint32_t s = fs.srcCorner[c];
            out.cornerPos.push_back(m.cornerPos[s]);
            out.cornerUv.push_back(m.cornerUv[s]);
            if (hasN) out.cornerNrm.push_back(m.cornerNrm[s]);
            if (hasSrc) out.cornerSrc.push_back(m.cornerSrc[s]);
            used[t.idx[c]] = 1;
            ++n;
        }
        out.faces.push_back({first, n, material});
    };
    for (uint32_t f = 0; f < nf; ++f) {
        if (!locked(f)) continue;
        std::vector<uint32_t> cs;
        for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c) cs.push_back(c);
        emit(m.faces[f].material, cs);
    }
    for (const Poly& p : polys) {
        if (p.dead) continue;
        emit(p.material, p.corner);
        res.triangles += p.corner.size() == 3;
    }
    for (uint32_t v = 0; v < nv; ++v) res.removed += !used[v] && vertEdgeFirst[v + 1] > vertEdgeFirst[v];
    out.positions = std::move(m.positions);
    out.normals = std::move(m.normals);
    out.uvs = std::move(m.uvs);
    compact(out);
    triangulate(out);
    m = std::move(out);
    return res;
}

void count_faces(const InScene& scene, size_t& faces, size_t& tris) {
    faces = tris = 0;
    for (const InMesh& m : scene.meshes) { faces += m.faces.size(); tris += m.tris.size(); }
}

}  // namespace

SubdivReport unsubdivide_scene(InScene& scene, const UnsubdivideOptions& opt) {
    const auto t0 = Clock::now();
    SubdivReport rep;
    count_faces(scene, rep.facesBefore, rep.trisBefore);
    const double diag = scene_diagonal(scene);
    double sumSq = 0;
    size_t rows = 0;
    for (InMesh& m : scene.meshes) {
        SubdivReport::Mesh mr;
        mr.name = m.name;
        mr.facesBefore = m.faces.size();
        double meshSq = 0;
        size_t meshRows = 0;
        for (int level = 0; level < opt.levels; ++level) {
            const FaceSet fs = faces_of(m);
            const Topo t = topo_for(fs, m.cornerPos, m.positions.size());
            Level R;
            reverse_level(m, fs, t, R);
            mr.reason = R.reason;
            mr.keptFaces = R.keptFaces;
            mr.keptSets = R.keptSets;
            rep.materialMismatch += R.materialMismatch;
            if (R.cfFirst.size() == 1) {  // nothing reversed exactly at this level
                if (opt.exactOnly || m.faces.empty()) break;
                const CoarsenResult d = coarsen_level(m, opt.keep, opt.symmetry);
                mr.stop = d.stop;
                if (!d.stop.empty()) break;
                mr.removed += d.removed;
                mr.paired += d.paired;
                mr.triangles = d.triangles;
                mr.unpaired += d.unpaired;
                mr.mirrorAxis = d.axis;
                mr.mirrored = d.mirrored;
                ++mr.approxLevels;
                continue;
            }
            if (opt.solveCage) {
                const Fit fit = solve_cage(m, fs, R, &m.positions);
                meshSq += fit.sumSq;
                meshRows += fit.rows;
                mr.fitMax = std::max(mr.fitMax, fit.worst / diag);
            }
            rebuild_mesh(m, fs, R);
            ++mr.levels;
        }
        mr.facesAfter = m.faces.size();
        if (meshRows) mr.fitRms = std::sqrt(meshSq / meshRows) / diag;
        sumSq += meshSq;
        rows += meshRows;
        rep.fitMax = std::max(rep.fitMax, mr.fitMax);
        rep.meshes.push_back(std::move(mr));
    }
    if (rows) rep.fitRms = std::sqrt(sumSq / rows) / diag;
    count_faces(scene, rep.facesAfter, rep.trisAfter);
    rep.ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    return rep;
}

SubdivReport subdivide_scene(InScene& scene, int levels) {
    const auto t0 = Clock::now();
    SubdivReport rep;
    count_faces(scene, rep.facesBefore, rep.trisBefore);
    for (InMesh& m : scene.meshes) {
        SubdivReport::Mesh mr;
        mr.name = m.name;
        mr.facesBefore = m.faces.size();
        for (int level = 0; level < levels; ++level) { subdivide_mesh(m); ++mr.levels; }
        mr.facesAfter = m.faces.size();
        rep.meshes.push_back(std::move(mr));
    }
    count_faces(scene, rep.facesAfter, rep.trisAfter);
    rep.ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    return rep;
}
