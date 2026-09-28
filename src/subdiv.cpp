// udimpacker - Catmull-Clark subdivision, forward and reverse.
#include "subdiv.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <type_traits>
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
    const bool hasN = !m.cornerNrm.empty();
    auto corner = [&](uint32_t c) {
        const uint32_t oc = fs.srcCorner[c];
        out.cornerPos.push_back(m.cornerPos[oc]);
        out.cornerUv.push_back(m.cornerUv[oc]);
        if (hasN) out.cornerNrm.push_back(m.cornerNrm[oc]);
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
// Approximate reverse subdivision for meshes that are not a subdivision result: Blender's
// un-subdivide (source/blender/bmesh/tools/bmesh_decimate_unsubdivide.cc), extended for per-corner
// data. A flood fill over edges colours the vertices like a checkerboard and every other one is
// dissolved: each face around it gives up that corner (the ear it sheds) and the ears join into one
// polygon of its neighbours. One pass halves the vertices in a diamond pattern; two passes are one
// reverse subdivision step on a regular grid. Nothing moves and nothing is interpolated: every
// surviving corner keeps its position, UV and normal. Vertices that cannot go (valence 5 and up,
// non-manifold fans, locked materials) take no part in the fill; where the pattern does not fit
// around them, triangles appear. Two additions to Blender: the fill picks its colours so those
// vertices stay, and afterwards triangle pairs that make a good quad are joined, as Blender's
// Tris to Quads does.
//
// Blender dissolves a vertex on the mesh border as the fan of its one side. A UV seam, hard normal
// edge or material border through a vertex is treated the same way here: each side is dissolved on
// its own, so the seam stays an edge and is halved like everything else.
//
// With `symmetric`, a mesh with a mirror (see find_mirror) stays symmetric: the fill colours a
// vertex and its mirror image alike, a vertex is dissolved only together with its image (the
// first is undone when the second cannot follow), and triangle pairs join only with their image.

struct DissolveResult {
    size_t passes = 0, dissolved = 0, joined = 0;
    std::string stop;     // why a pass found nothing to dissolve (empty = both passes did work)
    int axis = -1;        // mirror axis (-1 = none)
    double mirrored = 0;  // share of the vertices with a mirror image
    size_t unpaired = 0;  // dissolves and joins left out because the mirror side could not follow
};

DissolveResult dissolve_level(InMesh& m, const std::vector<uint8_t>& lockedMaterial, bool symmetric) {
    struct Corner { uint32_t pos, uv, nrm; };
    struct Face { std::vector<Corner> c; uint32_t material = 0; bool dead = false; };
    const bool hasN = !m.cornerNrm.empty();
    const uint32_t nv = (uint32_t)m.positions.size();
    DissolveResult res;
    const std::vector<uint32_t> mir = symmetric ? find_mirror(m, res.axis) : std::vector<uint32_t>(nv, kNone);
    std::vector<Face> faces(m.faces.size());
    std::vector<std::vector<uint32_t>> vertFaces(nv);  // live faces per position
    for (uint32_t f = 0; f < m.faces.size(); ++f) {
        faces[f].material = m.faces[f].material;
        for (uint32_t c = m.faces[f].first; c < m.faces[f].first + m.faces[f].count; ++c) {
            faces[f].c.push_back({m.cornerPos[c], m.cornerUv[c], hasN ? m.cornerNrm[c] : 0});
            vertFaces[m.cornerPos[c]].push_back(f);
        }
    }

    // One face around a vertex: its corner there, the neighbours before and after it, and the face
    // across the edge to `next` (which runs the other way there).
    struct Wedge { uint32_t face, corner, prev, next, succ; bool pred; };
    std::vector<Wedge> wedges, ring;              // ring: the faces around the vertex in order
    std::vector<std::vector<Corner>> polys;       // the polygons the ears join into, one per side
    std::vector<uint32_t> polyMat;
    std::vector<std::vector<uint32_t>> newFaces;  // positions of every face the dissolve leaves behind
    auto corner_at = [&](const Wedge& w, int d) -> const Corner& {
        const std::vector<Corner>& c = faces[w.face].c;
        return c[(w.corner + c.size() + d) % c.size()];
    };
    auto locked = [&](uint32_t mat) { return mat < lockedMaterial.size() && lockedMaterial[mat]; };

    // Can `v` be dissolved: an interior vertex of valence 2-4 (Blender: 3-4 as a fan, 2 as a chain,
    // which the fan covers) or a border vertex of valence 3, with at most two cuts. `full` also
    // builds the result and checks it is a clean manifold without duplicate faces; the surroundings
    // change as neighbours dissolve, so, as in Blender, the fill only uses the topology test.
    auto plan = [&](uint32_t v, bool full) {
        const std::vector<uint32_t>& fs = vertFaces[v];
        const size_t k = fs.size();
        if (k < 2 || k > 4) return false;
        wedges.resize(k);
        for (size_t i = 0; i < k; ++i) {
            const Face& F = faces[fs[i]];
            if (locked(F.material)) return false;
            const uint32_t n = (uint32_t)F.c.size();
            uint32_t at = kNone;
            for (uint32_t j = 0; j < n; ++j)
                if (F.c[j].pos == v) {
                    if (at != kNone) return false;
                    at = j;
                }
            wedges[i] = {fs[i], at, F.c[(at + n - 1) % n].pos, F.c[(at + 1) % n].pos, kNone, false};
            if (wedges[i].prev == wedges[i].next) return false;
        }
        for (size_t i = 0; i < k; ++i)
            for (size_t j = 0; j < k; ++j) {
                if (j == i) continue;
                if (wedges[j].next == wedges[i].next || wedges[j].prev == wedges[i].prev) return false;  // non-manifold or flipped
                if (wedges[j].prev == wedges[i].next) { wedges[i].succ = (uint32_t)j; wedges[j].pred = true; }
            }
        size_t start = 0, starts = 0;
        for (size_t i = 0; i < k; ++i)
            if (!wedges[i].pred) { start = i; ++starts; }
        const bool closed = starts == 0;
        if (!closed && (starts != 1 || k != 2)) return false;
        ring.clear();
        for (size_t i = start;;) {
            for (const Wedge& w : ring)
                if (w.face == wedges[i].face) return false;
            ring.push_back(wedges[i]);
            if (ring.size() == k) break;
            if (wedges[i].succ == kNone) return false;
            i = wedges[i].succ;
        }
        if (closed ? ring.back().succ != start : ring.back().succ != kNone) return false;

        // cuts: a UV seam, hard normal edge or material border between two faces of the ring
        auto cut = [&](const Wedge& a, const Wedge& b) {  // b follows a across the edge to a.next
            const Corner &an = corner_at(a, 1), &bp = corner_at(b, -1), &av = corner_at(a, 0), &bv = corner_at(b, 0);
            return faces[a.face].material != faces[b.face].material || an.uv != bp.uv || an.nrm != bp.nrm ||
                   av.uv != bv.uv || av.nrm != bv.nrm;
        };
        size_t cutAt[2] = {0, 0}, cuts = 0;  // cutAt: first face of a side
        for (size_t i = 0; i + (closed ? 0 : 1) < k; ++i)
            if (cut(ring[i], ring[(i + 1) % k])) {
                if (!closed || cuts == 2) return false;
                cutAt[cuts++] = (i + 1) % k;
            }
        if (cuts == 1) return false;
        if (!full) return true;

        // the polygon of one side: its neighbours, winding opposite to the ears it replaces
        polys.clear();
        polyMat.clear();
        auto side = [&](size_t b, size_t len, bool wrap) {
            std::vector<Corner> p;
            if (!wrap) p.push_back(corner_at(ring[(b + len - 1) % k], 1));
            for (size_t s = len; s-- > 0;) p.push_back(corner_at(ring[(b + s) % k], -1));
            if (p.size() < 3) return;  // a single face: giving up the corner is all there is to do
            polys.push_back(std::move(p));
            polyMat.push_back(faces[ring[b].face].material);
        };
        if (!closed) side(0, k, false);
        else if (!cuts) side(0, k, true);
        else {
            const size_t len = (cutAt[1] + k - cutAt[0]) % k;
            side(cutAt[0], len, false);
            side(cutAt[1], k - len, false);
        }

        // what is left behind: every face of the ring minus its corner at v (a triangle goes), and
        // the polygons. Each new edge may be used once per direction, twice in all, and no face
        // may repeat another one.
        newFaces.clear();
        for (const Wedge& w : ring) {
            const std::vector<Corner>& c = faces[w.face].c;
            if (c.size() <= 3) continue;
            newFaces.emplace_back();
            for (uint32_t j = 0; j < c.size(); ++j)
                if (j != w.corner) newFaces.back().push_back(c[j].pos);
        }
        for (const std::vector<Corner>& p : polys) {
            newFaces.emplace_back();
            for (const Corner& c : p) newFaces.back().push_back(c.pos);
        }
        auto in_ring = [&](uint32_t f) {
            for (const Wedge& w : ring)
                if (w.face == f) return true;
            return false;
        };
        auto count_edge = [](const auto& verts, uint32_t a, uint32_t b, size_t& same, size_t& opp) {
            for (size_t j = 0; j < verts.size(); ++j) {
                const uint32_t x = verts[j], y = verts[(j + 1) % verts.size()];
                same += x == a && y == b;
                opp += x == b && y == a;
            }
        };
        auto same_set = [](std::vector<uint32_t> x, std::vector<uint32_t> y) {
            if (x.size() != y.size()) return false;
            std::sort(x.begin(), x.end());
            std::sort(y.begin(), y.end());
            return x == y;
        };
        std::vector<uint32_t> ext;
        for (size_t i = 0; i < newFaces.size(); ++i) {
            const std::vector<uint32_t>& F = newFaces[i];
            for (size_t a = 0; a < F.size(); ++a)
                for (size_t b = a + 1; b < F.size(); ++b)
                    if (F[a] == F[b]) return false;
            for (size_t j = 0; j < F.size(); ++j) {
                const uint32_t a = F[j], b = F[(j + 1) % F.size()];
                size_t same = 0, opp = 0;
                for (uint32_t g : vertFaces[a]) {
                    if (in_ring(g)) continue;
                    ext.clear();
                    for (const Corner& c : faces[g].c) ext.push_back(c.pos);
                    count_edge(ext, a, b, same, opp);
                }
                for (size_t o = 0; o < newFaces.size(); ++o)
                    if (o != i) count_edge(newFaces[o], a, b, same, opp);
                if (same || opp > 1) return false;
            }
            for (uint32_t g : vertFaces[F[0]]) {
                if (in_ring(g)) continue;
                ext.clear();
                for (const Corner& c : faces[g].c) ext.push_back(c.pos);
                if (same_set(ext, F)) return false;
            }
            for (size_t o = i + 1; o < newFaces.size(); ++o)
                if (same_set(newFaces[o], F)) return false;
        }
        return true;
    };

    // Dissolve `v` as planned: the ring faces give up their corner at v, the polygons join in. With
    // `u`, what it changes is saved first so `restore` can put it back.
    struct Undo {
        size_t faceCount = 0;
        std::vector<std::pair<uint32_t, Face>> faces;
        std::vector<std::pair<uint32_t, std::vector<uint32_t>>> vertFaces;
    } undo;
    auto dissolve = [&](uint32_t v, Undo* u) {
        if (u) {
            u->faceCount = faces.size();
            u->faces.clear();
            u->vertFaces.clear();
            for (const Wedge& w : ring) {
                u->faces.push_back({w.face, faces[w.face]});
                for (const Corner& c : faces[w.face].c) {
                    bool saved = false;
                    for (const auto& [x, list] : u->vertFaces) saved |= x == c.pos;
                    if (!saved) u->vertFaces.push_back({c.pos, vertFaces[c.pos]});
                }
            }
        }
        for (const Wedge& w : ring) {
            Face& F = faces[w.face];
            if (F.c.size() > 3) {
                F.c.erase(F.c.begin() + w.corner);
                continue;
            }
            F.dead = true;
            for (const Corner& c : F.c) {
                if (c.pos == v) continue;
                std::vector<uint32_t>& vf = vertFaces[c.pos];
                vf.erase(std::find(vf.begin(), vf.end(), w.face));
            }
        }
        vertFaces[v].clear();
        for (size_t i = 0; i < polys.size(); ++i) {
            const uint32_t id = (uint32_t)faces.size();
            faces.push_back({std::move(polys[i]), polyMat[i]});
            for (const Corner& c : faces[id].c) vertFaces[c.pos].push_back(id);
        }
    };
    auto restore = [&](Undo& u) {
        for (auto& [f, F] : u.faces) faces[f] = std::move(F);
        faces.resize(u.faceCount);
        for (auto& [v, list] : u.vertFaces) vertFaces[v] = std::move(list);
    };

    // Two passes of Blender's fill: a flood fill over the vertices that can go colours them in
    // alternation, and one colour is dissolved. Blender dissolves the colour its seed does not
    // have. Here each region of the fill dissolves the colour that keeps the vertices around it
    // that cannot go (poles, seam junctions, border corners), as a real subdivision would: its
    // extraordinary vertices are always original ones. A pole on the dissolved colour stays behind
    // with a fan of triangles around it.
    std::vector<uint8_t> can(nv), parity(nv), marked(nv);
    std::vector<uint32_t> region(nv), queue, nb;
    std::vector<std::array<size_t, 2>> votes;  // per region and colour: edges to vertices that cannot go
    auto neighbours = [&](uint32_t v) {
        nb.clear();
        for (uint32_t f : vertFaces[v]) {
            const std::vector<Corner>& c = faces[f].c;
            for (size_t j = 0; j < c.size(); ++j)
                if (c[j].pos == v) {
                    nb.push_back(c[(j + c.size() - 1) % c.size()].pos);
                    nb.push_back(c[(j + 1) % c.size()].pos);
                }
        }
        std::sort(nb.begin(), nb.end());
        nb.erase(std::unique(nb.begin(), nb.end()), nb.end());
    };
    {
        size_t used = 0, paired = 0;
        for (uint32_t v = 0; v < nv; ++v) {
            used += !vertFaces[v].empty();
            paired += !vertFaces[v].empty() && mir[v] != kNone;
        }
        res.mirrored = used ? (double)paired / (double)used : 0.0;
    }
    // a vertex and its mirror image get the same region and colour
    auto assign = [&](uint32_t u, uint32_t r, uint8_t par) {
        for (uint32_t x : {u, mir[u]}) {
            if (x == kNone || !can[x] || region[x] != kNone) continue;
            region[x] = r;
            parity[x] = par;
            queue.push_back(x);
        }
    };
    for (int pass = 0; pass < 2; ++pass) {
        for (uint32_t v = 0; v < nv; ++v) {
            can[v] = plan(v, false);
            region[v] = kNone;
        }
        votes.clear();
        for (uint32_t s = 0; s < nv; ++s) {
            if (!can[s] || region[s] != kNone) continue;
            const uint32_t r = (uint32_t)votes.size();
            votes.push_back({0, 0});
            queue.clear();
            assign(s, r, 0);
            for (size_t qi = 0; qi < queue.size(); ++qi) {
                const uint32_t v = queue[qi];
                neighbours(v);
                for (uint32_t u : nb) {
                    if (!can[u]) {
                        if (!vertFaces[u].empty()) ++votes[r][parity[v]];  // u stays if v goes
                    } else if (region[u] == kNone) {
                        assign(u, r, parity[v] ^ 1);
                    }
                }
            }
        }
        bool any = false;
        for (uint32_t v = 0; v < nv; ++v) {
            marked[v] = 0;
            if (!can[v]) continue;
            const std::array<size_t, 2>& vt = votes[region[v]];
            const uint8_t go = vt[0] > vt[1] ? 0 : 1;  // a tie: Blender's choice
            marked[v] = parity[v] == go;
            const uint32_t w = mir[v];  // both sides go or neither, and never a vertex next to its own image
            if (marked[v] && w != kNone && w != v) {
                neighbours(v);
                marked[v] = can[w] && parity[w] == parity[v] && !std::binary_search(nb.begin(), nb.end(), w);
            }
            any |= marked[v] != 0;
        }
        if (!any) {
            res.stop = "no vertex can be dissolved";
            break;
        }
        for (uint32_t v = 0; v < nv; ++v) {
            if (!marked[v]) continue;
            const uint32_t w = mir[v];
            if (w == kNone || w == v) {
                if (plan(v, true)) {
                    dissolve(v, nullptr);
                    ++res.dissolved;
                }
                continue;
            }
            if (w < v || !plan(v, true)) continue;  // a pair goes when its first vertex comes up
            dissolve(v, &undo);
            if (plan(w, true)) {
                dissolve(w, nullptr);
                res.dissolved += 2;
            } else {
                restore(undo);
                ++res.unpaired;
            }
        }
        ++res.passes;
    }

    // Blender's Tris to Quads (source/blender/bmesh/operators/bmo_join_triangles.cc) on what the
    // level leaves: triangle pairs that make a good quad are joined, best first by Blender's error
    // (bend + corners off a right angle + concavity). With its default limits the two halves may
    // bend by at most 40 degrees and no corner may be more than 40 degrees off a right angle; a
    // concave quad never joins, and neither does a pair across a seam, hard edge or material border.
    auto sub = [](const Vec3d& a, const Vec3d& b) { return Vec3d{a[0] - b[0], a[1] - b[1], a[2] - b[2]}; };
    auto dot = [](const Vec3d& a, const Vec3d& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
    auto cross = [](const Vec3d& a, const Vec3d& b) {
        return Vec3d{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    };
    auto angle = [&](const Vec3d& a, const Vec3d& b) {
        const double l = std::sqrt(dot(a, a) * dot(b, b));
        return l > 0 ? std::acos(std::clamp(dot(a, b) / l, -1.0, 1.0)) : 0.0;
    };
    auto normal = [&](uint32_t a, uint32_t b, uint32_t c) {
        return cross(sub(m.positions[b], m.positions[a]), sub(m.positions[c], m.positions[a]));
    };
    auto area = [&](uint32_t a, uint32_t b, uint32_t c) { const Vec3d n = normal(a, b, c); return 0.5 * std::sqrt(dot(n, n)); };
    constexpr double kPi = 3.14159265358979323846, kJoinLimit = 40.0 * kPi / 180.0;
    struct Join { double error; uint32_t t, g, at, gt; };  // triangles t and g, corners of the shared edge
    std::vector<Join> joins;
    for (uint32_t t = 0; t < faces.size() && res.passes; ++t) {
        const Face& T = faces[t];
        if (T.dead || T.c.size() != 3 || locked(T.material)) continue;
        for (uint32_t j = 0; j < 3; ++j) {
            const Corner &ca = T.c[j], &cb = T.c[(j + 1) % 3], &cc = T.c[(j + 2) % 3];
            uint32_t g = kNone, gt = 0, users = 0;  // the other face on edge a -> b, which runs b -> a
            for (uint32_t h : vertFaces[ca.pos]) {
                if (h == t) continue;
                const std::vector<Corner>& c = faces[h].c;
                for (uint32_t k = 0; k < c.size(); ++k) {
                    const uint32_t x = c[k].pos, y = c[(k + 1) % c.size()].pos;
                    if (x == cb.pos && y == ca.pos) { g = h; gt = k; ++users; }
                    else if (x == ca.pos && y == cb.pos) ++users;
                }
            }
            if (users != 1 || g < t) continue;  // each pair once
            const Face& G = faces[g];
            if (G.c.size() != 3 || G.material != T.material) continue;
            const Corner &gb = G.c[gt], &ga = G.c[(gt + 1) % 3], &gd = G.c[(gt + 2) % 3];
            if (ga.uv != ca.uv || gb.uv != cb.uv || ga.nrm != ca.nrm || gb.nrm != cb.nrm || gd.pos == cc.pos) continue;
            // the quad (a, d, b, c) in the triangles' winding
            const uint32_t q[4] = {ca.pos, gd.pos, cb.pos, cc.pos};
            if (angle(normal(ca.pos, cb.pos, cc.pos), normal(cb.pos, ca.pos, gd.pos)) > kJoinLimit) continue;
            if (dot(normal(q[0], q[1], q[2]), normal(q[0], q[2], q[3])) <= 0 ||
                dot(normal(q[1], q[2], q[3]), normal(q[1], q[3], q[0])) <= 0) continue;  // concave or folded
            Vec3d e[4];
            for (int i = 0; i < 4; ++i) e[i] = sub(m.positions[q[i]], m.positions[q[(i + 1) % 4]]);
            double corners = 0, worst = 0;
            for (int i = 0; i < 4; ++i) {
                const double off = std::abs(angle(e[i], e[(i + 1) % 4]) - kPi / 2);
                corners += off;
                worst = std::max(worst, off);
            }
            if (worst > kJoinLimit) continue;
            const double bend = (angle(normal(q[0], q[1], q[2]), normal(q[0], q[2], q[3])) +
                                 angle(normal(q[1], q[2], q[3]), normal(q[3], q[0], q[1]))) / (2 * kPi);
            const double a0 = area(q[0], q[1], q[2]) + area(q[0], q[2], q[3]);
            const double a1 = area(q[1], q[2], q[3]) + area(q[3], q[0], q[1]);
            const double concave = std::max(a0, a1) > 0 ? 1.0 - std::min(a0, a1) / std::max(a0, a1) : 1.0;
            joins.push_back({bend + corners / (2 * kPi) + concave, t, g, j, gt});
        }
    }
    std::sort(joins.begin(), joins.end(), [](const Join& x, const Join& y) { return x.error < y.error; });
    auto triangle = [&](uint32_t f) { return f != kNone && !faces[f].dead && faces[f].c.size() == 3; };
    auto corner_of = [&](uint32_t f, uint32_t v) {  // the corner of v in face f, or kNone
        for (uint32_t k = 0; k < faces[f].c.size(); ++k)
            if (faces[f].c[k].pos == v) return k;
        return kNone;
    };
    auto mirror_tri = [&](uint32_t f) {  // the live triangle on the mirror images of f's corners
        uint32_t mv[3];
        for (int k = 0; k < 3; ++k)
            if ((mv[k] = mir[faces[f].c[k].pos]) == kNone) return kNone;
        for (uint32_t h : vertFaces[mv[0]])
            if (triangle(h) && corner_of(h, mv[1]) != kNone && corner_of(h, mv[2]) != kNone) return h;
        return kNone;
    };
    auto join = [&](uint32_t t, uint32_t at, uint32_t g, uint32_t gt) {  // g runs t's edge at -> at + 1 backwards
        Face& T = faces[t];
        const Corner a = T.c[at], b = T.c[(at + 1) % 3], c = T.c[(at + 2) % 3], d = faces[g].c[(gt + 2) % 3];
        T.c = {a, d, b, c};
        faces[g].dead = true;
        for (const Corner& x : faces[g].c) {
            std::vector<uint32_t>& vf = vertFaces[x.pos];
            vf.erase(std::find(vf.begin(), vf.end(), g));
        }
        vertFaces[d.pos].push_back(t);
        ++res.joined;
    };
    for (const Join& J : joins) {
        if (!triangle(J.t) || !triangle(J.g)) continue;  // one of them already joined
        const uint32_t a = faces[J.t].c[J.at].pos, b = faces[J.t].c[(J.at + 1) % 3].pos;
        bool alone = true;  // no mirror here: nothing to keep symmetric
        for (const uint32_t f : {J.t, J.g})
            for (const Corner& c : faces[f].c) alone &= mir[c.pos] == kNone;
        const uint32_t mt = alone ? kNone : mirror_tri(J.t), mg = alone ? kNone : mirror_tri(J.g);
        if (alone || (mt == J.t && mg == J.g) || (mt == J.g && mg == J.t)) {  // no mirror, or its own mirror
            join(J.t, J.at, J.g, J.gt);
            continue;
        }
        // the mirror pair runs the shared edge the other way: mt from mir[b] to mir[a], mg back
        const uint32_t mat = mt == kNone ? kNone : corner_of(mt, mir[b]), mgt = mg == kNone ? kNone : corner_of(mg, mir[a]);
        if (mat == kNone || mgt == kNone || mt == J.t || mt == J.g || mg == J.t || mg == J.g ||
            faces[mt].c[(mat + 1) % 3].pos != mir[a] || faces[mg].c[(mgt + 1) % 3].pos != mir[b]) {
            ++res.unpaired;
            continue;
        }
        join(J.t, J.at, J.g, J.gt);
        join(mt, mat, mg, mgt);
    }

    InMesh out;
    out.name = m.name;
    out.positions = std::move(m.positions);
    out.normals = std::move(m.normals);
    out.uvs = std::move(m.uvs);
    for (const Face& F : faces) {
        if (F.dead) continue;
        out.faces.push_back({(uint32_t)out.cornerPos.size(), (uint32_t)F.c.size(), F.material});
        for (const Corner& c : F.c) {
            out.cornerPos.push_back(c.pos);
            out.cornerUv.push_back(c.uv);
            if (hasN) out.cornerNrm.push_back(c.nrm);
        }
    }
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
                const DissolveResult d = dissolve_level(m, opt.keep, opt.symmetry);
                mr.dissolved += d.dissolved;
                mr.passes += d.passes;
                mr.joined += d.joined;
                mr.unpaired += d.unpaired;
                mr.mirrorAxis = d.axis;
                mr.mirrored = d.mirrored;
                mr.stop = d.stop;
                if (d.passes) ++mr.approxLevels;
                if (!d.stop.empty()) break;
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
