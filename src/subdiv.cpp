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
// Approximate reverse subdivision for meshes that are not a subdivision result: polychord
// collapse. A chord is the strip of quads a run of edges crosses (the dual of an edge loop).
// Collapsing it merges its two rails and removes its quads, so the mesh stays a quad mesh and the
// edge flow is kept. Chords go in rounds, cheapest first and never two touching chords in one
// round, until the face target is reached. On a regular region that removes every other chord in
// each direction, i.e. exactly one reverse subdivision step; around poles and triangles chords
// simply end. Each merged vertex slides along its transversal edge to the point of least
// quadric error, and its UVs and normals are interpolated at the same parameter, per corner, so
// seams and hard edges stay seams and hard edges.

inline Vec3d v_sub(const Vec3d& a, const Vec3d& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
inline double v_dot(const Vec3d& a, const Vec3d& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline Vec3d v_lerp(const Vec3d& a, const Vec3d& b, double t) { return {a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])}; }

struct Quadric {
    double a[6] = {}, b[3] = {}, c = 0;  // xx xy xz yy yz zz
    void add_plane(const Vec3d& n, double d, double w) {
        a[0] += w * n[0] * n[0]; a[1] += w * n[0] * n[1]; a[2] += w * n[0] * n[2];
        a[3] += w * n[1] * n[1]; a[4] += w * n[1] * n[2]; a[5] += w * n[2] * n[2];
        b[0] += w * d * n[0]; b[1] += w * d * n[1]; b[2] += w * d * n[2];
        c += w * d * d;
    }
    void add(const Quadric& o) {
        for (int i = 0; i < 6; ++i) a[i] += o.a[i];
        for (int i = 0; i < 3; ++i) b[i] += o.b[i];
        c += o.c;
    }
    Vec3d mul(const Vec3d& p) const {
        return {a[0] * p[0] + a[1] * p[1] + a[2] * p[2], a[1] * p[0] + a[3] * p[1] + a[4] * p[2], a[2] * p[0] + a[4] * p[1] + a[5] * p[2]};
    }
    double eval(const Vec3d& p) const { return v_dot(p, mul(p)) + 2 * (b[0] * p[0] + b[1] * p[1] + b[2] * p[2]) + c; }
    // parameter along a -> b with the least error
    double best_t(const Vec3d& pa, const Vec3d& pb) const {
        const Vec3d d = v_sub(pb, pa);
        const double alpha = v_dot(d, mul(d));
        const double beta = 2 * (v_dot(d, mul(pa)) + b[0] * d[0] + b[1] * d[1] + b[2] * d[2]);
        if (alpha <= 1e-300) return 0.5;
        return std::clamp(-beta / (2 * alpha), 0.0, 1.0);
    }
};

struct CoarsenResult {
    size_t rounds = 0, chords = 0;
    std::string stop;  // why the target was not reached (empty = reached)
};

CoarsenResult coarsen_mesh(InMesh& m, size_t targetFaces, const std::vector<uint8_t>& lockedMaterial) {
    constexpr double kSing = 4.0;   // price of the two singularity pairs a partial chord leaves, spread over its quads
    constexpr size_t kMinPartial = 6;  // shorter partial chords are not worth their singularities
    CoarsenResult res;
    const bool hasN = !m.cornerNrm.empty();

    // Original local edge length per vertex: costs are relative to it, so dense and sparse regions
    // thin out at the same rate and a chord next to a removed one waits for the next level.
    std::vector<double> scale(m.positions.size(), 0.0);
    {
        const FaceSet fs = faces_of(m);
        const Topo t = topo_for(fs, m.cornerPos, m.positions.size());
        std::vector<uint32_t> cnt(t.nv, 0);
        for (uint32_t e = 0; e < t.ne(); ++e) {
            const Vec3d d = v_sub(m.positions[t.edgeA[e]], m.positions[t.edgeB[e]]);
            const double len = std::sqrt(v_dot(d, d));
            scale[t.edgeA[e]] += len; scale[t.edgeB[e]] += len;
            ++cnt[t.edgeA[e]]; ++cnt[t.edgeB[e]];
        }
        for (uint32_t v = 0; v < t.nv; ++v) scale[v] = cnt[v] && scale[v] > 0 ? scale[v] / cnt[v] : 1.0;
    }

    // face normal (Newell); vertices whose repStamp equals `id` are taken from repPos instead
    std::vector<uint32_t> repStamp;
    std::vector<Vec3d> repPos;
    auto newell = [&](const Topo& t, uint32_t f, const std::vector<Vec3d>& pos, uint32_t id) {
        Vec3d n{0, 0, 0};
        auto at = [&](uint32_t c) -> const Vec3d& {
            const uint32_t v = t.idx[c];
            return id != kNone && repStamp[v] == id ? repPos[v] : pos[v];
        };
        for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c) {
            const Vec3d& p = at(c);
            const Vec3d& q = at(t.next(c));
            n[0] += (p[1] - q[1]) * (p[2] + q[2]);
            n[1] += (p[2] - q[2]) * (p[0] + q[0]);
            n[2] += (p[0] - q[0]) * (p[1] + q[1]);
        }
        return n;
    };

    struct Chord {
        std::vector<uint32_t> quads;  // in walk order
        std::vector<uint8_t> entry;   // per quad: corner index of the transversal edge shared with the previous quad
        bool closed = false;
    };
    // One merge of two vertices: the whole transversal edge of a full chord, or the diagonal of a
    // quad in a partial one. `corner` is the quad corner the pair starts at (its UV / normal and
    // those of the corner `span` further on are interpolated at `t`).
    struct Pair { uint32_t A, B, quad, corner; uint8_t span; double t; Vec3d p; };
    // A collapsible run of quads: a whole chord (edge merges) or a segment of one (diagonal merges).
    struct Run { std::vector<uint32_t> quads; std::vector<Pair> pairs; bool whole = false, nextLevel = false; double cost = 0; };

    // Level rule: a collapse creates fresh rail edges; a chord that has a fresh edge as one of its
    // rails is a parallel neighbour of a removed chord and waits for the next level. Crossing
    // chords (the fresh edge is transversal to them) are fine. So each level removes at most
    // every other chord per direction, like a real reverse subdivision step. Keys are vertex pairs
    // in the current numbering, remapped after every round.
    std::vector<uint64_t> fresh;
    auto edge_key = [](uint32_t a, uint32_t b) { return a < b ? (uint64_t)a << 32 | b : (uint64_t)b << 32 | a; };

    while (m.faces.size() > targetFaces) {
        const FaceSet fs = faces_of(m);
        const Topo t = topo_for(fs, m.cornerPos, m.positions.size());
        const uint32_t nf = t.nf(), nv = t.nv;

        std::vector<uint8_t> boundary(nv, 0), locked(nv, 0);
        for (uint32_t e = 0; e < t.ne(); ++e)
            if (t.edgeFaces(e) != 2) boundary[t.edgeA[e]] = boundary[t.edgeB[e]] = 1;
        for (uint32_t f = 0; f < nf; ++f)
            if (m.faces[f].material < lockedMaterial.size() && lockedMaterial[m.faces[f].material])
                for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c) locked[t.idx[c]] = 1;

        std::vector<Quadric> Q(nv);
        std::vector<Vec3d> faceN(nf);
        for (uint32_t f = 0; f < nf; ++f) {
            const Vec3d n = newell(t, f, m.positions, kNone);
            faceN[f] = n;
            const double len = std::sqrt(v_dot(n, n));
            if (len <= 0) continue;
            const Vec3d nh = {n[0] / len, n[1] / len, n[2] / len};
            Vec3d ctr{0, 0, 0};
            for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c)
                for (int k = 0; k < 3; ++k) ctr[k] += m.positions[t.idx[c]][k] / t.deg(f);
            for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c) Q[t.idx[c]].add_plane(nh, -v_dot(nh, ctr), 0.5 * len);
        }

        // ---- chords -------------------------------------------------------------------------
        std::vector<uint32_t> chordOf(nf * 2, kNone);  // per quad and parity (0: edges 0-1 / 2-3 are crossed)
        std::vector<Chord> chords;
        // cross edge k of quad f: the quad beyond and the edge to leave it through
        auto step = [&](uint32_t f, int k, uint32_t& g, int& kOut) {
            const uint32_t e = t.cornerEdge[t.faceFirst[f] + k];
            if (t.edgeFaces(e) != 2) return false;
            g = t.faceOf[t.edgeCorners[t.edgeFirst[e]]];
            if (g == f) g = t.faceOf[t.edgeCorners[t.edgeFirst[e] + 1]];
            if (g == f || t.deg(g) != 4) return false;
            for (int j = 0; j < 4; ++j)
                if (t.cornerEdge[t.faceFirst[g] + j] == e) { kOut = (j + 2) & 3; return true; }
            return false;
        };
        for (uint32_t f = 0; f < nf; ++f) {
            if (t.deg(f) != 4) continue;
            for (int p = 0; p < 2; ++p) {
                if (chordOf[f * 2 + p] != kNone) continue;
                const uint32_t id = (uint32_t)chords.size();
                Chord ch;
                chordOf[f * 2 + p] = id;
                ch.quads.push_back(f);
                std::vector<uint32_t> back;
                for (int dir = 0; dir < 2 && !ch.closed; ++dir) {
                    uint32_t cur = f;
                    int k = p + 2 * dir;
                    for (;;) {
                        uint32_t g;
                        int kOut;
                        if (!step(cur, k, g, kOut)) break;
                        const uint32_t slot = g * 2 + (kOut & 1);
                        if (chordOf[slot] == id) { if (g == f && (kOut & 1) == p) ch.closed = true; break; }
                        chordOf[slot] = id;
                        (dir ? back : ch.quads).push_back(g);
                        cur = g;
                        k = kOut;
                    }
                }
                if (!back.empty()) {
                    std::reverse(back.begin(), back.end());
                    back.insert(back.end(), ch.quads.begin(), ch.quads.end());
                    ch.quads = std::move(back);
                }
                // entry edge per quad: the transversal edge shared with the previous quad
                const size_t n = ch.quads.size();
                ch.entry.resize(n);
                for (size_t i = 0; i < n; ++i) {
                    const uint32_t q = ch.quads[i], base = t.faceFirst[q];
                    const uint8_t par = (uint8_t)(chordOf[q * 2] == id ? 0 : 1);
                    uint8_t k = par;
                    if (n > 1) {
                        const uint32_t prev = ch.quads[i == 0 ? (ch.closed ? n - 1 : 1) : i - 1], pb = t.faceFirst[prev];
                        bool shared = false;
                        for (int j = 0; j < 4 && !shared; ++j) shared = t.cornerEdge[pb + j] == t.cornerEdge[base + par];
                        k = shared ? par : par + 2;
                        if (i == 0 && !ch.closed) k = (k + 2) & 3;  // first quad: the edge the chain does not use
                    }
                    ch.entry[i] = k;
                }
                chords.push_back(std::move(ch));
            }
        }

        // ---- runs: whole chords, or the longest usable segments of them ----------------------
        std::vector<uint32_t> nstamp(nv, kNone), pstamp(nv, kNone), fcount(nf, 0), fstamp(nf, kNone);
        std::vector<uint32_t> grp(nv), gsize(nv), estamp(t.ne(), kNone), gstamp(nv, kNone), gcnt(nv, 0);
        std::vector<Vec3d> gacc(nv);
        auto groot = [&](uint32_t v) { while (grp[v] != v) v = grp[v] = grp[grp[v]]; return v; };
        std::vector<Run> runs;
        uint32_t nbStamp = 0;
        repStamp.assign(nv, kNone);
        repPos.resize(nv);
        std::vector<uint8_t> blocked, pairBad;
        std::vector<uint32_t> segQuads;

        // Fill in the pairs of a run, check them, and return false with the offending quads flagged.
        auto make_pairs = [&](const Chord& ch, Run& run, const std::vector<uint32_t>& at, uint32_t id) {
            run.pairs.clear();
            for (uint32_t i : at) {
                const uint32_t q = ch.quads[i], base = t.faceFirst[q];
                const uint8_t k = ch.entry[i];
                Pair pr;
                pr.quad = q;
                if (run.whole) {  // edge merges: each transversal edge once
                    for (int side = 0; side < 2; ++side) {
                        const uint32_t c0 = base + ((k + 2 * side) & 3), e = t.cornerEdge[c0];
                        if (estamp[e] == id) continue;
                        estamp[e] = id;
                        pr.corner = c0; pr.span = 1;
                        pr.A = t.idx[c0]; pr.B = t.idx[t.next(c0)];
                        run.pairs.push_back(pr);
                    }
                } else {  // diagonal from the first corner of the entry edge
                    pr.corner = base + k; pr.span = 2;
                    pr.A = t.idx[base + k]; pr.B = t.idx[base + ((k + 2) & 3)];
                    run.pairs.push_back(pr);
                }
            }
            // Pairs merge into groups. A group is two vertices, or the four corners of a quad the
            // chord crosses twice (that quad shrinks to a point, as in a real reverse subdivision
            // step). Anything else (rails meeting at a pole, a vertex in two diagonals) is not
            // collapsible.
            for (const Pair& pr : run.pairs)
                for (uint32_t v : {pr.A, pr.B})
                    if (pstamp[v] != id) { pstamp[v] = id; grp[v] = v; gsize[v] = 1; }
            for (const Pair& pr : run.pairs) {
                if (pr.A == pr.B) return false;
                const uint32_t ra = groot(pr.A), rb = groot(pr.B);
                if (ra != rb) { grp[rb] = ra; gsize[ra] += gsize[rb]; }
            }
            for (uint32_t q : run.quads) fcount[q] = 0;
            for (uint32_t q : run.quads) ++fcount[q];
            for (uint32_t q : run.quads) {
                if (fcount[q] != 2) continue;
                const uint32_t r = groot(t.idx[t.faceFirst[q]]);
                bool same = gsize[r] == 4;
                for (uint32_t c = t.faceFirst[q] + 1; c < t.faceFirst[q] + 4; ++c) same &= groot(t.idx[c]) == r;
                if (!same) return false;
                gsize[r] = 0;  // accounted for
            }
            for (const Pair& pr : run.pairs) {
                const uint32_t s = gsize[groot(pr.A)];
                if (s != 2 && s != 0) return false;
            }
            return true;
        };

        auto check_pairs = [&](const Chord& ch, Run& run, uint32_t id) {
            pairBad.assign(run.pairs.size(), 0);
            double cost = 0;
            for (size_t pi = 0; pi < run.pairs.size(); ++pi) {
                Pair& pr = run.pairs[pi];
                const uint32_t A = pr.A, B = pr.B;
                bool bad = locked[A] || locked[B];
                // pinch: both on a border but the pair is not a border edge
                if (boundary[A] && boundary[B]) {
                    bool borderEdge = false;
                    if (run.whole) borderEdge = t.edgeFaces(t.cornerEdge[pr.corner]) == 1;
                    bad |= !borderEdge;
                }
                if (bad) { pairBad[pi] = 1; continue; }
                // a shared neighbour outside the collapsed quads would fold onto the merged vertex
                const uint32_t stamp = ++nbStamp;
                for (uint32_t kk = t.vertFirst[A]; kk < t.vertFirst[A + 1]; ++kk) {
                    const uint32_t c = t.vertCorners[kk];
                    nstamp[t.idx[t.next(c)]] = stamp;
                    nstamp[t.idx[t.prev(c)]] = stamp;
                }
                uint32_t allowed[2] = {kNone, kNone};
                if (!run.whole) {  // the quad's other two corners are legitimately shared
                    const uint32_t base = t.faceFirst[pr.quad];
                    allowed[0] = t.idx[base + ((pr.corner - base + 1) & 3)];
                    allowed[1] = t.idx[base + ((pr.corner - base + 3) & 3)];
                }
                for (uint32_t kk = t.vertFirst[B]; kk < t.vertFirst[B + 1] && !bad; ++kk) {
                    const uint32_t c = t.vertCorners[kk];
                    for (uint32_t x : {t.idx[t.next(c)], t.idx[t.prev(c)]})
                        if (x != A && x != allowed[0] && x != allowed[1] && nstamp[x] == stamp) bad = true;
                }
                if (bad) { pairBad[pi] = 1; continue; }
                // UV / normal pairs across a merged edge must be either shared or fully separate
                if (!bad && run.whole) {
                    const uint32_t e = t.cornerEdge[pr.corner];
                    if (t.edgeFaces(e) == 2) {
                        uint32_t ua[2], ub[2], na[2], nb[2];
                        for (int s = 0; s < 2; ++s) {
                            const uint32_t c = t.edgeCorners[t.edgeFirst[e] + s], c1 = t.next(c);
                            const bool fwd = t.idx[c] == A;
                            ua[s] = m.cornerUv[fs.srcCorner[fwd ? c : c1]];
                            ub[s] = m.cornerUv[fs.srcCorner[fwd ? c1 : c]];
                            na[s] = hasN ? m.cornerNrm[fs.srcCorner[fwd ? c : c1]] : 0;
                            nb[s] = hasN ? m.cornerNrm[fs.srcCorner[fwd ? c1 : c]] : 0;
                        }
                        if ((ua[0] == ua[1]) != (ub[0] == ub[1]) || (na[0] == na[1]) != (nb[0] == nb[1])) bad = true;
                    }
                }
                if (bad) { pairBad[pi] = 1; continue; }
                // where the merged vertex goes
                const Vec3d& pa = m.positions[A];
                const Vec3d& pb = m.positions[B];
                Quadric q = Q[A];
                q.add(Q[B]);
                pr.t = boundary[A] && !boundary[B] ? 0.0 : boundary[B] && !boundary[A] ? 1.0 : q.best_t(pa, pb);
                pr.p = v_lerp(pa, pb, pr.t);
                const double err = std::max(0.0, q.eval(pr.p));
                // cost: the shape error relative to the local original edge length. No size term:
                // like a real reverse subdivision step, the level rule alternates chords in every
                // direction whatever the quads' aspect ratio.
                const double s = 0.5 * (scale[A] + scale[B]);
                cost += err / (s * s);
                repStamp[A] = repStamp[B] = id;
                repPos[A] = repPos[B] = pr.p;
            }
            bool anyBad = false;
            for (uint8_t b : pairBad) anyBad |= b != 0;
            if (anyBad) return false;
            // a group of four (a quad the chord crosses twice) goes to the mean of its pair points
            for (const Pair& pr : run.pairs) {
                const uint32_t r = groot(pr.A);
                if (gstamp[r] != id) { gstamp[r] = id; gacc[r] = {0, 0, 0}; gcnt[r] = 0; }
                for (int d = 0; d < 3; ++d) gacc[r][d] += pr.p[d];
                ++gcnt[r];
            }
            for (const Pair& pr : run.pairs) {
                const uint32_t r = groot(pr.A);
                for (uint32_t v : {pr.A, pr.B})
                    for (int d = 0; d < 3; ++d) repPos[v][d] = gacc[r][d] / gcnt[r];
            }
            // no face next to the run may flip or vanish
            for (uint32_t q : run.quads) fstamp[q] = id;
            for (size_t pi = 0; pi < run.pairs.size(); ++pi) {
                for (uint32_t v : {run.pairs[pi].A, run.pairs[pi].B}) {
                    for (uint32_t kk = t.vertFirst[v]; kk < t.vertFirst[v + 1]; ++kk) {
                        const uint32_t g = t.faceOf[t.vertCorners[kk]];
                        if (fstamp[g] == id) continue;
                        const Vec3d n = newell(t, g, m.positions, id);
                        if (v_dot(n, faceN[g]) <= 1e-6 * v_dot(faceN[g], faceN[g])) pairBad[pi] = 1;
                    }
                }
            }
            for (uint8_t b : pairBad) anyBad |= b != 0;
            if (anyBad) return false;
            run.cost = cost / run.pairs.size() + (run.whole ? 0.0 : kSing / run.quads.size());
            return true;
        };

        uint32_t runId = 0;
        for (uint32_t ci = 0; ci < chords.size(); ++ci) {
            const Chord& ch = chords[ci];
            const size_t n = ch.quads.size();
            // hard blocks per quad: a locked vertex or a non-manifold transversal edge
            blocked.assign(n, 0);
            for (size_t i = 0; i < n; ++i) {
                const uint32_t q = ch.quads[i], base = t.faceFirst[q];
                bool b = false;
                for (uint32_t c = base; c < base + 4; ++c) b |= locked[t.idx[c]] != 0;
                for (int side = 0; side < 2; ++side) b |= t.edgeFaces(t.cornerEdge[base + ((ch.entry[i] + 2 * side) & 3)]) > 2;
                blocked[i] = b;
            }
            bool anyBlocked = false;
            for (uint8_t b : blocked) anyBlocked |= b != 0;

            // The whole chord with edge merges when nothing blocks it and it is closed or ends on
            // the border on both sides; otherwise segments with diagonal merges, split further
            // wherever a pair fails its checks.
            std::vector<std::vector<uint32_t>> work;  // quad indices into the chord
            if (!anyBlocked) {
                std::vector<uint32_t> all(n);
                for (uint32_t i = 0; i < n; ++i) all[i] = i;
                bool ends = ch.closed;
                if (!ch.closed) {
                    const uint32_t first = t.cornerEdge[t.faceFirst[ch.quads[0]] + ch.entry[0]];
                    const uint32_t last = t.cornerEdge[t.faceFirst[ch.quads[n - 1]] + ((ch.entry[n - 1] + 2) & 3)];
                    ends = t.edgeFaces(first) == 1 && t.edgeFaces(last) == 1;
                }
                bool done = false;
                if (ends) {
                    Run run;
                    run.whole = true;
                    run.quads = ch.quads;
                    const uint32_t id = ++runId;
                    if (make_pairs(ch, run, all, id) && check_pairs(ch, run, id)) { runs.push_back(std::move(run)); done = true; }
                }
                if (!done) work.push_back(all);
            } else {
                size_t start = 0;  // maximal unblocked segments (cyclic for closed chords)
                if (ch.closed) { while (start < n && !blocked[start]) ++start; }
                segQuads.clear();
                for (size_t s = 0; s < n; ++s) {
                    const size_t i = (start + s) % n;
                    if (blocked[i]) { if (!segQuads.empty()) work.push_back(segQuads); segQuads.clear(); }
                    else segQuads.push_back((uint32_t)i);
                }
                if (!segQuads.empty()) work.push_back(segQuads);
            }
            for (size_t w = 0; w < work.size(); ++w) {
                const std::vector<uint32_t> at = work[w];
                if (at.size() < kMinPartial) continue;
                Run run;
                for (uint32_t i : at) run.quads.push_back(ch.quads[i]);
                const uint32_t id = ++runId;
                if (!make_pairs(ch, run, at, id)) {  // a vertex in two pairs: try the halves
                    if (at.size() > 1) {
                        const size_t half = at.size() / 2;
                        work.push_back(std::vector<uint32_t>(at.begin(), at.begin() + half));
                        work.push_back(std::vector<uint32_t>(at.begin() + half, at.end()));
                    }
                    continue;
                }
                if (check_pairs(ch, run, id)) { runs.push_back(std::move(run)); continue; }
                segQuads.clear();  // split around the bad pairs (one pair per quad)
                for (size_t pi = 0; pi < at.size(); ++pi) {
                    if (pairBad[pi]) { if (!segQuads.empty()) work.push_back(segQuads); segQuads.clear(); }
                    else segQuads.push_back(at[pi]);
                }
                if (!segQuads.empty()) work.push_back(segQuads);
            }
        }

        // ---- pick the cheapest runs that do not touch ---------------------------------------
        std::sort(fresh.begin(), fresh.end());
        size_t waiting = 0;
        for (Run& run : runs) {
            for (uint32_t q : run.quads) {
                const uint32_t base = t.faceFirst[q];
                uint8_t k = 0;  // the run's transversal edges of this quad start at corner k and k + 2
                for (const Pair& pr : run.pairs)
                    if (pr.quad == q) { k = (uint8_t)((pr.corner - base) & 3); break; }
                for (int side = 0; side < 2; ++side) {
                    const uint32_t x = t.idx[base + ((k + 1 + 2 * side) & 3)], y = t.idx[base + ((k + 2 + 2 * side) & 3)];
                    if (std::binary_search(fresh.begin(), fresh.end(), edge_key(x, y))) run.nextLevel = true;
                }
            }
            waiting += run.nextLevel;
        }
        std::vector<uint32_t> order;
        for (uint32_t i = 0; i < runs.size(); ++i)
            if (!runs[i].nextLevel) order.push_back(i);
        std::sort(order.begin(), order.end(), [&](uint32_t x, uint32_t y) { return runs[x].cost < runs[y].cost; });
        std::vector<uint8_t> used(nv, 0), dead(nf, 0);
        std::vector<uint32_t> picked;
        size_t removed = 0;
        const size_t needed = m.faces.size() - targetFaces;
        // whole chords first; partial ones only in a round where no whole chord is left, so their
        // singularities cannot break whole chords that were still to come in this level
        size_t wholePicked = 0;
        for (int pass = 0; pass < 2; ++pass)
        for (uint32_t id : order) {
            if (removed >= needed || (pass == 1 && wholePicked)) break;
            const Run& run = runs[id];
            if (run.whole != (pass == 0)) continue;
            bool free = true;
            for (uint32_t f : run.quads)
                for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1] && free; ++c) free = !used[t.idx[c]];
            if (!free) continue;
            for (uint32_t f : run.quads) {
                dead[f] = 1;
                for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c) used[t.idx[c]] = 1;
            }
            picked.push_back(id);
            removed += run.quads.size();
            wholePicked += run.whole;
        }
        if (std::getenv("UDIM_COARSEN_DEBUG")) {  // per-round trace for tuning
            size_t whole = 0, part = 0, hist[6] = {};
            for (uint32_t id : picked) { const Run& r = runs[id]; if (r.whole) ++whole; else { ++part; size_t L = r.quads.size(); ++hist[L >= 32 ? 5 : L >= 16 ? 4 : L >= 8 ? 3 : L >= 4 ? 2 : L >= 2 ? 1 : 0]; } }
            size_t rw = 0; for (const Run& r : runs) rw += r.whole;
            std::fprintf(stderr, "round %zu: faces %zu needed %zu removed %zu | runs %zu (whole %zu) picked whole %zu partial %zu len<2:%zu <4:%zu <8:%zu <16:%zu <32:%zu >=32:%zu | cost first %.3g last %.3g\n",
                         res.rounds + 1, m.faces.size(), needed, removed, runs.size(), rw, whole, part, hist[0], hist[1], hist[2], hist[3], hist[4], hist[5],
                         picked.empty() ? 0.0 : runs[picked.front()].cost, picked.empty() ? 0.0 : runs[picked.back()].cost);
        }
        if (picked.empty()) {
            if (!res.rounds) res.stop = runs.empty() ? "no collapsible chords" : "chords touch locked ones";
            break;  // the level is done: every remaining chord waits for the next level
        }

        // ---- collapse: merge the pairs per channel ----------------------------------------------
        std::vector<uint32_t> posRoot(nv), uvRoot(m.uvs.size()), nrmRoot(m.normals.size());
        for (uint32_t i = 0; i < nv; ++i) posRoot[i] = i;
        for (uint32_t i = 0; i < uvRoot.size(); ++i) uvRoot[i] = i;
        for (uint32_t i = 0; i < nrmRoot.size(); ++i) nrmRoot[i] = i;
        auto find = [](std::vector<uint32_t>& root, uint32_t x) {
            while (root[x] != x) { root[x] = root[root[x]]; x = root[x]; }
            return x;
        };
        std::vector<Vec2d> uvNew = m.uvs;
        std::vector<Vec3d> nrmNew = m.normals;
        auto merge_attrs = [&](uint32_t c0, uint32_t c1, double tt) {  // corners of one face, c0 -> c1 at tt
            const uint32_t u0 = m.cornerUv[fs.srcCorner[c0]], u1 = m.cornerUv[fs.srcCorner[c1]];
            const uint32_t r0 = find(uvRoot, u0), r1 = find(uvRoot, u1);
            if (r0 != r1) {
                uvNew[r0] = {m.uvs[u0].x + tt * (m.uvs[u1].x - m.uvs[u0].x), m.uvs[u0].y + tt * (m.uvs[u1].y - m.uvs[u0].y)};
                uvRoot[r1] = r0;
            }
            if (hasN) {
                const uint32_t n0 = m.cornerNrm[fs.srcCorner[c0]], n1 = m.cornerNrm[fs.srcCorner[c1]];
                const uint32_t s0 = find(nrmRoot, n0), s1 = find(nrmRoot, n1);
                if (s0 != s1) {
                    Vec3d n = v_lerp(m.normals[n0], m.normals[n1], tt);
                    const double len = std::sqrt(v_dot(n, n));
                    if (len > 1e-12) for (double& x : n) x /= len;
                    nrmNew[s0] = n;
                    nrmRoot[s1] = s0;
                }
            }
        };
        for (uint32_t id : picked)
            for (const Pair& pr : runs[id].pairs) {
                const uint32_t ra = find(posRoot, pr.A), rb = find(posRoot, pr.B);
                if (ra != rb) { posRoot[rb] = ra; scale[ra] = 0.5 * (scale[ra] + scale[rb]); }
            }
        {   // merged position: mean of the pair points of the group (one pair, or four at a crossing)
            std::vector<Vec3d> acc(nv, Vec3d{0, 0, 0});
            std::vector<uint32_t> cnt(nv, 0);
            for (uint32_t id : picked)
                for (const Pair& pr : runs[id].pairs) {
                    const uint32_t r = find(posRoot, pr.A);
                    for (int d = 0; d < 3; ++d) acc[r][d] += pr.p[d];
                    ++cnt[r];
                }
            for (uint32_t v = 0; v < nv; ++v)
                if (cnt[v]) for (int d = 0; d < 3; ++d) m.positions[v][d] = acc[v][d] / cnt[v];
        }
        for (uint32_t id : picked) {
            const Run& run = runs[id];
            for (const Pair& pr : run.pairs) {
                if (run.whole) {  // both faces on the edge carry their own UV / normal pair
                    const uint32_t e = t.cornerEdge[pr.corner];
                    for (uint32_t kk = t.edgeFirst[e]; kk < t.edgeFirst[e + 1]; ++kk) {
                        const uint32_t c = t.edgeCorners[kk], c1 = t.next(c);
                        merge_attrs(c, c1, t.idx[c] == pr.A ? pr.t : 1.0 - pr.t);
                    }
                } else {
                    const uint32_t base = t.faceFirst[pr.quad];
                    merge_attrs(pr.corner, base + ((pr.corner - base + 2) & 3), pr.t);
                }
            }
        }
        res.chords += picked.size();
        ++res.rounds;

        // ---- rebuild the mesh --------------------------------------------------------------------
        InMesh out;
        out.name = m.name;
        std::vector<uint32_t> posMap(nv, kNone), uvMap(m.uvs.size(), kNone), nrmMap(m.normals.size(), kNone);
        std::vector<double> scaleOut;
        for (uint32_t f = 0; f < nf; ++f) {
            if (dead[f]) continue;
            out.faces.push_back({(uint32_t)out.cornerPos.size(), t.deg(f), m.faces[f].material});
            for (uint32_t c = t.faceFirst[f]; c < t.faceFirst[f + 1]; ++c) {
                const uint32_t oc = fs.srcCorner[c];
                const uint32_t pv = find(posRoot, m.cornerPos[oc]);
                if (posMap[pv] == kNone) { posMap[pv] = (uint32_t)out.positions.size(); out.positions.push_back(m.positions[pv]); scaleOut.push_back(scale[pv]); }
                out.cornerPos.push_back(posMap[pv]);
                const uint32_t uv = find(uvRoot, m.cornerUv[oc]);
                if (uvMap[uv] == kNone) { uvMap[uv] = (uint32_t)out.uvs.size(); out.uvs.push_back(uvNew[uv]); }
                out.cornerUv.push_back(uvMap[uv]);
                if (hasN) {
                    const uint32_t nn = find(nrmRoot, m.cornerNrm[oc]);
                    if (nrmMap[nn] == kNone) { nrmMap[nn] = (uint32_t)out.normals.size(); out.normals.push_back(nrmNew[nn]); }
                    out.cornerNrm.push_back(nrmMap[nn]);
                }
            }
        }
        // fresh rail edges for the level rule, in the new numbering
        std::vector<uint64_t> freshOut;
        auto add_fresh = [&](uint32_t a, uint32_t b) {
            const uint32_t x = posMap[find(posRoot, a)], y = posMap[find(posRoot, b)];
            if (x != kNone && y != kNone && x != y) freshOut.push_back(edge_key(x, y));
        };
        for (uint64_t key : fresh) add_fresh((uint32_t)(key >> 32), (uint32_t)key);
        for (uint32_t id : picked)
            for (const Pair& pr : runs[id].pairs) {
                const uint32_t base = t.faceFirst[pr.quad], k = (pr.corner - base) & 3;
                for (int side = 0; side < 2; ++side)
                    add_fresh(t.idx[base + ((k + 1 + 2 * side) & 3)], t.idx[base + ((k + 2 + 2 * side) & 3)]);
            }
        std::sort(freshOut.begin(), freshOut.end());
        freshOut.erase(std::unique(freshOut.begin(), freshOut.end()), freshOut.end());
        fresh = std::move(freshOut);
        m = std::move(out);
        scale = std::move(scaleOut);
        if (!wholePicked) break;  // partial chords only get the one last round of a level
    }
    triangulate(m);
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
                const CoarsenResult c = coarsen_mesh(m, (m.faces.size() + 3) / 4, opt.keep);
                mr.chords += c.chords;
                mr.rounds += c.rounds;
                mr.stop = c.stop;
                ++mr.approxLevels;
                if (!c.stop.empty()) break;
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
