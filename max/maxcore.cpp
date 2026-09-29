// udimpacker for 3ds Max: the work, in a DLL the plugin loads from a copy and can reload
// (see maxcore_api.h). Everything here runs on Max's main thread, called from the plugin.
#include <max.h>
#include <iskin.h>
#include <modstack.h>
#include <polyobj.h>
#include <mnmesh.h>
#include <MNNormalSpec.h>
#include <stdmat.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "atlas.h"
#include "decimate.h"
#include "maxcore_api.h"
#include "subdiv.h"
#include "udimpacker.h"

namespace {

const char kBuild[] = __DATE__ " " __TIME__;
constexpr uint32_t kNone = UINT32_MAX;

std::string utf8(const MSTR& s) { return std::string(s.ToUTF8().data()); }

std::string json_string(const std::string& s) {
    std::string q = "\"";
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') { q += '\\'; q += (char)c; }
        else if (c < 0x20) { char buf[8]; std::snprintf(buf, sizeof buf, "\\u%04x", c); q += buf; }
        else q += (char)c;
    }
    return q + "\"";
}

std::string class_name(const Animatable* a) {
    MSTR s;
    a->GetClassName(s, false);
    return utf8(s);
}

// The modifier stack of a node: its modifiers top first, and the first Skin found.
struct Stack {
    std::vector<std::string> modifiers;
    IDerivedObject* skinOwner = nullptr;
    int skinIndex = -1;
    ISkin* skin = nullptr;
};

Stack stack_of(INode* node) {
    Stack s;
    for (Object* obj = node->GetObjectRef(); obj && obj->SuperClassID() == GEN_DERIVOB_CLASS_ID;) {
        IDerivedObject* d = static_cast<IDerivedObject*>(obj);
        for (int i = 0; i < d->NumModifiers(); ++i) {
            Modifier* m = d->GetModifier(i);
            s.modifiers.push_back(utf8(m->GetName(false)));
            if (!s.skin)
                if (auto* sk = static_cast<ISkin*>(m->GetInterface(I_SKIN))) {
                    s.skinOwner = d;
                    s.skinIndex = i;
                    s.skin = sk;
                }
        }
        obj = d->GetObjRef();
    }
    return s;
}

// Every bitmap file under a texmap (through Normal Bump, Color Correction and the like).
void bitmap_files(Texmap* tm, std::vector<std::string>& files, int depth = 0) {
    if (!tm || depth > 8) return;
    if (tm->ClassID() == Class_ID(BMTEX_CLASS_ID, 0)) {
        files.push_back(utf8(MSTR(static_cast<BitmapTex*>(tm)->GetMapName())));
        return;
    }
    for (int i = 0; i < tm->NumSubTexmaps(); ++i) bitmap_files(tm->GetSubTexmap(i), files, depth + 1);
}

std::string material_json(Mtl* m) {
    std::string j = "{\"name\": " + json_string(utf8(m->GetName())) + ", \"class\": " + json_string(class_name(m)) + ", \"maps\": [";
    bool first = true;
    for (int i = 0; i < m->NumSubTexmaps(); ++i) {
        Texmap* tm = m->GetSubTexmap(i);
        if (!tm) continue;
        std::vector<std::string> files;
        bitmap_files(tm, files);
        j += std::string(first ? "" : ", ") + "{\"slot\": " + json_string(utf8(m->GetSubTexmapSlotName(i, false))) +
             ", \"texmap\": " + json_string(class_name(tm)) + ", \"files\": [";
        for (size_t k = 0; k < files.size(); ++k) j += (k ? ", " : "") + json_string(files[k]);
        j += "]}";
        first = false;
    }
    return j + "]}";
}

bool inspect(INode* node, UdimSink out) {
    const TimeValue t = GetCOREInterface()->GetTime();
    std::string j = "{\"node\": " + json_string(utf8(MSTR(node->GetName())));
    const Stack st = stack_of(node);
    j += ", \"modifiers\": [";
    for (size_t i = 0; i < st.modifiers.size(); ++i) j += (i ? ", " : "") + json_string(st.modifiers[i]);
    j += "]";

    // The mesh Skin deforms (the bind pose), or the final mesh without Skin. Eval(t, i) evaluates
    // the stack up to and including modifier i (Skin itself calls Eval(t, own index) to run up to
    // itself, bonesdef.cpp), so the input of Skin is Eval(t, index + 1), or the object below the
    // derived object when Skin is the lowest modifier. Checked below against the base object.
    Object* base = node->GetObjectRef()->FindBaseObject();
    ObjectState os = !st.skin ? node->EvalWorldState(t)
                     : st.skinIndex + 1 < st.skinOwner->NumModifiers() ? st.skinOwner->Eval(t, st.skinIndex + 1)
                                                                        : st.skinOwner->GetObjRef()->Eval(t);
    if (!os.obj || !os.obj->CanConvertToType(polyObjectClassID)) {
        j += ", \"error\": \"not a mesh\"}";
        out.write(out.ctx, j.data(), j.size());
        return false;
    }
    PolyObject* po = static_cast<PolyObject*>(os.obj->ConvertToType(t, polyObjectClassID));
    MNMesh& mm = po->GetMesh();

    int liveVerts = 0, liveFaces = 0;
    std::map<int, int> degrees, materialIds;
    std::set<DWORD> smGroups;
    int smoothFaces = 0;
    for (int i = 0; i < mm.VNum(); ++i) liveVerts += !mm.V(i)->GetFlag(MN_DEAD);
    for (int i = 0; i < mm.FNum(); ++i) {
        const MNFace* f = mm.F(i);
        if (f->GetFlag(MN_DEAD)) continue;
        ++liveFaces;
        ++degrees[f->deg];
        ++materialIds[f->material + 1];
        if (f->smGroup) {
            ++smoothFaces;
            for (int b = 0; b < 32; ++b)
                if (f->smGroup & (1u << b)) smGroups.insert(b + 1);
        }
    }
    j += ", \"mesh\": {\"source\": " + json_string(st.skin ? "input of Skin" : "final") + ", \"class\": " + json_string(class_name(os.obj)) +
         ", \"verts\": " + std::to_string(liveVerts) + ", \"faces\": " + std::to_string(liveFaces) + ", \"degrees\": {";
    {
        bool first = true;
        for (auto [d, n] : degrees) { j += (first ? "\"" : ", \"") + std::to_string(d) + "\": " + std::to_string(n); first = false; }
    }
    j += "}, \"material_ids\": {";
    {
        bool first = true;
        for (auto [id, n] : materialIds) { j += (first ? "\"" : ", \"") + std::to_string(id) + "\": " + std::to_string(n); first = false; }
    }
    j += "}, \"smoothing\": {\"faces\": " + std::to_string(smoothFaces) + ", \"groups\": " + std::to_string(smGroups.size()) + "}";

    j += ", \"map_channels\": [";
    {
        bool first = true;
        for (int mp = -NUM_HIDDENMAPS; mp < mm.MNum(); ++mp) {
            const MNMap* map = mm.M(mp);
            if (!map || map->GetFlag(MN_DEAD)) continue;
            j += std::string(first ? "" : ", ") + "{\"channel\": " + std::to_string(mp) + ", \"verts\": " + std::to_string(map->VNum()) +
                 ", \"faces\": " + std::to_string(map->FNum()) + "}";
            first = false;
        }
    }
    j += "]";
    if (MNNormalSpec* ns = mm.GetSpecifiedNormals())
        j += ", \"normals\": {\"normals\": " + std::to_string(ns->GetNumNormals()) + ", \"faces\": " + std::to_string(ns->GetNumFaces()) + "}";
    else
        j += ", \"normals\": null";
    j += "}";  // mesh

    // Which mesh Eval returned: compare with the base object (equal when nothing sits below Skin)
    // and with the node's final mesh (different while Skin deforms away from the bind pose).
    if (st.skin && base && base->CanConvertToType(polyObjectClassID)) {
        PolyObject* bp = static_cast<PolyObject*>(base->ConvertToType(t, polyObjectClassID));
        ObjectState fin = node->EvalWorldState(t);
        PolyObject* fp = fin.obj && fin.obj->CanConvertToType(polyObjectClassID)
                             ? static_cast<PolyObject*>(fin.obj->ConvertToType(t, polyObjectClassID)) : nullptr;
        auto maxDiff = [&](const MNMesh& a) {
            if (a.VNum() != mm.VNum()) return -1.0;
            double d = 0;
            for (int i = 0; i < a.VNum(); ++i) d = std::max(d, (double)Length(a.P(i) - mm.P(i)));
            return d;
        };
        j += ", \"eval_check\": {\"vs_base\": " + std::to_string(maxDiff(bp->GetMesh())) +
             ", \"vs_final\": " + (fp ? std::to_string(maxDiff(fp->GetMesh())) : std::string("null")) + "}";
        if (bp != base) bp->MaybeAutoDelete();
        if (fp && fp != fin.obj) fp->MaybeAutoDelete();
    }
    if (po != os.obj) po->MaybeAutoDelete();

    // Materials
    if (Mtl* mtl = node->GetMtl()) {
        j += ", \"material\": {\"name\": " + json_string(utf8(mtl->GetName())) + ", \"class\": " + json_string(class_name(mtl)) +
             ", \"multi\": " + (mtl->IsMultiMtl() ? "true" : "false") + ", \"subs\": [";
        if (mtl->IsMultiMtl()) {
            for (int i = 0; i < mtl->NumSubMtls(); ++i) {
                Mtl* sub = mtl->GetSubMtl(i);
                j += std::string(i ? ", " : "") + "{\"index\": " + std::to_string(i + 1) + ", \"slot\": " +
                     json_string(utf8(mtl->GetSubMtlSlotName(i, false))) + ", \"material\": " + (sub ? material_json(sub) : "null") + "}";
            }
        } else {
            j += material_json(mtl);
        }
        j += "]}";
    }

    // Skin
    if (st.skin) {
        ISkinContextData* ctx = st.skin->GetContextInterface(node);
        const int bones = st.skin->GetNumBones();
        j += ", \"skin\": {\"modifier_index\": " + std::to_string(st.skinIndex) + ", \"bones\": " + std::to_string(bones);
        if (ctx) {
            const int points = ctx->GetNumPoints();
            std::map<int, int> influences;
            int maxInfluences = 0;
            double worstSum = 0;
            for (int v = 0; v < points; ++v) {
                const int n = ctx->GetNumAssignedBones(v);
                ++influences[n];
                maxInfluences = std::max(maxInfluences, n);
                double sum = 0;
                for (int k = 0; k < n; ++k) sum += ctx->GetBoneWeight(v, k);
                if (n) worstSum = std::max(worstSum, std::abs(sum - 1.0));
            }
            j += ", \"points\": " + std::to_string(points) + ", \"max_influences\": " + std::to_string(maxInfluences) +
                 ", \"worst_weight_sum_error\": " + std::to_string(worstSum) + ", \"influences\": {";
            bool first = true;
            for (auto [n, c] : influences) { j += (first ? "\"" : ", \"") + std::to_string(n) + "\": " + std::to_string(c); first = false; }
            j += "}";
        }
        j += "}";
    }
    j += "}";
    out.write(out.ctx, j.data(), j.size());
    return true;
}

// ------------------------------------------------------------------------------------------------
// MNMesh and udimpacker's mesh. Corners are numbered over the live faces in order. Every corner
// gets a wedge id: its values in every map channel, its explicit normal and its face's smoothing
// group; a change in any of them is a seam the algorithms keep.

struct CornerTable {
    std::vector<int> faces;         // live faces in order
    std::vector<uint32_t> first;    // per live face: its first corner (+ total at the end)
    std::vector<int> face, k;       // per corner: face and position in it
};

CornerTable corners_of(MNMesh& m) {
    CornerTable t;
    for (int f = 0; f < m.FNum(); ++f) {
        const MNFace* F = m.F(f);
        if (F->GetFlag(MN_DEAD)) continue;
        t.faces.push_back(f);
        t.first.push_back((uint32_t)t.face.size());
        for (int k = 0; k < F->deg; ++k) {
            t.face.push_back(f);
            t.k.push_back(k);
        }
    }
    t.first.push_back((uint32_t)t.face.size());
    return t;
}

std::vector<int> live_maps(MNMesh& m) {
    std::vector<int> maps;
    for (int mp = -NUM_HIDDENMAPS; mp < m.MNum(); ++mp) {
        const MNMap* map = m.M(mp);
        if (map && !map->GetFlag(MN_DEAD) && map->FNum() == m.FNum()) maps.push_back(mp);
    }
    return maps;
}

MNNormalSpec* live_normals(MNMesh& m) {
    MNNormalSpec* ns = m.GetSpecifiedNormals();
    return ns && ns->GetNumFaces() == m.FNum() ? ns : nullptr;
}

struct Fnv {
    uint64_t h = 1469598103934665603ull;
    void add(uint64_t x) {
        for (int i = 0; i < 8; ++i) {
            h ^= (x >> (8 * i)) & 0xff;
            h *= 1099511628211ull;
        }
    }
};

// Settings of one kind of plan, and the core build (a new build makes new plans).
Fnv settings_hash(char kind) {
    Fnv h;
    for (const char* p = kBuild; *p; ++p) h.add((unsigned char)*p);
    h.add((unsigned char)kind);
    return h;
}

// The input topology a plan depends on: faces, vertices, map faces, normal faces, materials,
// smoothing groups. Positions are left out: below Skin they do not change.
uint64_t topology_hash(MNMesh& m, Fnv h) {
    h.add((uint64_t)m.VNum());
    h.add((uint64_t)m.FNum());
    const std::vector<int> maps = live_maps(m);
    MNNormalSpec* ns = live_normals(m);
    for (int f = 0; f < m.FNum(); ++f) {
        const MNFace* F = m.F(f);
        if (F->GetFlag(MN_DEAD)) { h.add(~0ull); continue; }
        h.add((uint64_t)F->deg);
        h.add(F->material);
        h.add(F->smGroup);
        for (int k = 0; k < F->deg; ++k) h.add((uint64_t)F->vtx[k]);
        for (int mp : maps) {
            h.add((uint64_t)(mp + 64));
            for (int k = 0; k < F->deg; ++k) h.add((uint64_t)m.M(mp)->F(f)->tv[k]);
        }
        if (ns)
            for (int k = 0; k < F->deg; ++k) h.add((uint64_t)(int64_t)ns->Face(f).GetNormalID(k));
    }
    return h.h;
}

// The mesh as udimpacker's: corners carry their input corner (cornerSrc) and wedge (cornerSeam).
// Without `values` the wedges stand in for the UVs (unsubdivide only compares them); with it the
// UVs are channel 1, the normals the explicit normals, and the faces are triangulated (decimate).
InMesh to_inmesh(MNMesh& m, const CornerTable& ct, bool values) {
    const std::vector<int> maps = live_maps(m);
    MNNormalSpec* ns = live_normals(m);
    const MNMap* uv = values && std::find(maps.begin(), maps.end(), 1) != maps.end() ? m.M(1) : nullptr;
    InMesh im;
    im.name = "max";
    im.positions.resize(m.VNum());
    for (int v = 0; v < m.VNum(); ++v) im.positions[v] = {m.P(v).x, m.P(v).y, m.P(v).z};
    if (values) {
        if (uv)
            for (int i = 0; i < uv->VNum(); ++i) im.uvs.push_back({uv->v[i].x, uv->v[i].y});
        im.uvs.push_back({});  // the UV of corners without one
        if (ns) {
            for (int i = 0; i < ns->GetNumNormals(); ++i) im.normals.push_back({ns->Normal(i).x, ns->Normal(i).y, ns->Normal(i).z});
            im.normals.push_back({0, 0, 1});  // the normal of corners without one
        }
    }
    std::unordered_map<std::string, uint32_t> wedges;
    std::string key;
    auto put = [&](int32_t x) { key.append(reinterpret_cast<const char*>(&x), sizeof x); };
    Tab<int> tri;
    for (size_t i = 0; i < ct.faces.size(); ++i) {
        const int f = ct.faces[i];
        MNFace* F = m.F(f);
        im.faces.push_back({ct.first[i], (uint32_t)F->deg, (uint32_t)F->material});
        for (int k = 0; k < F->deg; ++k) {
            im.cornerPos.push_back((uint32_t)F->vtx[k]);
            im.cornerSrc.push_back(ct.first[i] + k);
            key.clear();
            put((int32_t)F->smGroup);
            put(ns ? ns->Face(f).GetNormalID(k) : -1);
            for (int mp : maps) put(m.M(mp)->F(f)->tv[k]);
            const uint32_t wedge = wedges.emplace(key, (uint32_t)wedges.size()).first->second;
            im.cornerSeam.push_back(wedge);
            if (!values) {
                im.cornerUv.push_back(wedge);
            } else {
                im.cornerUv.push_back(uv ? (uint32_t)uv->F(f)->tv[k] : (uint32_t)im.uvs.size() - 1);
                if (ns) {
                    const int n = ns->Face(f).GetNormalID(k);
                    im.cornerNrm.push_back(n >= 0 ? (uint32_t)n : (uint32_t)im.normals.size() - 1);
                }
            }
        }
        if (values) {
            F->GetTriangles(tri);
            for (int t = 0; t + 2 < tri.Count(); t += 3) {
                im.tris.push_back({ct.first[i] + tri[t], ct.first[i] + tri[t + 1], ct.first[i] + tri[t + 2]});
                im.triFace.push_back((uint32_t)i);
            }
        }
    }
    if (!values) im.uvs.resize(wedges.size());
    return im;
}

// The plan from an algorithm's result mesh (corners carry their input corner).
bool finish_plan(MNMesh& m, const CornerTable& ct, const InMesh& r, UdimPlan& plan) {
    if (r.cornerSrc.size() != r.cornerPos.size()) return false;
    plan.faceFirst.clear();
    plan.corner.clear();
    plan.vertex.clear();
    for (const InFace& f : r.faces) {
        plan.faceFirst.push_back((uint32_t)plan.corner.size());
        for (uint32_t k = 0; k < f.count; ++k) plan.corner.push_back(r.cornerSrc[f.first + k]);
    }
    plan.faceFirst.push_back((uint32_t)plan.corner.size());
    std::vector<uint8_t> used(m.VNum(), 0);
    for (uint32_t c : plan.corner) used[m.F(ct.face[c])->vtx[ct.k[c]]] = 1;
    for (int v = 0; v < m.VNum(); ++v)
        if (used[v]) plan.vertex.push_back((uint32_t)v);
    return true;
}

uint64_t unsubdivide_signature(MNMesh& m, const UdimUnsubdivide& s) {
    Fnv h = settings_hash('U');
    h.add((uint64_t)s.levels);
    h.add(s.symmetry);
    return topology_hash(m, h);
}

bool unsubdivide_plan(MNMesh& m, const UdimUnsubdivide& s, UdimPlan& plan) {
    plan = {};
    plan.signature = unsubdivide_signature(m, s);
    const CornerTable ct = corners_of(m);
    InScene scene;
    scene.meshes.push_back(to_inmesh(m, ct, false));
    UnsubdivideOptions opt;
    opt.levels = s.levels;
    opt.symmetry = s.symmetry;
    const SubdivReport rep = unsubdivide_scene(scene, opt);
    if (!finish_plan(m, ct, scene.meshes[0], plan)) return false;

    const SubdivReport::Mesh& mr = rep.meshes[0];
    char line[256];
    std::snprintf(line, sizeof line, "%zu -> %zu faces", mr.facesBefore, mr.facesAfter);
    plan.report = line;
    if (mr.levels) plan.report += ", " + std::to_string(mr.levels) + " exact";
    if (mr.approxLevels) plan.report += ", " + std::to_string(mr.approxLevels) + " approx";
    if (mr.mirrorAxis >= 0) {
        std::snprintf(line, sizeof line, ", mirror %c %.0f%%", "xyz"[mr.mirrorAxis], mr.mirrored * 100);
        plan.report += line;
    }
    if (!mr.stop.empty()) plan.report += " (stopped: " + mr.stop + ")";
    return true;
}

uint64_t decimate_signature(MNMesh& m, const UdimDecimate& s) {
    Fnv h = settings_hash('D');
    h.add(std::bit_cast<uint32_t>(s.percent));
    h.add(s.lockBorder);
    return topology_hash(m, h);
}

bool decimate_plan(MNMesh& m, const UdimDecimate& s, UdimPlan& plan) {
    plan = {};
    plan.signature = decimate_signature(m, s);
    const CornerTable ct = corners_of(m);
    InScene scene;
    scene.meshes.push_back(to_inmesh(m, ct, true));
    const size_t tris = scene.meshes[0].tris.size();
    uint32_t maxMaterial = 0;
    for (const InFace& f : scene.meshes[0].faces) maxMaterial = std::max(maxMaterial, f.material);
    DecimateOptions opt;
    opt.targetTris = std::max<size_t>(1, (size_t)std::llround((double)tris * std::clamp((double)s.percent, 0.0, 100.0) / 100.0));
    opt.maxError = -1;
    opt.lockBorder = s.lockBorder;
    const DecimateResult r = decimate_scene(scene, std::vector<uint8_t>(maxMaterial + 1, 0), opt);
    if (scene.meshes.empty() || !finish_plan(m, ct, scene.meshes[0], plan)) return false;
    char line[256];
    std::snprintf(line, sizeof line, "%zu -> %zu triangles, error %.3g%%", r.before, r.after, r.error * 100);
    plan.report = line;
    return true;
}

// Values of an input array used by the plan, renumbered in their original order.
struct Remap {
    std::vector<int> to;     // input id -> output id, -1 = unused
    std::vector<int> from;   // output id -> input id
    explicit Remap(int n) : to(n, -1) {}
    void use(int i) { if (i >= 0 && i < (int)to.size()) to[i] = 0; }
    void finish() {
        for (int i = 0; i < (int)to.size(); ++i)
            if (to[i] == 0) { to[i] = (int)from.size(); from.push_back(i); }
    }
    int operator[](int i) const { return i >= 0 && i < (int)to.size() ? to[i] : -1; }
};

// The result: the plan's faces with the input's own vertices, map channels, explicit normals,
// material IDs and smoothing groups, each renumbered in its original order.
bool apply_plan(MNMesh& m, const UdimPlan& plan, MNMesh& out) {
    const CornerTable ct = corners_of(m);
    const size_t nc = ct.face.size();
    for (uint32_t c : plan.corner)
        if (c >= nc) return false;
    const int nf = (int)plan.faceFirst.size() - 1;
    if (nf < 0) return false;
    auto srcFace = [&](int f) { return m.F(ct.face[plan.corner[plan.faceFirst[f]]]); };

    Remap verts(m.VNum());
    for (uint32_t c : plan.corner) verts.use(m.F(ct.face[c])->vtx[ct.k[c]]);
    verts.finish();

    out.ClearAndFree();
    out.setNumVerts((int)verts.from.size());
    for (int i = 0; i < (int)verts.from.size(); ++i) out.V(i)->p = m.P(verts.from[i]);
    out.setNumFaces(nf);
    for (int f = 0; f < nf; ++f) {
        MNFace* F = out.F(f);
        const int deg = (int)(plan.faceFirst[f + 1] - plan.faceFirst[f]);
        F->SetDeg(deg);
        for (int k = 0; k < deg; ++k) {
            const uint32_t c = plan.corner[plan.faceFirst[f] + k];
            F->vtx[k] = verts[m.F(ct.face[c])->vtx[ct.k[c]]];
        }
        F->material = srcFace(f)->material;
        F->smGroup = srcFace(f)->smGroup;
    }

    out.SetMapNum(m.MNum());
    const std::vector<int> maps = live_maps(m);
    for (int mp = -NUM_HIDDENMAPS; mp < m.MNum(); ++mp) {
        if (std::find(maps.begin(), maps.end(), mp) == maps.end()) {
            out.ClearMap(mp);
            continue;
        }
        MNMap* s = m.M(mp);
        MNMap* d = out.M(mp);
        Remap tv(s->VNum());
        for (uint32_t c : plan.corner) tv.use(s->F(ct.face[c])->tv[ct.k[c]]);
        tv.finish();
        d->ClearAndFree();
        d->ClearFlag(MN_DEAD);
        d->setNumVerts((int)tv.from.size());
        for (int i = 0; i < (int)tv.from.size(); ++i) d->v[i] = s->v[tv.from[i]];
        d->setNumFaces(nf);
        for (int f = 0; f < nf; ++f) {
            const int deg = (int)(plan.faceFirst[f + 1] - plan.faceFirst[f]);
            d->F(f)->SetSize(deg);
            for (int k = 0; k < deg; ++k) {
                const uint32_t c = plan.corner[plan.faceFirst[f] + k];
                d->F(f)->tv[k] = tv[s->F(ct.face[c])->tv[ct.k[c]]];
            }
        }
    }

    out.FillInMesh();
    for (int f = 0; f < nf; ++f)
        if (out.F(f)->deg > 3) out.RetriangulateFace(f);

    if (MNNormalSpec* sn = live_normals(m)) {
        Remap nr(sn->GetNumNormals());
        for (uint32_t c : plan.corner) nr.use(sn->Face(ct.face[c]).GetNormalID(ct.k[c]));
        nr.finish();
        out.SpecifyNormals();
        MNNormalSpec* dn = out.GetSpecifiedNormals();
        dn->SetParent(&out);
        dn->SetNumFaces(nf);
        dn->SetNumNormals((int)nr.from.size());
        for (int i = 0; i < (int)nr.from.size(); ++i) {
            dn->Normal(i) = sn->Normal(nr.from[i]);
            dn->SetNormalExplicit(i, sn->GetNormalExplicit(nr.from[i]));
        }
        for (int f = 0; f < nf; ++f) {
            const int deg = (int)(plan.faceFirst[f + 1] - plan.faceFirst[f]);
            MNNormalFace& fn = dn->Face(f);
            fn.SetDegree(deg);
            for (int k = 0; k < deg; ++k) {
                const uint32_t c = plan.corner[plan.faceFirst[f] + k];
                fn.SetNormalID(k, nr[sn->Face(ct.face[c]).GetNormalID(ct.k[c])]);
                fn.SetSpecified(k, sn->Face(ct.face[c]).GetSpecified(ct.k[c]));
            }
        }
        dn->CheckNormals();
    }
    return true;
}

// ------------------------------------------------------------------------------------------------
// Reducing modifiers below Skin (Unsubdivide, Decimate). Skin remaps its weights by position when
// the vertex count changes: a vertex within 0.00001 of an old one takes its weights exactly
// (bonesdef RemapVertData), others get a blend of their neighbours. Every vertex these modifiers
// keep is an input vertex at its input position, so going down in resolution is exact on its own.
// A vertex that comes back is one Skin no longer has, so each modifier keeps a copy of Skin's
// weights by position and the one nearest the Skin writes them back after the evaluation.

// The node a modifier context belongs to: Max passes no node to an object space modifier's
// ModifyObject (one modifier can sit on several nodes), so it is found the way the SDK's own
// modifiers do: the nodes that depend on the modifier, then the one whose pipeline holds `mc`.
INode* node_of(Modifier* mod, ModContext* mc) {
    struct Nodes : DependentEnumProc {
        std::vector<INode*> found;
        int proc(ReferenceMaker* rm) override {
            if (rm->SuperClassID() != BASENODE_CLASS_ID) return DEP_ENUM_CONTINUE;
            found.push_back(static_cast<INode*>(rm));
            return DEP_ENUM_SKIP;
        }
    } nodes;
    mod->DoEnumDependents(&nodes);
    for (INode* n : nodes.found)
        for (Object* obj = n->GetObjectRef(); obj && obj->SuperClassID() == GEN_DERIVOB_CLASS_ID;
             obj = static_cast<IDerivedObject*>(obj)->GetObjRef()) {
            auto* d = static_cast<IDerivedObject*>(obj);
            for (int i = 0; i < d->NumModifiers(); ++i)
                if (d->GetModifier(i) == mod && d->GetModContext(i) == mc) return n;
        }
    return nullptr;
}

// The nearest Skin above a modifier on a node, and whether another reducing modifier sits between.
struct SkinAbove {
    Modifier* mod = nullptr;
    ISkin* skin = nullptr;
    ISkinContextData* ctx = nullptr;
    bool top = false;  // no other reducing modifier between this one and the Skin
};

SkinAbove skin_above(INode* node, Modifier* self, UdimIsReducer isReducer) {
    SkinAbove s;
    if (!node) return s;
    for (Object* obj = node->GetObjectRef(); obj && obj->SuperClassID() == GEN_DERIVOB_CLASS_ID;
         obj = static_cast<IDerivedObject*>(obj)->GetObjRef()) {
        auto* d = static_cast<IDerivedObject*>(obj);
        for (int i = 0; i < d->NumModifiers(); ++i) {
            if (d->GetModifier(i) != self) continue;
            s.top = true;
            for (int j = i - 1; j >= 0; --j) {
                Modifier* m = d->GetModifier(j);
                if (auto* sk = static_cast<ISkin*>(m->GetInterface(I_SKIN))) {
                    s.mod = m;
                    s.skin = sk;
                    s.ctx = sk->GetContextInterface(node);
                    break;
                }
                if (isReducer && isReducer(m)) s.top = false;
            }
            return s;
        }
    }
    return s;
}

// Positions compared bit for bit (-0 and +0 alike).
struct PosHash {
    size_t operator()(const UdimPos& p) const {
        uint64_t h = 1469598103934665603ull;
        for (float x : p) {
            h ^= std::bit_cast<uint32_t>(x + 0.0f);
            h *= 1099511628211ull;
        }
        return (size_t)h;
    }
};

struct PosIndex {
    std::unordered_map<UdimPos, uint32_t, PosHash> at;
    explicit PosIndex(const UdimSkinCopy& c) {
        for (uint32_t i = 0; i < c.pos.size(); ++i) at.try_emplace(c.pos[i], i);
    }
    uint32_t find(const UdimPos& p) const {
        auto it = at.find(p);
        return it == at.end() ? kNone : it->second;
    }
};

UdimPos pos_of(MNMesh& m, int v) { return {m.P(v).x, m.P(v).y, m.P(v).z}; }

INode* copy_bone(UdimSkinCopy& c, uint32_t i) {
    if (!c.bones[i] && !c.boneNames[i].empty()) c.bones[i] = GetCOREInterface()->GetINodeByName(c.boneNames[i].c_str());
    return c.bones[i];
}

uint32_t copy_bone_index(UdimSkinCopy& c, INode* node) {
    for (uint32_t i = 0; i < c.bones.size(); ++i)
        if (copy_bone(c, i) == node) return i;
    c.bones.push_back(node);
    c.boneNames.push_back(node ? node->GetName() : L"");
    return (uint32_t)c.bones.size() - 1;
}

// Skin's weights of its vertex i against copied vertex v: the same bones, each weight within
// `tolerance` (0 = identical). Skin normalizes what it is given, which can move the last bit.
bool skin_matches(const SkinAbove& s, int i, UdimSkinCopy& c, uint32_t v, float tolerance) {
    const int n = s.ctx->GetNumAssignedBones(i);
    if (n != (int)(c.first[v + 1] - c.first[v])) return false;
    for (int k = 0; k < n; ++k) {
        INode* b = s.skin->GetBone(s.ctx->GetAssignedBone(i, k));
        const float w = s.ctx->GetBoneWeight(i, k);
        bool found = false;
        for (uint32_t x = c.first[v]; x < c.first[v + 1] && !found; ++x)
            found = copy_bone(c, c.bone[x]) == b && std::abs(c.weight[x] - w) <= tolerance;
        if (!found) return false;
    }
    return true;
}

// Appends Skin's weights of its vertex i as copied vertex at `p`.
void copy_skin_vertex(const SkinAbove& s, int i, const UdimPos& p, UdimSkinCopy& c) {
    if (c.first.empty()) c.first.push_back(0);
    c.pos.push_back(p);
    for (int k = 0; k < s.ctx->GetNumAssignedBones(i); ++k) {
        c.bone.push_back(copy_bone_index(c, s.skin->GetBone(s.ctx->GetAssignedBone(i, k))));
        c.weight.push_back(s.ctx->GetBoneWeight(i, k));
    }
    c.first.push_back((uint32_t)c.bone.size());
}

// Takes Skin's weights of every input vertex while Skin still has them all.
void capture(const SkinAbove& s, MNMesh& input, UdimSkinCopy& c) {
    c.pos.clear();
    c.first.clear();
    c.bone.clear();
    c.weight.clear();
    c.synced.clear();
    for (int v = 0; v < input.VNum(); ++v) {
        copy_skin_vertex(s, v, pos_of(input, v), c);
        c.synced.push_back(c.pos.back());
    }
    c.inSync = true;
}

// Before the vertex set changes: weights painted on the current vertices go into the copy
// (vertices Skin still has as copied, float noise aside, keep their copied values).
void fold_back(const SkinAbove& s, UdimSkinCopy& c) {
    if (!c.inSync || s.ctx->GetNumPoints() != (int)c.synced.size()) return;
    const PosIndex index(c);
    std::vector<int> painted(c.pos.size(), -1);  // copied vertex -> Skin vertex whose weights replace it
    std::vector<int> added;                      // Skin vertices the copy does not have
    for (size_t i = 0; i < c.synced.size(); ++i) {
        const uint32_t v = index.find(c.synced[i]);
        if (v == kNone) added.push_back((int)i);
        else if (!skin_matches(s, (int)i, c, v, 1e-6f)) painted[v] = (int)i;
    }
    if (added.empty() && std::all_of(painted.begin(), painted.end(), [](int i) { return i < 0; })) return;
    UdimSkinCopy out = c;
    out.pos.clear();
    out.first.clear();
    out.bone.clear();
    out.weight.clear();
    for (size_t v = 0; v < c.pos.size(); ++v) {
        if (painted[v] >= 0) {
            copy_skin_vertex(s, painted[v], c.pos[v], out);
            continue;
        }
        if (out.first.empty()) out.first.push_back(0);
        out.pos.push_back(c.pos[v]);
        for (uint32_t x = c.first[v]; x < c.first[v + 1]; ++x) {
            out.bone.push_back(copy_bone_index(out, copy_bone(c, c.bone[x])));
            out.weight.push_back(c.weight[x]);
        }
        out.first.push_back((uint32_t)out.bone.size());
    }
    for (int i : added) copy_skin_vertex(s, i, c.synced[i], out);
    c = std::move(out);
}

std::wstring wide(const std::string& s) { return std::wstring(MSTR::FromUTF8(s.c_str()).data()); }

// The body of a reducing modifier's ModifyObject; `plan` makes a fresh plan when the cached one
// does not match `signature`.
template <class MakePlan>
bool reduce_modify(Modifier* self, ModContext& mc, INode* node, MNMesh& mesh, bool active, uint64_t signature, MakePlan&& plan,
                   UdimReduceState& st, UdimIsReducer isReducer, std::wstring& status, INode*& nodeOut) {
    if (!node) node = node_of(self, &mc);
    nodeOut = node;
    const SkinAbove skin = skin_above(node, self, isReducer);
    if (skin.ctx && skin.top && st.skin.empty() && skin.ctx->GetNumPoints() == mesh.VNum()) capture(skin, mesh, st.skin);

    std::string report = "off";
    if (active) {
        if (st.plan.signature != signature) plan(st.plan);
        MNMesh result;
        if (apply_plan(mesh, st.plan, result)) {
            mesh = result;
            report = st.plan.report;
        } else {
            report = "the plan does not fit the input";
        }
    }
    std::vector<UdimPos> current(mesh.VNum());
    for (int v = 0; v < mesh.VNum(); ++v) current[v] = pos_of(mesh, v);

    bool write = false;
    if (skin.ctx && !skin.top) {
        report += "\nSkin weights: kept by the modifier above";
    } else if (skin.ctx) {
        if (st.current != current && st.skin.inSync) fold_back(skin, st.skin);
        if (st.skin.empty()) {
            report += "\nSkin weights: no copy (Skin had other vertices when the modifier was added)";
        } else if (st.skin.inSync && st.skin.synced == current) {
            report += "\nSkin weights: exact";
        } else {
            report += "\nSkin weights: updating";
            st.skin.inSync = false;
            write = !st.pending;
        }
    }
    st.current = std::move(current);
    status = wide(report);
    return write;
}

bool unsubdivide_modify(Modifier* self, ModContext& mc, INode* node, MNMesh& mesh, const UdimUnsubdivide& s, UdimReduceState& st,
                        UdimIsReducer isReducer, std::wstring& status, INode*& nodeOut) {
    return reduce_modify(self, mc, node, mesh, s.levels > 0, unsubdivide_signature(mesh, s),
                         [&](UdimPlan& p) { unsubdivide_plan(mesh, s, p); }, st, isReducer, status, nodeOut);
}

bool decimate_modify(Modifier* self, ModContext& mc, INode* node, MNMesh& mesh, const UdimDecimate& s, UdimReduceState& st,
                     UdimIsReducer isReducer, std::wstring& status, INode*& nodeOut) {
    return reduce_modify(self, mc, node, mesh, s.percent < 100, decimate_signature(mesh, s),
                         [&](UdimPlan& p) { decimate_plan(mesh, s, p); }, st, isReducer, status, nodeOut);
}

bool write_weights(INode* node, Modifier* self, UdimReduceState& st, const std::vector<UdimSkinCopy*>& below) {
    if (!node || st.skin.empty()) return false;
    node->EvalWorldState(GetCOREInterface()->GetTime());  // Skin remaps to the current vertices
    const SkinAbove skin = skin_above(node, self, nullptr);
    auto* import = skin.mod ? static_cast<ISkinImportData*>(skin.mod->GetInterface(I_SKINIMPORTDATA)) : nullptr;
    if (!skin.ctx || !import || skin.ctx->GetNumPoints() != (int)st.current.size()) return false;
    const PosIndex own(st.skin);
    std::vector<PosIndex> lower;
    for (UdimSkinCopy* c : below) lower.emplace_back(*c);
    Tab<INode*> bones;
    Tab<float> weights;
    int written = 0;
    for (size_t i = 0; i < st.current.size(); ++i) {
        UdimSkinCopy* c = &st.skin;
        uint32_t v = own.find(st.current[i]);
        for (size_t k = 0; v == kNone && k < below.size(); ++k)
            if ((v = lower[k].find(st.current[i])) != kNone) c = below[k];
        if (v == kNone || skin_matches(skin, (int)i, *c, v, 0.0f)) continue;  // unknown, or Skin still has it as copied
        const int n = (int)(c->first[v + 1] - c->first[v]);
        bones.SetCount(n);
        weights.SetCount(n);
        for (int k = 0; k < n; ++k) {
            bones[k] = copy_bone(*c, c->bone[c->first[v] + k]);
            weights[k] = c->weight[c->first[v] + k];
        }
        import->AddWeights(node, (int)i, bones, weights);
        ++written;
    }
    st.skin.synced = st.current;
    st.skin.inSync = true;
    if (written) skin.mod->NotifyDependents(FOREVER, PART_GEOM, REFMSG_CHANGE);
    return true;
}

// ------------------------------------------------------------------------------------------------
// UDIM Pack. One modifier instance sits on every part of a character and the layout is made from
// all of them, so it is kept here per modifier and each part's evaluation writes its share. The
// maps come per UDIM tile from the settings: every tile with maps becomes one material of the
// packer's scene, faces in other tiles are left out. Materials in the scene are not read.

// One node's input: the faces and source UVs the layout is made from.
struct PackPart {
    uint64_t signature = 0;  // source channel, face degrees and triangles, map faces and values
    int channel = 0;
    InMesh mesh;             // faces (materials set when the scene is put together), source UVs, triangles
    std::vector<UVVert> uvw; // the source map's values, w included
};

struct PackUVs {  // one node's packed UVs
    std::vector<UVVert> verts;
    std::vector<int> tv;  // per corner of the live faces in order
};

struct PackState {
    std::map<ULONG, PackPart> parts;  // by node handle
    uint64_t layout = 0;              // the signature the layout below was made for
    bool ok = false;
    std::unique_ptr<Packer> packer;
    std::map<ULONG, PackUVs> uvs;
    std::string report, log;          // status line and the packer's report of the layout
    std::vector<UdimPackTile> tiles;
};

std::recursive_mutex packLock;
std::map<Modifier*, PackState> packs;
std::set<Modifier*> packBusy;  // modifiers in the middle of pack_modify / pack_bake

// Collects the packer's report (say) while alive.
struct SayCapture {
    static inline std::string* target = nullptr;
    std::string* prevTarget;
    void (*prevHook)(const char*);
    explicit SayCapture(std::string& s) : prevTarget(target), prevHook(say_hook) {
        target = &s;
        say_hook = [](const char* text) { if (target) *target += text; };
    }
    ~SayCapture() {
        target = prevTarget;
        say_hook = prevHook;
    }
};

void add_text(Fnv& h, const std::wstring& s) {
    h.add(s.size());
    for (wchar_t c : s) h.add((uint64_t)c);
}

// Where the modifier sits: every node whose stack holds it, by handle.
struct PackUse {
    INode* node;
    IDerivedObject* owner;
    int index;
};

std::vector<PackUse> uses_of(Modifier* mod) {
    struct Nodes : DependentEnumProc {
        std::vector<INode*> found;
        int proc(ReferenceMaker* rm) override {
            if (rm->SuperClassID() != BASENODE_CLASS_ID) return DEP_ENUM_CONTINUE;
            found.push_back(static_cast<INode*>(rm));
            return DEP_ENUM_SKIP;
        }
    } nodes;
    mod->DoEnumDependents(&nodes);
    std::vector<PackUse> uses;
    for (INode* n : nodes.found) {
        if (!n->GetParentNode()) continue;  // deleted (kept for undo)
        bool found = false;
        for (Object* obj = n->GetObjectRef(); obj && obj->SuperClassID() == GEN_DERIVOB_CLASS_ID && !found;
             obj = static_cast<IDerivedObject*>(obj)->GetObjRef()) {
            auto* d = static_cast<IDerivedObject*>(obj);
            for (int i = 0; i < d->NumModifiers() && !found; ++i)
                if (d->GetModifier(i) == mod) {
                    uses.push_back({n, d, i});
                    found = true;
                }
        }
    }
    std::sort(uses.begin(), uses.end(), [](const PackUse& a, const PackUse& b) { return a.node->GetHandle() < b.node->GetHandle(); });
    uses.erase(std::unique(uses.begin(), uses.end(), [](const PackUse& a, const PackUse& b) { return a.node == b.node; }), uses.end());
    return uses;
}

bool make_part(MNMesh& m, int channel, PackPart& p, std::string& err) {
    MNMap* map = channel >= 1 && channel < m.MNum() ? m.M(channel) : nullptr;
    if (!map || map->GetFlag(MN_DEAD) || map->FNum() != m.FNum() || map->VNum() == 0) {
        err = "no UVs in map channel " + std::to_string(channel);
        return false;
    }
    const CornerTable ct = corners_of(m);
    Fnv h;
    h.add((uint64_t)channel);
    p = {};
    p.channel = channel;
    p.uvw.assign(map->v, map->v + map->VNum());
    h.add(p.uvw.size());
    InMesh& im = p.mesh;
    im.positions.resize(m.VNum());
    for (int v = 0; v < m.VNum(); ++v) im.positions[v] = {m.P(v).x, m.P(v).y, m.P(v).z};
    for (const UVVert& uv : p.uvw) {
        im.uvs.push_back({uv.x, uv.y});
        h.add(std::bit_cast<uint32_t>(uv.x));
        h.add(std::bit_cast<uint32_t>(uv.y));
        h.add(std::bit_cast<uint32_t>(uv.z));
    }
    Tab<int> tri;
    for (size_t i = 0; i < ct.faces.size(); ++i) {
        const int f = ct.faces[i];
        MNFace* F = m.F(f);
        const MNMapFace* mf = map->F(f);
        if (mf->deg != F->deg) {
            err = "map channel " + std::to_string(channel) + " does not match the faces";
            return false;
        }
        im.faces.push_back({ct.first[i], (uint32_t)F->deg, 0});
        h.add((uint64_t)F->deg);
        for (int k = 0; k < F->deg; ++k) {
            if (mf->tv[k] < 0 || mf->tv[k] >= map->VNum()) {
                err = "map channel " + std::to_string(channel) + " has broken faces";
                return false;
            }
            im.cornerPos.push_back((uint32_t)F->vtx[k]);
            im.cornerUv.push_back((uint32_t)mf->tv[k]);
            h.add((uint64_t)mf->tv[k]);
        }
        F->GetTriangles(tri);
        for (int t = 0; t + 2 < tri.Count(); t += 3) {
            im.tris.push_back({ct.first[i] + tri[t], ct.first[i] + tri[t + 1], ct.first[i] + tri[t + 2]});
            im.triFace.push_back((uint32_t)i);
            for (int k = 0; k < 3; ++k) h.add((uint64_t)tri[t + k]);
        }
    }
    p.signature = h.h;
    return true;
}

// A node's input, evaluated below the modifier.
bool read_part(const PackUse& u, TimeValue t, int channel, PackPart& p, std::string& err) {
    ObjectState os = u.index + 1 < u.owner->NumModifiers() ? u.owner->Eval(t, u.index + 1) : u.owner->GetObjRef()->Eval(t);
    if (!os.obj || !os.obj->CanConvertToType(polyObjectClassID)) {
        err = "not a mesh";
        return false;
    }
    auto* po = static_cast<PolyObject*>(os.obj->ConvertToType(t, polyObjectClassID));
    const bool ok = make_part(po->GetMesh(), channel, p, err);
    if (po != os.obj) po->MaybeAutoDelete();
    return ok;
}

// The tile of a face, as the packer finds it (the average of its UVs); -1 outside 1001 .. 1999.
int face_udim(const InMesh& m, const InFace& f) {
    double cu = 0, cv = 0;
    for (uint32_t c = 0; c < f.count; ++c) {
        const Vec2d& uv = m.uvs[m.cornerUv[f.first + c]];
        cu += uv.x;
        cv += uv.y;
    }
    const Tile t{(int)std::floor(cu / f.count), (int)std::floor(cv / f.count)};
    return t.u >= 0 && t.u < 10 && t.v >= 0 && t.v < 100 ? udim_number(t) : -1;
}

uint64_t layout_signature(const PackState& st, const UdimPackSettings& s) {
    Fnv h = settings_hash('P');
    for (const auto& [handle, part] : st.parts) {
        h.add(handle);
        h.add(part.signature);
    }
    h.add((uint64_t)s.size);
    h.add((uint64_t)s.maxSize);
    h.add((uint64_t)s.padding);
    h.add(s.rotate);
    for (int c = 0; c < kUdimPackChannels; ++c) {
        add_text(h, s.channel[c]);
        h.add((uint64_t)s.kind[c]);
    }
    for (const UdimPackMap& m : s.maps) {
        h.add((uint64_t)m.tile);
        h.add((uint64_t)m.channel);
        add_text(h, m.file);
    }
    return h.h;
}

// The packed UVs of the packer's mesh `mi`: new UVs for packed faces (one per source UV and
// island, as in the CLI's OBJ), the source UVs for faces left out.
PackUVs packed_uvs(const Packer& P, size_t mi, const PackPart& part) {
    const InMesh& mesh = P.scene.meshes[mi];
    PackUVs out;
    std::unordered_map<uint64_t, int> index;
    out.tv.reserve(mesh.cornerUv.size());
    for (size_t fi = 0; fi < mesh.faces.size(); ++fi) {
        const InFace& f = mesh.faces[fi];
        const int isl = P.faceIsland[mi][fi];
        const bool packed = P.faceSource[mi][fi] >= 0 && isl >= 0;
        for (uint32_t k = 0; k < f.count; ++k) {
            const uint32_t uvi = mesh.cornerUv[f.first + k];
            const uint64_t key = ((uint64_t)uvi << 32) | (packed ? (uint32_t)isl : UINT32_MAX);
            auto [it, fresh] = index.try_emplace(key, (int)out.verts.size());
            if (fresh) {
                UVVert v = part.uvw[uvi];
                if (packed) {
                    const Vec2d q = P.new_uv(P.islands[isl], mesh.uvs[uvi], P.faceTile[mi][fi]);
                    v.x = (float)q.x;
                    v.y = (float)q.y;
                }
                out.verts.push_back(v);
            }
            out.tv.push_back(it->second);
        }
    }
    return out;
}

// Tiles in use, from the parts' source UVs and the maps.
std::vector<UdimPackTile> pack_tiles(const PackState& st, const UdimPackSettings& s) {
    std::map<int, int> faces;
    for (const auto& [handle, part] : st.parts)
        for (const InFace& f : part.mesh.faces)
            if (const int udim = face_udim(part.mesh, f); udim > 0) ++faces[udim];
    for (const UdimPackMap& m : s.maps) faces.try_emplace(m.tile, 0);
    std::vector<UdimPackTile> tiles;
    for (auto [tile, n] : faces) tiles.push_back({tile, n});
    return tiles;
}

std::string last_line(const std::string& log) {
    std::string s = log;
    while (!s.empty() && (s.back() == '\n' || s.back() == ' ')) s.pop_back();
    const size_t nl = s.rfind('\n');
    s = nl == std::string::npos ? s : s.substr(nl + 1);
    return s.substr(s.find_first_not_of(' ') == std::string::npos ? 0 : s.find_first_not_of(' '));
}

void make_layout(PackState& st, const UdimPackSettings& s) {
    st.ok = false;
    st.packer.reset();
    st.uvs.clear();
    st.log.clear();
    st.tiles = pack_tiles(st, s);
    if (st.parts.empty()) {
        st.report = "no mesh with UVs";
        return;
    }

    auto P = std::make_unique<Packer>();
    PackOptions& o = P->o;
    o.sizeW = o.sizeH = s.size;
    o.maxSize = s.maxSize;
    o.padding = s.padding;
    o.rotate = s.rotate;
    o.obj = false;
    InScene& scene = P->scene;
    // Material 0 holds the faces of tiles without maps, left out; then one per tile with maps.
    scene.materials.push_back({"<no maps>"});
    o.excludes.push_back("<no maps>");
    std::map<int, uint32_t> tileMaterial;
    for (const UdimPackMap& m : s.maps) {
        if (m.channel < 0 || m.channel >= kUdimPackChannels || s.channel[m.channel].empty() || m.file.empty()) continue;
        auto [it, fresh] = tileMaterial.try_emplace(m.tile, (uint32_t)scene.materials.size());
        if (fresh) scene.materials.push_back({std::to_string(m.tile)});
        InTexture tex;
        tex.channel = utf8(s.channel[m.channel].c_str());
        tex.path = utf8(m.file.c_str());
        tex.label = ustr(upath(tex.path).filename());
        std::vector<InTexture>& textures = scene.materials[it->second].textures;
        auto same = std::find_if(textures.begin(), textures.end(), [&](const InTexture& x) { return x.channel == tex.channel; });
        if (same != textures.end()) *same = tex;
        else textures.push_back(tex);
    }
    for (int c = 0; c < kUdimPackChannels; ++c)
        if (!s.channel[c].empty()) o.kinds[utf8(s.channel[c].c_str())] = s.kind[c] == 1 ? "normal-gl" : s.kind[c] == 2 ? "normal-dx" : "color";
    if (tileMaterial.empty()) {
        st.report = "assign maps to the tiles to pack";
        return;
    }

    std::vector<ULONG> order;
    for (const auto& [handle, part] : st.parts) {
        InMesh mesh = part.mesh;
        if (INode* n = GetCOREInterface()->GetINodeByHandle(handle)) mesh.name = utf8(n->GetName());
        for (InFace& f : mesh.faces) {
            auto it = tileMaterial.find(face_udim(mesh, f));
            f.material = it == tileMaterial.end() ? 0 : it->second;
        }
        scene.meshes.push_back(std::move(mesh));
        order.push_back(handle);
    }

    SayCapture capture(st.log);
    if (!P->bind()) {
        st.report = last_line(st.log);
        return;
    }
    P->build_islands();
    if (!P->pack()) {
        st.report = last_line(st.log);
        return;
    }
    for (size_t mi = 0; mi < order.size(); ++mi) st.uvs[order[mi]] = packed_uvs(*P, mi, st.parts[order[mi]]);
    char line[256];
    std::snprintf(line, sizeof line, "%zu tile%s, %zu islands, %dx%d atlas, %zu map%s", tileMaterial.size(), tileMaterial.size() == 1 ? "" : "s",
                  P->islands.size(), P->AW, P->AH, P->channels.size(), P->channels.size() == 1 ? "" : "s");
    st.report = line;
    // The first warning, if any: unreadable maps, maps of one tile at unrelated sizes, ...
    if (const size_t w = st.log.find("  ! "); w != std::string::npos) st.report += "\n" + st.log.substr(w + 4, st.log.find('\n', w) - w - 4);
    for (int c = 0; c < kUdimPackChannels; ++c) {
        std::wstring low = s.channel[c];
        for (wchar_t& ch : low) ch = (wchar_t)towlower(ch);
        if (s.rotate && s.kind[c] == 0 && low.find(L"normal") != std::wstring::npos)
            st.report += "\n'" + utf8(s.channel[c].c_str()) + "' looks like a normal map: set its type, or turned islands get wrong tangents";
    }
    st.packer = std::move(P);
    st.ok = true;
}

// The packed UVs into the target channel of a node's mesh (the mesh the layout was made from).
bool write_uvs(MNMesh& m, int channel, const PackUVs& uv) {
    const CornerTable ct = corners_of(m);
    if (ct.face.size() != uv.tv.size() || uv.verts.empty() || channel < 1) return false;
    if (m.MNum() <= channel) m.SetMapNum(channel + 1);
    MNMap* d = m.M(channel);
    d->ClearAndFree();
    d->ClearFlag(MN_DEAD);
    d->setNumVerts((int)uv.verts.size());
    for (size_t i = 0; i < uv.verts.size(); ++i) d->v[i] = uv.verts[i];
    d->setNumFaces(m.FNum());
    std::vector<int> live(m.FNum(), -1);
    for (size_t i = 0; i < ct.faces.size(); ++i) live[ct.faces[i]] = (int)i;
    for (int f = 0; f < m.FNum(); ++f) {
        const int deg = m.F(f)->deg;
        d->F(f)->SetSize(deg);
        for (int k = 0; k < deg; ++k) d->F(f)->tv[k] = live[f] < 0 ? 0 : uv.tv[ct.first[live[f]] + k];
    }
    return true;
}

struct Busy {
    Modifier* mod;
    bool entered;
    explicit Busy(Modifier* m) : mod(m), entered(packBusy.insert(m).second) {}
    ~Busy() { if (entered) packBusy.erase(mod); }
};

bool pack_modify(Modifier* self, ModContext& mc, INode* node, MNMesh& mesh, const UdimPackSettings& s, std::wstring& status,
                 std::vector<UdimPackTile>& tiles) {
    std::lock_guard guard(packLock);
    Busy busy(self);
    if (!busy.entered) return false;  // evaluating another part came back here: leave this one as it is
    if (!node) node = node_of(self, &mc);
    PackState& st = packs[self];
    const std::vector<PackUse> uses = uses_of(self);
    std::string err;

    // This node's part from its input; the other parts evaluated below the modifier when unknown.
    if (node) {
        PackPart part;
        if (make_part(mesh, s.sourceChannel, part, err)) st.parts[node->GetHandle()] = std::move(part);
        else st.parts.erase(node->GetHandle());
    }
    const TimeValue t = GetCOREInterface()->GetTime();
    for (const PackUse& u : uses) {
        if (u.node == node) continue;
        auto it = st.parts.find(u.node->GetHandle());
        if (it != st.parts.end() && it->second.channel == s.sourceChannel) continue;
        PackPart part;
        std::string ignore;
        if (read_part(u, t, s.sourceChannel, part, ignore)) st.parts[u.node->GetHandle()] = std::move(part);
        else st.parts.erase(u.node->GetHandle());
    }
    for (auto it = st.parts.begin(); it != st.parts.end();) {
        const bool used = std::any_of(uses.begin(), uses.end(), [&](const PackUse& u) { return u.node->GetHandle() == it->first; }) ||
                          (node && node->GetHandle() == it->first);
        it = used ? std::next(it) : st.parts.erase(it);
    }

    const uint64_t signature = layout_signature(st, s);
    const bool changed = signature != st.layout;
    if (changed) {
        make_layout(st, s);
        st.layout = signature;
    }
    tiles = st.tiles;

    std::string report = st.report;
    if (!node) {
        report = "node not found";
    } else if (!err.empty()) {
        report = utf8(node->GetName()) + ": " + err;
    } else if (st.ok) {
        auto it = st.uvs.find(node->GetHandle());
        if (it == st.uvs.end() || !write_uvs(mesh, s.targetChannel, it->second)) report += "\nthe layout does not fit this mesh";
    }
    report += "\n" + std::to_string(st.parts.size()) + " part" + (st.parts.size() == 1 ? "" : "s");
    status = wide(report);
    return changed && uses.size() > 1;
}

bool pack_bake(Modifier* self, const UdimPackSettings& s, UdimSink out, std::wstring& status, bool& relayout) {
    std::lock_guard guard(packLock);
    Busy busy(self);
    relayout = false;
    auto fail = [&](const std::string& why) {
        status = wide(why);
        out.write(out.ctx, why.data(), why.size());
        return false;
    };
    if (!busy.entered) return fail("busy");
    PackState& st = packs[self];

    // Every part read again: a part the viewport has not evaluated since it changed would be stale.
    const TimeValue t = GetCOREInterface()->GetTime();
    st.parts.clear();
    std::string log, err;
    for (const PackUse& u : uses_of(self)) {
        PackPart part;
        if (read_part(u, t, s.sourceChannel, part, err)) st.parts[u.node->GetHandle()] = std::move(part);
        else log += "  ! " + utf8(u.node->GetName()) + ": " + err + "\n";
    }
    const uint64_t signature = layout_signature(st, s);
    if (signature != st.layout || !st.packer) {
        make_layout(st, s);
        st.layout = signature;
        relayout = true;
    }
    log = st.log + log;
    if (!st.ok) return fail(log + st.report);

    std::filesystem::path dir = s.outDir;
    const std::filesystem::path scene = GetCOREInterface()->GetCurFilePath().data();
    if (dir.empty()) {
        if (scene.empty()) return fail("save the scene or choose an output folder");
        dir = scene.parent_path() / L"udimpacker";
    }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (!std::filesystem::is_directory(dir, ec)) return fail("cannot create " + ustr(dir));

    Packer& P = *st.packer;
    P.o.outDir = ustr(dir);
    P.o.prefix = s.prefix.empty() ? "atlas" : utf8(s.prefix.c_str());
    P.scene.sourcePath = ustr(scene);
    P.verifySamples = P.verifyMismatches = 0;
    bool ok;
    {
        SayCapture capture(log);
        say("baking %dx%d into %s\n", P.AW, P.AH, P.o.outDir.c_str());
        std::vector<uint8_t> owned;
        BleedPlan plan;
        std::vector<std::string> files;
        ok = P.compose(owned, plan, files);
        if (ok) {
            P.write_layout(owned);
            P.write_manifest(files);
        }
    }
    out.write(out.ctx, log.data(), log.size());
    if (!ok) return fail(last_line(log));
    char line[512];
    std::snprintf(line, sizeof line, "baked %zu map%s %dx%d, %zu of %zu samples differ\n%s", P.channels.size(), P.channels.size() == 1 ? "" : "s",
                  P.AW, P.AH, P.verifyMismatches, P.verifySamples, P.o.outDir.c_str());
    status = wide(line);
    return P.verifyMismatches == 0;
}

void pack_forget(Modifier* self) {
    std::lock_guard guard(packLock);
    packs.erase(self);
}

const UdimCoreApi kApi{kUdimCoreApiVersion, kBuild, inspect, unsubdivide_modify, decimate_modify, write_weights,
                       pack_modify, pack_bake, pack_forget};

}  // namespace

extern "C" __declspec(dllexport) const UdimCoreApi* udim_core_api() { return &kApi; }
