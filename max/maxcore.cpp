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
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

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

const UdimCoreApi kApi{kUdimCoreApiVersion, kBuild, inspect, unsubdivide_modify, decimate_modify, write_weights};

}  // namespace

extern "C" __declspec(dllexport) const UdimCoreApi* udim_core_api() { return &kApi; }
