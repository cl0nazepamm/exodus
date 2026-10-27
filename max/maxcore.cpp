// Exodus for 3ds Max: the work, in a DLL the plugin loads from a copy and can reload
// (see maxcore_api.h). Everything here runs on Max's main thread, called from the plugin.
#include <max.h>
#include <AssetEnumCallback.h>
#include <AssetManagement/AssetUser.h>
#include <imorpher.h>
#include <iskin.h>
#pragma warning(push)
#pragma warning(disable : 4535)  // the SDK's MAXScript headers call _set_se_translator, which wants /EHa
#include <maxscript/maxscript.h>
#pragma warning(pop)
#include <modstack.h>
#include <polyobj.h>
#include <mnmesh.h>
#include <MNNormalSpec.h>
#include <stdmat.h>
#include <triobj.h>

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
#include <unordered_set>
#include <vector>

#include "atlas.h"
#include "decimate.h"
#include "maxcore_api.h"
#include "subdiv.h"
#include "exodus.h"

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

bool inspect(INode* node, ExodusSink out) {
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
// MNMesh and Exodus's mesh. Corners are numbered over the live faces in order. Every corner
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

// The mesh as Exodus's: corners carry their input corner (cornerSrc) and wedge (cornerSeam).
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
bool finish_plan(MNMesh& m, const CornerTable& ct, const InMesh& r, ExodusPlan& plan) {
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

uint64_t unsubdivide_signature(MNMesh& m, const ExodusUnsubdivide& s) {
    Fnv h = settings_hash('U');
    h.add((uint64_t)s.levels);
    h.add(s.symmetry);
    h.add(s.quadDominant);
    return topology_hash(m, h);
}

bool unsubdivide_plan(MNMesh& m, const ExodusUnsubdivide& s, ExodusPlan& plan) {
    plan = {};
    plan.signature = unsubdivide_signature(m, s);
    const CornerTable ct = corners_of(m);
    InScene scene;
    scene.meshes.push_back(to_inmesh(m, ct, false));
    UnsubdivideOptions opt;
    opt.levels = s.levels;
    opt.symmetry = s.symmetry;
    opt.quadDominant = s.quadDominant;
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

uint64_t decimate_signature(MNMesh& m, const ExodusDecimate& s) {
    Fnv h = settings_hash('D');
    h.add(std::bit_cast<uint32_t>(s.percent));
    h.add(s.lockBorder);
    h.add(s.symmetry);
    h.add(s.lockSmoothing);
    return topology_hash(m, h);
}

bool decimate_plan(MNMesh& m, const ExodusDecimate& s, ExodusPlan& plan) {
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
    opt.symmetry = s.symmetry;
    size_t locked = 0;
    if (s.lockSmoothing) {  // the hard edges of the smoothing groups stay
        std::vector<uint32_t> groups(ct.faces.size());
        for (size_t i = 0; i < ct.faces.size(); ++i) groups[i] = m.F(ct.faces[i])->smGroup;
        opt.lockPos.push_back(group_border_positions(scene.meshes[0], groups));
        locked = (size_t)std::count(opt.lockPos[0].begin(), opt.lockPos[0].end(), 1);
    }
    const DecimateResult r = decimate_scene(scene, std::vector<uint8_t>(maxMaterial + 1, 0), opt);
    if (scene.meshes.empty() || !finish_plan(m, ct, scene.meshes[0], plan)) return false;
    char line[256];
    std::snprintf(line, sizeof line, "%zu -> %zu triangles, error %.3g%%", r.before, r.after, r.error * 100);
    plan.report = line;
    if (r.meshes[0].mirrorAxis >= 0) {
        std::snprintf(line, sizeof line, ", mirror %c %.0f%%", "xyz"[r.meshes[0].mirrorAxis], r.meshes[0].mirrored * 100);
        plan.report += line;
    }
    if (s.lockSmoothing) plan.report += ", " + std::to_string(locked) + " smoothing-border vertices kept";
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
bool apply_plan(MNMesh& m, const ExodusPlan& plan, MNMesh& out) {
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

SkinAbove skin_above(INode* node, Modifier* self, ExodusIsReducer isReducer) {
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
    size_t operator()(const ExodusPos& p) const {
        uint64_t h = 1469598103934665603ull;
        for (float x : p) {
            h ^= std::bit_cast<uint32_t>(x + 0.0f);
            h *= 1099511628211ull;
        }
        return (size_t)h;
    }
};

struct PosIndex {
    std::unordered_map<ExodusPos, uint32_t, PosHash> at;
    explicit PosIndex(const ExodusSkinCopy& c) {
        for (uint32_t i = 0; i < c.pos.size(); ++i) at.try_emplace(c.pos[i], i);
    }
    uint32_t find(const ExodusPos& p) const {
        auto it = at.find(p);
        return it == at.end() ? kNone : it->second;
    }
};

ExodusPos pos_of(MNMesh& m, int v) { return {m.P(v).x, m.P(v).y, m.P(v).z}; }

INode* copy_bone(ExodusSkinCopy& c, uint32_t i) {
    if (!c.bones[i] && !c.boneNames[i].empty()) c.bones[i] = GetCOREInterface()->GetINodeByName(c.boneNames[i].c_str());
    return c.bones[i];
}

uint32_t copy_bone_index(ExodusSkinCopy& c, INode* node) {
    for (uint32_t i = 0; i < c.bones.size(); ++i)
        if (copy_bone(c, i) == node) return i;
    c.bones.push_back(node);
    c.boneNames.push_back(node ? node->GetName() : L"");
    return (uint32_t)c.bones.size() - 1;
}

// Skin's weights of its vertex i against copied vertex v: the same bones, each weight within
// `tolerance` (0 = identical). Skin normalizes what it is given, which can move the last bit.
bool skin_matches(const SkinAbove& s, int i, ExodusSkinCopy& c, uint32_t v, float tolerance) {
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
void copy_skin_vertex(const SkinAbove& s, int i, const ExodusPos& p, ExodusSkinCopy& c) {
    if (c.first.empty()) c.first.push_back(0);
    c.pos.push_back(p);
    for (int k = 0; k < s.ctx->GetNumAssignedBones(i); ++k) {
        c.bone.push_back(copy_bone_index(c, s.skin->GetBone(s.ctx->GetAssignedBone(i, k))));
        c.weight.push_back(s.ctx->GetBoneWeight(i, k));
    }
    c.first.push_back((uint32_t)c.bone.size());
}

// Takes Skin's weights of every input vertex while Skin still has them all.
void capture(const SkinAbove& s, MNMesh& input, ExodusSkinCopy& c) {
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
void fold_back(const SkinAbove& s, ExodusSkinCopy& c) {
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
    ExodusSkinCopy out = c;
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
                   ExodusReduceState& st, ExodusIsReducer isReducer, std::wstring& status, INode*& nodeOut) {
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
    std::vector<ExodusPos> current(mesh.VNum());
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

bool unsubdivide_modify(Modifier* self, ModContext& mc, INode* node, MNMesh& mesh, const ExodusUnsubdivide& s, ExodusReduceState& st,
                        ExodusIsReducer isReducer, std::wstring& status, INode*& nodeOut) {
    return reduce_modify(self, mc, node, mesh, s.levels > 0, unsubdivide_signature(mesh, s),
                         [&](ExodusPlan& p) { unsubdivide_plan(mesh, s, p); }, st, isReducer, status, nodeOut);
}

bool decimate_modify(Modifier* self, ModContext& mc, INode* node, MNMesh& mesh, const ExodusDecimate& s, ExodusReduceState& st,
                     ExodusIsReducer isReducer, std::wstring& status, INode*& nodeOut) {
    return reduce_modify(self, mc, node, mesh, s.percent < 100, decimate_signature(mesh, s),
                         [&](ExodusPlan& p) { decimate_plan(mesh, s, p); }, st, isReducer, status, nodeOut);
}

bool write_weights(INode* node, Modifier* self, ExodusReduceState& st, const std::vector<ExodusSkinCopy*>& below) {
    if (!node || st.skin.empty()) return false;
    node->EvalWorldState(GetCOREInterface()->GetTime());  // Skin remaps to the current vertices
    const SkinAbove skin = skin_above(node, self, nullptr);
    auto* import = skin.mod ? static_cast<ISkinImportData*>(skin.mod->GetInterface(I_SKINIMPORTDATA)) : nullptr;
    if (!skin.ctx || !import || skin.ctx->GetNumPoints() != (int)st.current.size()) return false;
    const PosIndex own(st.skin);
    std::vector<PosIndex> lower;
    for (ExodusSkinCopy* c : below) lower.emplace_back(*c);
    Tab<INode*> bones;
    Tab<float> weights;
    int written = 0;
    for (size_t i = 0; i < st.current.size(); ++i) {
        ExodusSkinCopy* c = &st.skin;
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
// maps come per UDIM tile from the settings, for all objects or for one object by name: every
// tile with maps becomes one material of the packer's scene, an object's own maps one of their
// own, faces in other tiles are left out. Materials in the scene are not read.
// Keep layout: the target channel already is the layout (separate objects packed by hand), so
// nothing is packed or written to the mesh; Bake resamples each face's maps into that layout.

// One node's input: the faces and source UVs the layout is made from.
struct PackPart {
    uint64_t signature = 0;  // source channel, face degrees and triangles, map faces and values
    int channel = 0;
    InMesh mesh;             // faces (materials set when the scene is put together), source UVs, triangles
    std::vector<UVVert> uvw; // the source map's values, w included
    bool collapsed = false;  // read from the base object of an object collapsed with the modifier
    int target = 0;          // collapsed: the target channel read into `baked`
    std::vector<UVVert> baked;  // collapsed: its target channel per corner of the live faces (the UVs it was packed with)
    int layoutChannel = 0;      // keep layout: the channel read into `layout`, else 0
    std::vector<Vec2d> layout;  // keep layout: that channel per corner of the live faces
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
    std::vector<ExodusPackTile> tiles;
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

// The channel a part's layout is read from: the target channel when the layout is kept, else none.
int layout_channel(const ExodusPackSettings& s) { return s.keepLayout ? s.targetChannel : 0; }

bool make_part(MNMesh& m, int channel, int layoutChannel, PackPart& p, std::string& err) {
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
    if (layoutChannel) {
        MNMap* lm = layoutChannel >= 1 && layoutChannel < m.MNum() ? m.M(layoutChannel) : nullptr;
        if (!lm || lm->GetFlag(MN_DEAD) || lm->FNum() != m.FNum() || lm->VNum() == 0) {
            err = "no UVs in map channel " + std::to_string(layoutChannel) + " to keep";
            return false;
        }
        p.layoutChannel = layoutChannel;
        h.add((uint64_t)layoutChannel << 32);
        for (int f : ct.faces) {
            const MNMapFace* mf = lm->F(f);
            if (mf->deg != m.F(f)->deg) {
                err = "map channel " + std::to_string(layoutChannel) + " does not match the faces";
                return false;
            }
            for (int k = 0; k < mf->deg; ++k) {
                if (mf->tv[k] < 0 || mf->tv[k] >= lm->VNum()) {
                    err = "map channel " + std::to_string(layoutChannel) + " has broken faces";
                    return false;
                }
                const UVVert& uv = lm->v[mf->tv[k]];
                p.layout.push_back({uv.x, uv.y});
                h.add(std::bit_cast<uint32_t>(uv.x));
                h.add(std::bit_cast<uint32_t>(uv.y));
            }
        }
    }
    p.signature = h.h;
    return true;
}

// A node's input, evaluated below the modifier.
bool read_part(const PackUse& u, TimeValue t, const ExodusPackSettings& s, PackPart& p, std::string& err) {
    ObjectState os = u.index + 1 < u.owner->NumModifiers() ? u.owner->Eval(t, u.index + 1) : u.owner->GetObjRef()->Eval(t);
    if (!os.obj || !os.obj->CanConvertToType(polyObjectClassID)) {
        err = "not a mesh";
        return false;
    }
    auto* po = static_cast<PolyObject*>(os.obj->ConvertToType(t, polyObjectClassID));
    const bool ok = make_part(po->GetMesh(), s.sourceChannel, layout_channel(s), p, err);
    if (po != os.obj) po->MaybeAutoDelete();
    return ok;
}

// A collapsed object's input: its base object, which is the mesh as it was at the modifier, with
// the packed UVs it got in the target channel.
bool read_collapsed(INode* node, TimeValue t, const ExodusPackSettings& s, PackPart& p, std::string& err) {
    Object* base = node->GetObjectRef() ? node->GetObjectRef()->FindBaseObject() : nullptr;
    ObjectState os = base ? base->Eval(t) : ObjectState();
    if (!os.obj || !os.obj->CanConvertToType(polyObjectClassID)) {
        err = "not a mesh";
        return false;
    }
    auto* po = static_cast<PolyObject*>(os.obj->ConvertToType(t, polyObjectClassID));
    MNMesh& m = po->GetMesh();
    const bool ok = make_part(m, s.sourceChannel, layout_channel(s), p, err);
    if (ok) {
        p.collapsed = true;
        p.target = s.targetChannel;
        MNMap* map = s.targetChannel >= 1 && s.targetChannel < m.MNum() ? m.M(s.targetChannel) : nullptr;
        if (map && !map->GetFlag(MN_DEAD) && map->FNum() == m.FNum()) {
            const CornerTable ct = corners_of(m);
            for (int f : ct.faces)
                for (int k = 0; k < map->F(f)->deg; ++k) {
                    const int tv = map->F(f)->tv[k];
                    p.baked.push_back(tv >= 0 && tv < map->VNum() ? map->v[tv] : UVVert(0, 0, 0));
                }
        }
    }
    if (po != os.obj) po->MaybeAutoDelete();
    return ok;
}

// The collapsed objects (by name; the ones back on the modifier after an undo are uses again)
// into the parts, read when not read yet. Returns their handles.
std::set<ULONG> read_collapsed_parts(PackState& st, const std::vector<PackUse>& uses, const ExodusPackSettings& s, TimeValue t, std::string& log) {
    std::set<ULONG> kept;
    for (const std::wstring& name : s.collapsed) {
        INode* n = GetCOREInterface()->GetINodeByName(name.c_str());
        if (!n || std::any_of(uses.begin(), uses.end(), [&](const PackUse& u) { return u.node == n; })) continue;
        kept.insert(n->GetHandle());
        auto it = st.parts.find(n->GetHandle());
        if (it != st.parts.end() && it->second.collapsed && it->second.channel == s.sourceChannel && it->second.target == s.targetChannel &&
            it->second.layoutChannel == layout_channel(s))
            continue;
        PackPart part;
        std::string err;
        if (read_collapsed(n, t, s, part, err)) {
            st.parts[n->GetHandle()] = std::move(part);
        } else {
            st.parts.erase(n->GetHandle());
            log += "  ! " + utf8(n->GetName()) + " (collapsed): " + err + "\n";
        }
    }
    return kept;
}

std::string objects_line(const PackState& st) {
    const size_t collapsed = (size_t)std::count_if(st.parts.begin(), st.parts.end(), [](const auto& p) { return p.second.collapsed; });
    std::string line = std::to_string(st.parts.size()) + " object" + (st.parts.size() == 1 ? "" : "s");
    if (collapsed) line += " (" + std::to_string(collapsed) + " collapsed)";
    return line;
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

std::wstring node_name(ULONG handle) {
    INode* n = GetCOREInterface()->GetINodeByHandle(handle);
    return n && n->GetName() ? std::wstring(n->GetName()) : std::wstring();
}

uint64_t layout_signature(const PackState& st, const ExodusPackSettings& s) {
    Fnv h = settings_hash('P');
    for (const auto& [handle, part] : st.parts) {
        h.add(handle);
        h.add(part.signature);
        add_text(h, node_name(handle));  // maps are found by name
    }
    h.add((uint64_t)s.size);
    h.add((uint64_t)s.maxSize);
    h.add((uint64_t)s.padding);
    h.add(s.rotate);
    h.add(s.keepLayout);
    for (int c = 0; c < kExodusPackChannels; ++c) {
        add_text(h, s.channel[c]);
        h.add((uint64_t)s.kind[c]);
    }
    for (const ExodusPackMap& m : s.maps) {
        add_text(h, m.object);
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

// Tiles in use per object, from the parts' source UVs (the rollout adds the tiles with maps).
std::vector<ExodusPackTile> pack_tiles(const PackState& st) {
    std::map<std::pair<std::wstring, int>, int> faces;
    for (const auto& [handle, part] : st.parts) {
        const std::wstring name = node_name(handle);
        for (const InFace& f : part.mesh.faces)
            if (const int udim = face_udim(part.mesh, f); udim > 0) ++faces[{name, udim}];
    }
    std::vector<ExodusPackTile> tiles;
    for (const auto& [key, n] : faces) tiles.push_back({key.first, key.second, n});
    return tiles;
}

std::string last_line(const std::string& log) {
    std::string s = log;
    while (!s.empty() && (s.back() == '\n' || s.back() == ' ')) s.pop_back();
    const size_t nl = s.rfind('\n');
    s = nl == std::string::npos ? s : s.substr(nl + 1);
    return s.substr(s.find_first_not_of(' ') == std::string::npos ? 0 : s.find_first_not_of(' '));
}

void make_layout(PackState& st, const ExodusPackSettings& s) {
    st.ok = false;
    st.packer.reset();
    st.uvs.clear();
    st.log.clear();
    st.tiles = pack_tiles(st);
    if (st.parts.empty()) {
        st.report = "no mesh with UVs";
        return;
    }

    auto P = std::make_unique<Packer>();
    PackOptions& o = P->o;
    o.sizeW = o.sizeH = s.size;
    o.fill = s.size > 0 && !s.keepLayout;  // a size is filled: the islands scaled to it (0: their own size, smallest atlas)
    o.maxSize = s.maxSize;
    o.padding = s.padding;
    o.rotate = s.rotate;
    o.obj = false;
    InScene& scene = P->scene;
    // Material 0 holds the faces of tiles without maps, left out; then one per tile with maps for
    // all objects and one per object and tile with maps of its own.
    scene.materials.push_back({"<no maps>"});
    o.excludes.push_back("<no maps>");
    std::map<std::pair<std::wstring, int>, uint32_t> tileMaterial;  // (object, empty = all; tile) -> material
    for (const ExodusPackMap& m : s.maps) {
        if (m.channel < 0 || m.channel >= kExodusPackChannels || s.channel[m.channel].empty() || m.file.empty()) continue;
        auto [it, fresh] = tileMaterial.try_emplace({m.object, m.tile}, (uint32_t)scene.materials.size());
        if (fresh) scene.materials.push_back({(m.object.empty() ? "" : utf8(m.object.c_str()) + " ") + std::to_string(m.tile)});
        InTexture tex;
        tex.channel = utf8(s.channel[m.channel].c_str());
        tex.path = utf8(m.file.c_str());
        tex.label = ustr(upath(tex.path).filename());
        std::vector<InTexture>& textures = scene.materials[it->second].textures;
        auto same = std::find_if(textures.begin(), textures.end(), [&](const InTexture& x) { return x.channel == tex.channel; });
        if (same != textures.end()) *same = tex;
        else textures.push_back(tex);
    }
    for (int c = 0; c < kExodusPackChannels; ++c)
        if (!s.channel[c].empty()) o.kinds[utf8(s.channel[c].c_str())] = s.kind[c] == 1 ? "normal-gl" : s.kind[c] == 2 ? "normal-dx" : "color";
    if (tileMaterial.empty()) {
        st.report = "assign maps to the tiles to pack";
        return;
    }

    // A face takes its object's own maps in its tile, else the maps for all objects there.
    std::vector<ULONG> order;
    std::set<uint32_t> used;
    std::map<std::wstring, int> named;  // objects per name
    for (const auto& [handle, part] : st.parts) {
        InMesh mesh = part.mesh;
        const std::wstring name = node_name(handle);
        mesh.name = utf8(name.c_str());
        ++named[name];
        for (InFace& f : mesh.faces) {
            const int udim = face_udim(mesh, f);
            auto it = tileMaterial.find({name, udim});
            if (it == tileMaterial.end()) it = tileMaterial.find({std::wstring(), udim});
            f.material = it == tileMaterial.end() ? 0 : it->second;
            if (f.material) used.insert(f.material);
        }
        scene.meshes.push_back(std::move(mesh));
        if (s.keepLayout) P->layoutUv.push_back(part.layout);
        order.push_back(handle);
    }

    SayCapture capture(st.log);
    if (!P->bind()) {
        st.report = last_line(st.log);
        return;
    }
    char line[256];
    if (s.keepLayout) {
        // The target channel is the layout: nothing to pack, and the mesh keeps its UVs.
        if (!P->use_layout()) {
            st.report = last_line(st.log);
            return;
        }
        std::snprintf(line, sizeof line, "%zu tile%s, layout of channel %d kept, %dx%d atlas at %.0f%% density, %zu map%s", used.size(),
                      used.size() == 1 ? "" : "s", s.targetChannel, P->AW, P->AH, P->layoutDensity * 100, P->channels.size(),
                      P->channels.size() == 1 ? "" : "s");
        st.report = line;
        if (s.sourceChannel == s.targetChannel) st.report += "\nsource and target are one channel: the maps' own UVs are the layout";
    } else {
        P->build_islands();
        if (!(o.fill ? P->pack_fill() : P->pack())) {
            st.report = last_line(st.log);
            return;
        }
        for (size_t mi = 0; mi < order.size(); ++mi) st.uvs[order[mi]] = packed_uvs(*P, mi, st.parts[order[mi]]);
        std::snprintf(line, sizeof line, "%zu tile%s, %zu islands, %dx%d atlas, %zu map%s", used.size(), used.size() == 1 ? "" : "s",
                      P->islands.size(), P->AW, P->AH, P->channels.size(), P->channels.size() == 1 ? "" : "s");
        st.report = line;
    }
    if (o.fill) {
        std::snprintf(line, sizeof line, ", filled at %.0f%% of the maps' size", P->fillScale * 100);
        st.report += line;
    }
    // The first warning, if any: unreadable maps, maps of one tile at unrelated sizes, ...
    if (const size_t w = st.log.find("  ! "); w != std::string::npos) st.report += "\n" + st.log.substr(w + 4, st.log.find('\n', w) - w - 4);
    for (const auto& [name, n] : named)
        if (n > 1 && !name.empty() && std::any_of(s.maps.begin(), s.maps.end(), [&](const ExodusPackMap& m) { return m.object == name; }))
            st.report += "\n" + std::to_string(n) + " objects are named '" + utf8(name.c_str()) + "': they share its maps";
    // A collapsed object keeps the UVs it was packed with: if this layout gives it others (settings
    // or maps changed since), it no longer matches the atlas.
    for (const auto& [handle, part] : st.parts) {
        if (!part.collapsed || s.keepLayout) continue;  // a kept layout is read from the collapsed mesh itself
        const PackUVs& uv = st.uvs[handle];
        bool same = uv.tv.size() == part.baked.size();
        for (size_t c = 0; same && c < uv.tv.size(); ++c)
            same = std::abs(uv.verts[uv.tv[c]].x - part.baked[c].x) < 1e-6f && std::abs(uv.verts[uv.tv[c]].y - part.baked[c].y) < 1e-6f;
        if (!same) st.report += "\n" + utf8(node_name(handle).c_str()) + " was collapsed with another layout: its UVs do not match this one";
    }
    for (int c = 0; c < kExodusPackChannels; ++c) {
        std::wstring low = s.channel[c];
        for (wchar_t& ch : low) ch = (wchar_t)towlower(ch);
        if ((s.rotate || s.keepLayout) && s.kind[c] == 0 && low.find(L"normal") != std::wstring::npos)
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

bool pack_modify(Modifier* self, ModContext& mc, INode* node, MNMesh& mesh, const ExodusPackSettings& s, std::wstring& status,
                 std::vector<ExodusPackTile>& tiles) {
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
        if (make_part(mesh, s.sourceChannel, layout_channel(s), part, err)) st.parts[node->GetHandle()] = std::move(part);
        else st.parts.erase(node->GetHandle());
    }
    const TimeValue t = GetCOREInterface()->GetTime();
    for (const PackUse& u : uses) {
        if (u.node == node) continue;
        auto it = st.parts.find(u.node->GetHandle());
        if (it != st.parts.end() && it->second.channel == s.sourceChannel && it->second.layoutChannel == layout_channel(s) &&
            !it->second.collapsed)
            continue;
        PackPart part;
        std::string ignore;
        if (read_part(u, t, s, part, ignore)) st.parts[u.node->GetHandle()] = std::move(part);
        else st.parts.erase(u.node->GetHandle());
    }
    std::string ignore;
    const std::set<ULONG> kept = read_collapsed_parts(st, uses, s, t, ignore);
    for (auto it = st.parts.begin(); it != st.parts.end();) {
        const bool used = std::any_of(uses.begin(), uses.end(), [&](const PackUse& u) { return u.node->GetHandle() == it->first; }) ||
                          (node && node->GetHandle() == it->first) || kept.count(it->first);
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
    } else if (st.ok && !s.keepLayout) {
        auto it = st.uvs.find(node->GetHandle());
        if (it == st.uvs.end() || !write_uvs(mesh, s.targetChannel, it->second)) report += "\nthe layout does not fit this mesh";
    }
    report += "\n" + objects_line(st);
    // Collapse To this modifier bakes everything below it into the mesh: say so for Skin and Morpher.
    for (const PackUse& u : uses) {
        if (u.node != node) continue;
        std::string below;
        for (int i = u.index + 1; i < u.owner->NumModifiers(); ++i) {
            const Class_ID id = u.owner->GetModifier(i)->ClassID();
            const char* name = id == Class_ID(9815843, 87654) ? "Skin" : id == Class_ID(0x17bb6854, 0xa5cba2a3) ? "Morpher" : nullptr;  // iskin.h, imorpher.h
            if (name) below += (below.empty() ? "" : ", ") + std::string(name);
        }
        if (!below.empty()) report += "\n" + below + " below: Collapse To here would bake it (move UDIM Pack under it)";
    }
    status = wide(report);
    return changed && uses.size() > 1;
}

// Bake and Preview: every part read again (a part the viewport has not evaluated since it changed
// would be stale) and laid out again when that changed the layout. Returns the packer's report
// with the parts that could not be read.
std::string reread_layout(Modifier* self, PackState& st, const ExodusPackSettings& s, bool& relayout) {
    const TimeValue t = GetCOREInterface()->GetTime();
    st.parts.clear();
    std::string log, err;
    const std::vector<PackUse> uses = uses_of(self);
    for (const PackUse& u : uses) {
        PackPart part;
        if (read_part(u, t, s, part, err)) st.parts[u.node->GetHandle()] = std::move(part);
        else log += "  ! " + utf8(u.node->GetName()) + ": " + err + "\n";
    }
    read_collapsed_parts(st, uses, s, t, log);
    const uint64_t signature = layout_signature(st, s);
    if (signature != st.layout || !st.packer) {
        make_layout(st, s);
        st.layout = signature;
        relayout = true;
    }
    return st.log + log;
}

bool pack_bake(Modifier* self, const ExodusPackSettings& s, ExodusSink out, std::wstring& status, bool& relayout) {
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
    std::string log = reread_layout(self, st, s, relayout);
    if (!st.ok) return fail(log + st.report);

    std::filesystem::path dir = s.outDir;
    const std::filesystem::path scene = GetCOREInterface()->GetCurFilePath().data();
    if (dir.empty()) {
        if (scene.empty()) return fail("save the scene or choose an output folder");
        dir = scene.parent_path() / L"exodus";
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
    if (P.existing_layout())  // resampled, so there is no texel-exact check
        std::snprintf(line, sizeof line, "baked %zu map%s %dx%d into the kept layout\n%s", P.channels.size(), P.channels.size() == 1 ? "" : "s",
                      P.AW, P.AH, P.o.outDir.c_str());
    else
        std::snprintf(line, sizeof line, "baked %zu map%s %dx%d, %zu of %zu samples differ\n%s", P.channels.size(),
                      P.channels.size() == 1 ? "" : "s", P.AW, P.AH, P.verifyMismatches, P.verifySamples, P.o.outDir.c_str());
    std::string text = line;
    for (size_t at = st.report.find(" was collapsed with another layout"); at != std::string::npos;
         at = st.report.find(" was collapsed with another layout", at + 1)) {
        const size_t from = st.report.rfind('\n', at) + 1;
        text += "\n" + st.report.substr(from, st.report.find('\n', at) - from);
    }
    status = wide(text);
    return P.verifyMismatches == 0;
}

// Bake's dry run: the layout Bake would write, as the packed faces' edges per object; nothing is
// written. An edge shared by two faces of an island is given once.
bool pack_preview(Modifier* self, const ExodusPackSettings& s, ExodusSink out, std::wstring& status, bool& relayout,
                  ExodusPackPreview& preview) {
    std::lock_guard guard(packLock);
    Busy busy(self);
    relayout = false;
    preview = {};
    if (!busy.entered) {
        status = preview.report = L"busy";
        return false;
    }
    PackState& st = packs[self];
    const std::string log = reread_layout(self, st, s, relayout);
    out.write(out.ctx, log.data(), log.size());
    status = wide(st.report + "\n" + objects_line(st));
    preview.report = wide(st.report);
    if (!st.ok) return false;

    const Packer& P = *st.packer;
    const bool kept = P.existing_layout();
    preview.width = P.AW;
    preview.height = P.AH;
    for (size_t mi = 0; mi < P.scene.meshes.size(); ++mi) {
        const InMesh& mesh = P.scene.meshes[mi];
        // (source UV, island) -> packed UV, as packed_uvs makes them; a kept layout's UVs by value
        std::unordered_map<uint64_t, uint32_t> index;
        std::vector<Vec2d> uv;
        std::unordered_set<uint64_t> seen;  // edges by their packed UVs
        const size_t first = preview.edges.size();
        for (size_t fi = 0; fi < mesh.faces.size(); ++fi) {
            const int isl = kept ? -1 : P.faceIsland[mi][fi];
            if (P.faceSource[mi][fi] < 0 || (!kept && isl < 0)) continue;
            const InFace& f = mesh.faces[fi];
            auto packed = [&](uint32_t k) {
                const uint32_t corner = f.first + k % f.count, uvi = mesh.cornerUv[corner];
                const Vec2d q = kept ? P.layoutUv[mi][corner] : Vec2d{};
                const uint64_t key = kept ? ((uint64_t)std::bit_cast<uint32_t>((float)q.x) << 32) | std::bit_cast<uint32_t>((float)q.y)
                                          : ((uint64_t)uvi << 32) | (uint32_t)isl;
                auto [it, fresh] = index.try_emplace(key, (uint32_t)uv.size());
                if (fresh) uv.push_back(kept ? q : P.new_uv(P.islands[isl], mesh.uvs[uvi], P.faceTile[mi][fi]));
                return it->second;
            };
            for (uint32_t k = 0; k < f.count; ++k) {
                const uint32_t a = packed(k), b = packed(k + 1);
                if (a == b || !seen.insert(((uint64_t)std::min(a, b) << 32) | std::max(a, b)).second) continue;
                preview.edges.push_back({(float)uv[a].x, (float)uv[a].y, (float)uv[b].x, (float)uv[b].y});
            }
        }
        if (preview.edges.size() == first) continue;  // nothing of this object is packed
        preview.objects.push_back(wide(mesh.name));
        preview.objectFirst.push_back((uint32_t)first);
    }
    preview.objectFirst.push_back((uint32_t)preview.edges.size());
    return true;
}

// ---- Auto detect: the maps from the objects' materials ------------------------------------------

enum DetectRole { kRoleColor, kRoleNormal, kRoleRoughness, kRoleMetallic, kRoleOpacity, kRoleEmission, kRoleOcclusion, kRoles };
const wchar_t* const kRoleName[kRoles] = {L"BaseColor", L"Normal", L"Roughness", L"Metallic", L"Opacity", L"Emission", L"Occlusion"};

std::wstring lower(std::wstring s) {
    for (wchar_t& c : s) c = (wchar_t)towlower(c);
    return s;
}

bool has(const std::wstring& s, const wchar_t* key) { return s.find(key) != std::wstring::npos; }

// The Normal Bump under a texmap, if any.
Texmap* normal_bump(Texmap* tm, int depth = 0) {
    if (!tm || depth > 8) return nullptr;
    if (tm->ClassID() == GNORMAL_CLASS_ID) return tm;
    for (int i = 0; i < tm->NumSubTexmaps(); ++i)
        if (Texmap* nb = normal_bump(tm->GetSubTexmap(i), depth + 1)) return nb;
    return nullptr;
}

// The role of a material's map slot, by its name (Physical, OpenPBR, USD Preview Surface,
// Standard and the like); -1 for slots that are not packed (coat, specular, displacement, ...).
int slot_role(const std::wstring& slot, Texmap* tm) {
    const std::wstring s = lower(slot);
    if (has(s, L"coat")) return -1;
    if (has(s, L"normal") || (has(s, L"bump") && normal_bump(tm))) return kRoleNormal;
    if (has(s, L"occlusion")) return kRoleOcclusion;
    if (has(s, L"emissi")) return has(s, L"color") ? kRoleEmission : -1;
    if (has(s, L"opacity") || has(s, L"cutout")) return kRoleOpacity;
    if (has(s, L"metal")) return kRoleMetallic;
    if (has(s, L"rough"))
        return has(s, L"diffuse") || has(s, L"anisotropy") || has(s, L"transmission") || has(s, L"transparency") ? -1 : kRoleRoughness;
    if (has(s, L"base color") || has(s, L"basecolor") || has(s, L"base_color") || has(s, L"albedo") || has(s, L"diffuse")) return kRoleColor;
    return -1;
}

// The role a channel of the modifier is for, by its name; -1 for none (or no name).
int channel_role(const std::wstring& name) {
    const std::wstring s = lower(name);
    if (s.empty()) return -1;
    if (has(s, L"normal")) return kRoleNormal;
    if (has(s, L"occlusion") || s == L"ao") return kRoleOcclusion;
    if (has(s, L"emiss") || has(s, L"emit")) return kRoleEmission;
    if (has(s, L"opac") || has(s, L"alpha") || has(s, L"cutout")) return kRoleOpacity;
    if (has(s, L"metal")) return kRoleMetallic;
    if (has(s, L"rough")) return kRoleRoughness;
    if (has(s, L"color") || has(s, L"colour") || has(s, L"diffuse") || has(s, L"albedo") || has(s, L"base")) return kRoleColor;
    return -1;
}

// The image files under a texmap, as Max's asset tracking finds them (any map type: Bitmap, OSL
// bitmaps, ...), full paths where they resolve.
std::vector<std::wstring> image_files(Texmap* tm) {
    struct Files : AssetEnumCallback {
        std::vector<std::wstring> list;
        void RecordAsset(const MaxSDK::AssetManagement::AssetUser& a) override {
            if (a.GetType() != MaxSDK::AssetManagement::kBitmapAsset) return;
            MSTR full;
            const std::wstring f = a.GetFullFilePath(full) ? full.data() : a.GetFileName().data();
            if (!f.empty() && std::find(list.begin(), list.end(), f) == list.end()) list.push_back(f);
        }
    } files;
    if (tm) tm->EnumAuxFiles(files, FILE_ENUM_ALL);
    return files.list;
}

bool readable_image(const std::wstring& file) {
    const std::wstring ext = lower(std::filesystem::path(file).extension().wstring());
    return ext == L".png" || ext == L".jpg" || ext == L".jpeg" || ext == L".tga" || ext == L".bmp" || ext == L".psd";
}

// A material's packed maps: the file per role (empty: none), and notes on what was left.
struct MaterialMaps {
    std::wstring file[kRoles];
    std::vector<std::string> notes;
};

MaterialMaps material_maps(Mtl* m) {
    MaterialMaps out;
    if (!m) return out;
    bool any = false;
    for (int i = 0; i < m->NumSubTexmaps(); ++i) {
        Texmap* tm = m->GetSubTexmap(i);
        if (!tm || !m->SubTexmapOn(i)) continue;
        const std::wstring slot = m->GetSubTexmapSlotName(i, false).data();
        const int role = slot_role(slot, tm);
        if (role < 0 || !out.file[role].empty()) continue;
        Texmap* nb = role == kRoleNormal ? normal_bump(tm) : nullptr;
        std::vector<std::wstring> files = image_files(nb && nb->GetSubTexmap(0) ? nb->GetSubTexmap(0) : tm);
        std::erase_if(files, [&](const std::wstring& f) {
            if (readable_image(f)) return false;
            out.notes.push_back(utf8(slot.c_str()) + ": " + utf8(std::filesystem::path(f).filename().wstring().c_str()) +
                                " is not png, jpg, tga, bmp or psd");
            return true;
        });
        if (files.empty()) continue;
        if (files.size() > 1) out.notes.push_back(utf8(slot.c_str()) + ": " + std::to_string(files.size()) + " maps, took the first");
        out.file[role] = files.front();
        any = true;
    }
    // Shell and similar materials keep the maps in a sub-material.
    if (!any && !m->IsMultiMtl())
        for (int i = 0; i < m->NumSubMtls() && !any; ++i)
            if (Mtl* sub = m->GetSubMtl(i)) {
                MaterialMaps s = material_maps(sub);
                for (int r = 0; r < kRoles; ++r) any |= !s.file[r].empty();
                if (any) out = std::move(s);
            }
    return out;
}

// The sub-material faces of material ID `id` (0-based) show; materials that are not multi show
// themselves. Multi/Sub-Object's GetSubMtl takes an ID of its ID list (stored 0-based) and IDs above
// the largest wrap around (samples/materials/multi.cpp, FindSubMtlMod); other multi materials take
// the ID modulo their count.
Mtl* face_material(Mtl* m, int id) {
    if (!m || !m->IsMultiMtl() || m->NumSubMtls() == 0) return m;
    if (m->ClassID() == Class_ID(MULTI_CLASS_ID, 0))
        if (IParamBlock2* pb = m->GetParamBlock(0)) {
            const ParamID list = pb->IndextoID(pb->GetDesc()->NameToIndex(_M("materialIDList")));
            if (list >= 0 && pb->Count(list) > 0) {
                int top = 0;
                for (int k = 0; k < pb->Count(list); ++k) top = std::max(top, pb->GetInt(list, 0, k));
                return m->GetSubMtl(id > top ? id % (top + 1) : id);
            }
        }
    return m->GetSubMtl(id % m->NumSubMtls());
}

bool pack_detect(Modifier* self, ExodusPackSettings& s, std::wstring& report) {
    std::lock_guard guard(packLock);
    Busy busy(self);
    if (!busy.entered) {
        report = L"busy";
        return false;
    }
    const TimeValue t = GetCOREInterface()->GetTime();
    // The objects: where the modifier sits (its input mesh), and the collapsed ones (their base object).
    struct Target {
        INode* node;
        PackUse use;            // owner null: collapsed
        ObjectState eval(TimeValue t) const {
            if (use.owner) return use.index + 1 < use.owner->NumModifiers() ? use.owner->Eval(t, use.index + 1) : use.owner->GetObjRef()->Eval(t);
            Object* base = node->GetObjectRef() ? node->GetObjectRef()->FindBaseObject() : nullptr;
            return base ? base->Eval(t) : ObjectState();
        }
    };
    std::vector<Target> targets;
    const std::vector<PackUse> uses = uses_of(self);
    for (const PackUse& u : uses) targets.push_back({u.node, u});
    for (const std::wstring& name : s.collapsed) {
        INode* n = GetCOREInterface()->GetINodeByName(name.c_str());
        if (!n || std::any_of(uses.begin(), uses.end(), [&](const PackUse& u) { return u.node == n; })) continue;
        targets.push_back({n, {n, nullptr, 0}});
    }

    std::string log;
    std::vector<ExodusPackMap> found;  // the rows of the objects with maps
    std::set<std::wstring> detected;
    bool roles[kRoles] = {};
    std::map<Mtl*, MaterialMaps> cache;
    for (const Target& tg : targets) {
        const std::wstring name = tg.node->GetName() ? tg.node->GetName() : L"";
        const std::string uname = utf8(name.c_str());
        Mtl* mtl = tg.node->GetMtl();
        if (!mtl) {
            log += "  ! " + uname + ": no material\n";
            continue;
        }
        // Faces per (tile, material ID), tiles from the source UVs as the packer finds them.
        std::map<std::pair<int, int>, int> faces;
        std::string err;
        const ObjectState os = tg.eval(t);
        if (!os.obj || !os.obj->CanConvertToType(polyObjectClassID)) {
            err = "not a mesh";
        } else {
            auto* po = static_cast<PolyObject*>(os.obj->ConvertToType(t, polyObjectClassID));
            MNMesh& m = po->GetMesh();
            MNMap* map = s.sourceChannel >= 1 && s.sourceChannel < m.MNum() ? m.M(s.sourceChannel) : nullptr;
            if (!map || map->GetFlag(MN_DEAD) || map->FNum() != m.FNum()) err = "no UVs in map channel " + std::to_string(s.sourceChannel);
            else
                for (int f = 0; f < m.FNum(); ++f) {
                    const MNFace* F = m.F(f);
                    if (F->GetFlag(MN_DEAD) || map->F(f)->deg != F->deg) continue;
                    double cu = 0, cv = 0;
                    for (int k = 0; k < F->deg; ++k) {
                        const int tv = map->F(f)->tv[k];
                        if (tv >= 0 && tv < map->VNum()) cu += map->v[tv].x, cv += map->v[tv].y;
                    }
                    const Tile tile{(int)std::floor(cu / F->deg), (int)std::floor(cv / F->deg)};
                    if (tile.u >= 0 && tile.u < 10 && tile.v >= 0 && tile.v < 100) ++faces[{udim_number(tile), (int)F->material}];
                }
            if (po != os.obj) po->MaybeAutoDelete();
        }
        if (!err.empty()) {
            log += "  ! " + uname + ": " + err + "\n";
            continue;
        }
        // Per tile and role, the file on the most faces (sub-materials of one tile may differ).
        std::map<std::pair<int, int>, std::map<std::wstring, int>> votes;  // (tile, role) -> file -> faces
        std::map<int, int> bare;  // per tile: faces whose (sub-)material has no maps
        for (const auto& [key, n] : faces) {
            Mtl* fm = face_material(mtl, key.second);
            auto it = cache.find(fm);
            if (it == cache.end()) {
                it = cache.emplace(fm, material_maps(fm)).first;
                for (const std::string& note : it->second.notes) log += "  ! " + uname + ", " + utf8(fm->GetName()) + ": " + note + "\n";
            }
            bool any = false;
            for (int r = 0; r < kRoles; ++r)
                if (!it->second.file[r].empty()) votes[{key.first, r}][it->second.file[r]] += n, any = true;
            if (!any) bare[key.first] += n;
        }
        if (votes.empty()) {
            log += "  ! " + uname + ": no maps in its material\n";
            continue;
        }
        // A tile has one set of maps per object: faces there without maps get the others' maps.
        for (const auto& [tile, n] : bare)
            if (std::any_of(votes.begin(), votes.end(), [&](const auto& v) { return v.first.first == tile; }))
                log += "  ! " + uname + ", tile " + std::to_string(tile) + ": " + std::to_string(n) +
                       " faces show a material without maps; they get the maps of the other faces\n";
        std::string line = "  " + uname + ":";
        for (const auto& [key, files] : votes) {
            auto best = std::max_element(files.begin(), files.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
            found.push_back({name, key.first, key.second, best->first});  // channel: the role for now
            roles[key.second] = true;
            line += " " + std::to_string(key.first) + " " + utf8(kRoleName[key.second]) + " " +
                    utf8(std::filesystem::path(best->first).filename().wstring().c_str());
            if (files.size() > 1)
                log += "  ! " + uname + ", tile " + std::to_string(key.first) + ": " + std::to_string(files.size()) + " " +
                       utf8(kRoleName[key.second]) + " maps (sub-materials), took the one on the most faces\n";
        }
        log += line + "\n";
        detected.insert(name);
    }

    // Roles into channels: the channel named for it, else a free one named now.
    int channelOf[kRoles];
    std::string roleList;
    for (int r = 0; r < kRoles; ++r) {
        channelOf[r] = -1;
        if (!roles[r]) continue;
        for (int c = 0; c < kExodusPackChannels && channelOf[r] < 0; ++c)
            if (channel_role(s.channel[c]) == r) channelOf[r] = c;
        for (int c = 0; c < kExodusPackChannels && channelOf[r] < 0; ++c)
            if (s.channel[c].empty()) {
                s.channel[c] = kRoleName[r];
                s.kind[c] = r == kRoleNormal ? 1 : 0;
                channelOf[r] = c;
                if (r == kRoleNormal) log += "  ! channel Normal added as Normal GL: set Normal DX if the maps are DirectX\n";
            }
        if (channelOf[r] < 0) log += "  ! no free channel for " + utf8(kRoleName[r]) + "\n";
        else roleList += (roleList.empty() ? "" : ", ") + utf8(s.channel[channelOf[r]].c_str());
    }
    // The detected objects' rows replace their old ones; other rows stay.
    std::erase_if(s.maps, [&](const ExodusPackMap& m) { return detected.count(m.object) > 0; });
    for (ExodusPackMap& m : found)
        if ((m.channel = channelOf[m.channel]) >= 0) s.maps.push_back(m);

    char head[256];
    std::snprintf(head, sizeof head, "auto detect: maps for %zu of %zu objects%s%s\n", detected.size(), targets.size(),
                  roleList.empty() ? "" : " (", roleList.empty() ? "" : (roleList + ")").c_str());
    report = wide(head + log);
    return !detected.empty();
}

void pack_forget(Modifier* self) {
    std::lock_guard guard(packLock);
    packs.erase(self);
}

// ------------------------------------------------------------------------------------------------
// Merge. Attach takes the posed mesh, drops the attached object's Skin and morphs and leaves the
// target's Skin guessing weights for the new vertices. Here each object's mesh is taken at the
// input of its Skin with its Morpher off (the bind shape), moved into the first object's space at
// the bind pose and appended in order, so every vertex has a known place. One Skin gets every bone
// with the bind pose the objects were skinned with, every vertex its own weights and dual
// quaternion blend; one Morpher gets every channel (channels of the same name, targets and values
// become one), built from target meshes as Morpher's own pick does. Map channels, explicit normals,
// smoothing groups and material IDs are carried (different materials go into a Multi/Sub). The
// objects are hidden, not deleted.

struct MergeSource {
    INode* node = nullptr;
    Modifier* skinMod = nullptr;
    ISkin* skin = nullptr;
    ISkinContextData* ctx = nullptr;
    int skinIndex = 0;               // 1-based in MAXScript's modifiers array (top first)
    Modifier* morphMod = nullptr;
    IMorpher* morpher = nullptr;
    int morphIndex = 0;
    std::vector<Point3> morphBase;   // the input of Morpher (progressive targets are points relative to it)
    std::vector<std::string> above;  // modifiers above Skin: not carried
    MNMesh mesh;                     // at the input of Skin with the morphs off, in its object space
    Matrix3 toMerged;                // its object space at the bind pose -> the merged object's
    std::vector<int> vertex;         // its vertex -> merged vertex (-1: dead)
};

struct MergeBone {
    INode* node;
    Matrix3 objectTm, nodeTm, stretchTm;  // at the bind pose
    size_t from;                          // the first object skinned to it
};

struct MergeChannel {
    std::wstring name;
    std::vector<double> percents;                 // per target (0: the main one)
    std::vector<std::pair<size_t, int>> parts;    // (object, its channel)
};

struct IdMap {  // an object's material IDs in the merged object
    int offset = 0, count = -1;  // count < 0: kept; 0: all to `offset`; else offset + id % count (as Multi/Sub picks)
    MtlID operator()(MtlID id) const { return (MtlID)(count < 0 ? id : offset + (count ? id % count : 0)); }
};

// Two bind matrices alike: the axes within 1e-4, the position within 1e-4 of its size.
bool close_tm(const Matrix3& a, const Matrix3& b) {
    for (int r = 0; r < 3; ++r)
        if (Length(a.GetRow(r) - b.GetRow(r)) > 1e-4f * std::max(1.0f, Length(a.GetRow(r)))) return false;
    return Length(a.GetRow(3) - b.GetRow(3)) <= 1e-4f * std::max(1.0f, Length(a.GetRow(3)));
}

std::wstring mxs_string(const std::wstring& s) {
    std::wstring q = L"\"";
    for (wchar_t c : s) {
        if (c == L'"' || c == L'\\') q += L'\\';
        q += c;
    }
    return q + L"\"";
}

bool run_mxs(const std::wstring& code, FPValue* result = nullptr) {
    return ExecuteMAXScriptScript(code.c_str(), MAXScript::ScriptSource::Dynamic, TRUE, result) != FALSE;
}

// An object's Skin, Morpher and bind shape.
bool read_source(INode* node, TimeValue t, MergeSource& s, std::string& err) {
    s.node = node;
    std::vector<std::pair<IDerivedObject*, int>> mods;  // top first
    for (Object* obj = node->GetObjectRef(); obj && obj->SuperClassID() == GEN_DERIVOB_CLASS_ID;
         obj = static_cast<IDerivedObject*>(obj)->GetObjRef())
        for (int i = 0; i < static_cast<IDerivedObject*>(obj)->NumModifiers(); ++i) mods.push_back({static_cast<IDerivedObject*>(obj), i});
    size_t skinAt = mods.size(), morphAt = mods.size();
    for (size_t k = 0; k < mods.size(); ++k) {
        Modifier* m = mods[k].first->GetModifier(mods[k].second);
        const bool morpher = m->ClassID() == MR3_CLASS_ID;
        if (skinAt == mods.size()) {
            if (auto* sk = static_cast<ISkin*>(m->GetInterface(I_SKIN))) {
                skinAt = k;
                s.skinMod = m;
                s.skin = sk;
                s.skinIndex = (int)k + 1;
            } else if (morpher) {
                err = "its Morpher is above Skin";
                return false;
            } else {
                s.above.push_back(utf8(m->GetName(false)));
            }
        } else if (morpher) {
            if (s.morphMod) {
                err = "it has two Morphers";
                return false;
            }
            s.morphMod = m;
            s.morpher = static_cast<IMorpher*>(m->GetInterface(I_MORPHER_INTERFACE_ID));
            s.morphIndex = (int)k + 1;
            morphAt = k;
        }
    }
    if (!s.skin) {
        err = "it has no Skin";
        return false;
    }
    const bool morphOn = s.morphMod && s.morphMod->IsEnabled();
    theHold.Suspend();
    if (morphOn) s.morphMod->DisableMod();
    auto [owner, i] = mods[skinAt];
    ObjectState os = i + 1 < owner->NumModifiers() ? owner->Eval(t, i + 1) : owner->GetObjRef()->Eval(t);
    const bool ok = os.obj && os.obj->CanConvertToType(polyObjectClassID);
    if (ok) {
        auto* po = static_cast<PolyObject*>(os.obj->ConvertToType(t, polyObjectClassID));
        s.mesh = po->GetMesh();
        if (po != os.obj) po->MaybeAutoDelete();
    }
    if (morphOn) s.morphMod->EnableMod();
    if (morphAt < mods.size()) {
        auto [mo, mi] = mods[morphAt];
        ObjectState ms = mi + 1 < mo->NumModifiers() ? mo->Eval(t, mi + 1) : mo->GetObjRef()->Eval(t);
        for (int v = 0; ms.obj && v < ms.obj->NumPoints(); ++v) s.morphBase.push_back(ms.obj->GetPoint(v));
    }
    theHold.Resume();
    node->EvalWorldState(t);  // Morpher and Skin as they were
    if (!ok) {
        err = "it is not a mesh";
        return false;
    }
    s.ctx = s.skin->GetContextInterface(node);
    if (!s.ctx || s.ctx->GetNumPoints() != s.mesh.VNum()) {
        err = "its Skin does not have the vertices of the mesh under it";
        return false;
    }
    return true;
}

// The objects' materials as one: kept when they share one, else a Multi/Sub with a slot per
// material (a Multi/Sub's slots in order), as Attach's "match material IDs to material" does.
Mtl* merged_material(const std::vector<std::unique_ptr<MergeSource>>& src, std::vector<IdMap>& ids) {
    ids.assign(src.size(), IdMap{});
    Mtl* first = src[0]->node->GetMtl();
    if (std::all_of(src.begin(), src.end(), [&](const auto& s) { return s->node->GetMtl() == first; })) return first;
    std::vector<std::pair<Mtl*, MSTR>> slots;
    std::map<Mtl*, IdMap> placed;
    for (size_t i = 0; i < src.size(); ++i) {
        Mtl* m = src[i]->node->GetMtl();
        if (auto it = placed.find(m); it != placed.end()) {
            ids[i] = it->second;
            continue;
        }
        IdMap map{(int)slots.size(), m && m->IsMultiMtl() ? m->NumSubMtls() : 0};
        if (map.count > 0) {
            for (int k = 0; k < map.count; ++k) slots.push_back({m->GetSubMtl(k), m->GetSubMtlSlotName(k, false)});
        } else {
            slots.push_back({m, MSTR(src[i]->node->GetName())});
        }
        ids[i] = placed[m] = map;
    }
    MultiMtl* multi = NewDefaultMultiMtl();
    multi->SetNumSubMtls((int)slots.size());
    for (int k = 0; k < (int)slots.size(); ++k) multi->SetSubMtlAndName(k, slots[k].first, slots[k].second);
    multi->SetName(MSTR(src[0]->node->GetName()) + _M("_merged"));
    return multi;
}

// The objects' meshes appended in order, in the merged object's space.
void merge_meshes(std::vector<std::unique_ptr<MergeSource>>& src, const std::vector<IdMap>& ids, MNMesh& out) {
    int nv = 0, nf = 0, mapNum = 0;
    for (auto& s : src) {
        s->vertex.assign(s->mesh.VNum(), -1);
        for (int v = 0; v < s->mesh.VNum(); ++v)
            if (!s->mesh.V(v)->GetFlag(MN_DEAD)) s->vertex[v] = nv++;
        for (int f = 0; f < s->mesh.FNum(); ++f) nf += !s->mesh.F(f)->GetFlag(MN_DEAD);
        mapNum = std::max(mapNum, s->mesh.MNum());
    }
    out.ClearAndFree();
    out.setNumVerts(nv);
    out.setNumFaces(nf);
    int fo = 0;
    std::vector<int> retriangulate;  // faces whose triangulation could not be copied
    for (size_t i = 0; i < src.size(); ++i) {
        MergeSource& s = *src[i];
        for (int v = 0; v < s.mesh.VNum(); ++v)
            if (s.vertex[v] >= 0) out.V(s.vertex[v])->p = s.mesh.P(v) * s.toMerged;
        for (int f = 0; f < s.mesh.FNum(); ++f) {
            const MNFace* S = s.mesh.F(f);
            if (S->GetFlag(MN_DEAD)) continue;
            MNFace* F = out.F(fo++);
            F->SetDeg(S->deg);
            for (int k = 0; k < S->deg; ++k) F->vtx[k] = s.vertex[S->vtx[k]];
            if (S->deg > 3 && S->diag && F->diag)
                for (int d = 0; d < (S->deg - 3) * 2; ++d) F->diag[d] = S->diag[d];
            else if (S->deg > 3)
                retriangulate.push_back(fo - 1);
            F->material = ids[i](S->material);
            F->smGroup = S->smGroup;
        }
    }

    // Map channels: every channel any object has; an object without it gets one value (white for
    // vertex color, illumination and alpha, 0 for UVs).
    out.SetMapNum(mapNum);
    for (int mp = -NUM_HIDDENMAPS; mp < mapNum; ++mp) {
        auto live = [&](const MergeSource& s) {
            const std::vector<int> maps = live_maps(const_cast<MNMesh&>(s.mesh));
            return std::find(maps.begin(), maps.end(), mp) != maps.end();
        };
        if (std::none_of(src.begin(), src.end(), [&](const auto& s) { return live(*s); })) {
            out.ClearMap(mp);
            continue;
        }
        MNMap* d = out.M(mp);
        d->ClearAndFree();
        d->ClearFlag(MN_DEAD);
        int total = 0;
        for (auto& s : src) total += live(*s) ? s->mesh.M(mp)->VNum() : 1;
        d->setNumVerts(total);
        d->setNumFaces(nf);
        int vo = 0;
        fo = 0;
        for (auto& s : src) {
            const MNMap* sm = live(*s) ? s->mesh.M(mp) : nullptr;
            if (sm) for (int v = 0; v < sm->VNum(); ++v) d->v[vo + v] = sm->v[v];
            else d->v[vo] = mp <= 0 ? UVVert(1, 1, 1) : UVVert(0, 0, 0);
            for (int f = 0; f < s->mesh.FNum(); ++f) {
                if (s->mesh.F(f)->GetFlag(MN_DEAD)) continue;
                MNMapFace* mf = d->F(fo++);
                mf->SetSize(s->mesh.F(f)->deg);
                for (int k = 0; k < mf->deg; ++k) mf->tv[k] = vo + (sm ? sm->F(f)->tv[k] : 0);
            }
            vo += sm ? sm->VNum() : 1;
        }
    }
    out.FillInMesh();
    for (int f : retriangulate) out.RetriangulateFace(f);

    // Explicit normals (turned with the object); objects without them get smoothing-group normals.
    if (std::none_of(src.begin(), src.end(), [](const auto& s) { return live_normals(s->mesh) != nullptr; })) return;
    out.SpecifyNormals();
    MNNormalSpec* dn = out.GetSpecifiedNormals();
    dn->SetParent(&out);
    int nn = 0;
    for (auto& s : src)
        if (MNNormalSpec* sn = live_normals(s->mesh)) nn += sn->GetNumNormals();
    dn->SetNumFaces(nf);
    dn->SetNumNormals(nn);
    int no = 0;
    fo = 0;
    for (auto& s : src) {
        MNNormalSpec* sn = live_normals(s->mesh);
        if (sn)
            for (int k = 0; k < sn->GetNumNormals(); ++k) {
                dn->Normal(no + k) = Normalize(VectorTransform(sn->Normal(k), s->toMerged));
                dn->SetNormalExplicit(no + k, sn->GetNormalExplicit(k));
            }
        for (int f = 0; f < s->mesh.FNum(); ++f) {
            if (s->mesh.F(f)->GetFlag(MN_DEAD)) continue;
            MNNormalFace& fn = dn->Face(fo++);
            fn.SetDegree(s->mesh.F(f)->deg);
            for (int k = 0; k < s->mesh.F(f)->deg; ++k) {
                const int id = sn ? sn->Face(f).GetNormalID(k) : -1;
                fn.SetNormalID(k, id < 0 ? -1 : no + id);
                fn.SetSpecified(k, sn && id >= 0 && sn->Face(f).GetSpecified(k));
            }
        }
        if (sn) no += sn->GetNumNormals();
    }
    dn->CheckNormals();
}

// A channel's value at every frame of the animation range (its controller, or its constant value).
std::vector<float> channel_values(IMorpherChannel* ch) {
    std::vector<float> values;
    const Interval range = GetCOREInterface()->GetAnimRange();
    Control* c = ch->GetControl();
    for (TimeValue t = range.Start(); t <= range.End(); t += GetTicksPerFrame()) {
        float v = 0;
        Interval valid = FOREVER;
        if (c) c->GetValue(t, &v, valid);
        else v = (float)ch->GetInitPercent();
        values.push_back(v);
        if (!c) break;
    }
    return values;
}

// The morph channels of the merged object, in the order the objects and their channels come.
// Channels of different objects become one when they have the same name, targets (the percents of
// the progressive ones) and value at every frame, as the parts of one character driven together
// do; otherwise a channel keeps its own, named after its object when the name is taken.
std::vector<MergeChannel> merge_channels(const std::vector<std::unique_ptr<MergeSource>>& src, std::string& log) {
    std::vector<MergeChannel> out;
    std::vector<std::vector<float>> values;  // per merged channel: its first part's values
    for (size_t i = 0; i < src.size(); ++i) {
        IMorpher* mr = src[i]->morpher;
        for (int c = 0; mr && c < mr->NumChannels(); ++c) {
            IMorpherChannel* ch = mr->GetChannel(c);
            if (!ch || !ch->IsActive() || ch->NumPoints() == 0) continue;
            const MCHAR* nm = ch->GetName(true);  // the name shown (WM3_MC_SetName sets only this one)
            std::wstring name = nm && *nm ? nm : (ch->GetName(false) ? ch->GetName(false) : L"");
            if (ch->NumPoints() != src[i]->mesh.VNum()) {
                log += "  ! " + utf8(MSTR(src[i]->node->GetName())) + ", morph " + utf8(MSTR(name.c_str())) +
                       ": its points do not match the mesh, left out\n";
                continue;
            }
            std::vector<double> percents;
            for (int k = 0; k < ch->GetTargetCount(); ++k) percents.push_back(ch->GetTargetPercent(k));
            const std::vector<float> value = channel_values(ch);
            size_t at = out.size();
            for (size_t m = 0; m < out.size() && at == out.size(); ++m)
                if (out[m].name == name && out[m].percents == percents && values[m] == value &&
                    std::none_of(out[m].parts.begin(), out[m].parts.end(), [&](const auto& p) { return p.first == i; }))
                    at = m;
            if (at == out.size()) {
                const bool taken = std::any_of(out.begin(), out.end(), [&](const MergeChannel& m) { return m.name == name; });
                out.push_back({taken ? name + L" (" + src[i]->node->GetName() + L")" : name, percents, {}});
                values.push_back(value);
            }
            out[at].parts.push_back({i, c});
        }
    }
    return out;
}

// Builds merged channel `index` (1-based) from target meshes: the merged bind shape with each
// part's offsets for that target, turned into the merged object's space. Morpher moves a point by
// its stored delta in a channel with one target, and along a spline through its input point and
// the targets' points in a progressive channel (wm3_core.cpp), so the offsets are those.
bool build_channel(INode* merged, int morphIndex, int index, const MergeChannel& ch, const std::vector<std::unique_ptr<MergeSource>>& src,
                   const MNMesh& base, TimeValue t, std::string& log) {
    const bool progressive = ch.percents.size() > 1;
    std::vector<INode*> targets;
    for (size_t k = 0; k < ch.percents.size(); ++k) {
        TriObject* obj = CreateNewTriObject();
        Mesh& m = obj->GetMesh();
        m.setNumVerts(base.VNum());
        for (int v = 0; v < base.VNum(); ++v) m.setVert(v, base.P(v));
        for (const auto& [i, c] : ch.parts) {
            const MergeSource& s = *src[i];
            IMorpherChannel* sc = s.morpher->GetChannel(c);
            Tab<Point3> deltas;
            if (!progressive) {
                sc->GetBaseDeltas(deltas);
            } else if (s.morphBase.size() == s.vertex.size()) {
                deltas.SetCount((int)s.morphBase.size());
                for (int v = 0; v < deltas.Count(); ++v) deltas[v] = sc->GetTargetPoint((int)k, v) - s.morphBase[v];
            }
            for (int v = 0; v < deltas.Count() && v < (int)s.vertex.size(); ++v)
                if (s.vertex[v] >= 0) m.setVert(s.vertex[v], m.getVert(s.vertex[v]) + VectorTransform(deltas[v], s.toMerged));
        }
        INode* node = GetCOREInterface()->CreateObjectNode(obj);
        node->Hide(TRUE);
        targets.push_back(node);
    }
    // Settings and value (animation, wires) from the channel's first part.
    const MergeSource& first = *src[ch.parts[0].first];
    const int j = ch.parts[0].second + 1;
    std::wstring code = L"(local m = (maxOps.getNodeByHandle " + std::to_wstring(merged->GetHandle()) + L").modifiers[" + std::to_wstring(morphIndex) +
                        L"]\nlocal s = (maxOps.getNodeByHandle " + std::to_wstring(first.node->GetHandle()) + L").modifiers[" +
                        std::to_wstring(first.morphIndex) + L"]\nlocal t = #(";
    for (size_t k = 0; k < targets.size(); ++k) code += (k ? L", " : L"") + std::to_wstring(targets[k]->GetHandle());
    code += L")\nt = for h in t collect maxOps.getNodeByHandle h\nlocal i = " + std::to_wstring(index) + L"\nlocal j = " + std::to_wstring(j) +
            L"\nWM3_MC_BuildFromNode m i t[1]\nfor k = 2 to t.count do WM3_AddProgressiveMorphNode m i t[k]\nlocal w = #(";
    for (size_t k = 0; k < ch.percents.size(); ++k) code += (k ? L", " : L"") + std::to_wstring(ch.percents[k]);
    code += L")\nif t.count > 1 do (for k = 1 to t.count do WM3_SetProgressiveMorphWeight m i t[k] w[k]; "
            L"WM3_SetProgressiveMorphTension m i (WM3_GetProgressiveMorphTension s j))\n"
            L"WM3_MC_SetName m i " + mxs_string(ch.name) +
            L"\nWM3_MC_SetUseLimits m i (WM3_MC_GetUseLimits s j)\nWM3_MC_SetLimitMIN m i (WM3_MC_GetLimitMIN s j)\n"
            L"WM3_MC_SetLimitMAX m i (WM3_MC_GetLimitMAX s j)\n"
            L"try (m[i].controller = copy s[j].controller) catch (WM3_MC_SetValue m i (WM3_MC_GetValue s j))\n"
            L"WM3_MC_GetUseVertexSel s j)";
    FPValue r;
    const bool ok = run_mxs(code, &r);
    for (INode* n : targets) GetCOREInterface()->DeleteNode(n, FALSE);
    if (!ok) {
        log += "  ! morph " + utf8(MSTR(ch.name.c_str())) + ": could not be built\n";
        return false;
    }
    if (r.type == TYPE_BOOL && r.b)
        log += "  ! morph " + utf8(MSTR(ch.name.c_str())) + " uses a vertex selection in " + utf8(MSTR(first.node->GetName())) +
               ": the merged channel moves every vertex the target moves\n";
    return true;
}

bool merge(const std::vector<INode*>& nodes, ExodusSink out, INode*& mergedOut) {
    mergedOut = nullptr;
    std::string log;
    auto fail = [&](const std::string& why) {
        log += why + "\n";
        out.write(out.ctx, log.data(), log.size());
        return false;
    };
    std::vector<INode*> list;
    for (INode* n : nodes)
        if (n && std::find(list.begin(), list.end(), n) == list.end()) list.push_back(n);
    if (list.size() < 2) return fail("select two or more skinned objects");
    const TimeValue t = GetCOREInterface()->GetTime();

    std::vector<std::unique_ptr<MergeSource>> src;
    for (INode* n : list) {
        src.push_back(std::make_unique<MergeSource>());
        std::string err;
        if (!read_source(n, t, *src.back(), err)) return fail(utf8(MSTR(n->GetName())) + ": " + err);
        if (!src.back()->above.empty()) {
            log += "  ! " + utf8(MSTR(n->GetName())) + ": not carried (above Skin):";
            for (const std::string& m : src.back()->above) log += " " + m;
            log += "\n";
        }
    }

    // The bind pose: one per bone over all objects, and the first object's space. A bone's comes
    // from the objects with weights on it, which must agree; an object that lists a bone without
    // using it (FBX imports list the whole rig) may hold any matrix for it, and none of its
    // vertices depend on it.
    std::vector<MergeBone> bones;
    std::map<INode*, size_t> boneAt;
    for (int pass = 0; pass < 2; ++pass)
        for (size_t i = 0; i < src.size(); ++i) {
            MergeSource& s = *src[i];
            std::set<INode*> used;
            for (int v = 0; v < s.ctx->GetNumPoints(); ++v)
                for (int k = 0; k < s.ctx->GetNumAssignedBones(v); ++k)
                    if (s.ctx->GetBoneWeight(v, k) > 0) used.insert(s.skin->GetBone(s.ctx->GetAssignedBone(v, k)));
            auto* s2 = static_cast<ISkin2*>(s.skinMod->GetInterface(I_SKIN2));
            for (int b = 0; b < s.skin->GetNumBonesFlat(); ++b) {
                INode* bone = s.skin->GetBoneFlat(b);
                if (!bone || (pass == 0) != (used.count(bone) > 0)) continue;  // users first, then the rest
                MergeBone mb{bone, Matrix3(), Matrix3(), Matrix3(), i};
                mb.objectTm.IdentityMatrix();
                mb.nodeTm.IdentityMatrix();
                mb.stretchTm.IdentityMatrix();
                s.skin->GetBoneInitTM(bone, mb.objectTm, true);
                s.skin->GetBoneInitTM(bone, mb.nodeTm, false);
                if (s2) mb.stretchTm = s2->GetBoneStretchTm(bone);
                if (auto it = boneAt.find(bone); it != boneAt.end()) {
                    if (pass == 0 && !close_tm(bones[it->second].objectTm, mb.objectTm))
                        return fail(utf8(MSTR(bone->GetName())) + " has another bind pose in " + utf8(MSTR(s.node->GetName())) + " than in " +
                                    utf8(MSTR(src[bones[it->second].from]->node->GetName())) + ": they cannot share one Skin");
                    continue;
                }
                boneAt[bone] = bones.size();
                bones.push_back(mb);
            }
        }
    // The merged Skin lists the bones in the objects' order.
    std::vector<MergeBone> ordered;
    for (auto& s : src)
        for (int b = 0; b < s->skin->GetNumBonesFlat(); ++b)
            if (INode* bone = s->skin->GetBoneFlat(b); bone && boneAt.count(bone) &&
                std::none_of(ordered.begin(), ordered.end(), [&](const MergeBone& m) { return m.node == bone; }))
                ordered.push_back(bones[boneAt[bone]]);
    bones = std::move(ordered);
    Matrix3 baseObj, baseNode;
    src[0]->skin->GetSkinInitTM(src[0]->node, baseObj, true);
    src[0]->skin->GetSkinInitTM(src[0]->node, baseNode, false);
    for (auto& s : src) {
        Matrix3 tm;
        s->skin->GetSkinInitTM(s->node, tm, true);
        if (tm == baseObj) s->toMerged.IdentityMatrix();
        else s->toMerged = tm * Inverse(baseObj);
    }

    std::vector<IdMap> ids;
    Mtl* mtl = merged_material(src, ids);
    auto mesh = std::make_unique<MNMesh>();
    merge_meshes(src, ids, *mesh);
    const std::vector<MergeChannel> channels = merge_channels(src, log);

    theHold.Begin();
    INode* first = src[0]->node;
    PolyObject* obj = CreateEditablePolyObject();
    obj->GetMesh() = *mesh;
    INode* merged = GetCOREInterface()->CreateObjectNode(obj);
    MSTR name = MSTR(first->GetName()) + _M("_merged");
    if (GetCOREInterface()->GetINodeByName(name)) GetCOREInterface()->MakeNameUnique(name);
    merged->SetName(name);
    merged->SetNodeTM(t, first->GetNodeTM(t));
    merged->SetObjOffsetPos(first->GetObjOffsetPos());
    merged->SetObjOffsetRot(first->GetObjOffsetRot());
    merged->SetObjOffsetScale(first->GetObjOffsetScale());
    merged->SetWireColor(first->GetWireColor());
    merged->SetMtl(mtl);
    if (INode* parent = first->GetParentNode(); parent && !parent->IsRootNode()) parent->AttachChild(merged, 1);

    // Morpher first: each channel is built against the merged bind shape, then Skin on top.
    int built = 0;
    if (!channels.empty()) {
        auto* morph = static_cast<Modifier*>(CreateInstance(OSM_CLASS_ID, MR3_CLASS_ID));
        if (!morph || GetCOREInterface7()->AddModifier(*merged, *morph) != Interface7::kRES_SUCCESS) {
            theHold.Cancel();
            return fail("cannot add a Morpher");
        }
        merged->EvalWorldState(t);
        for (size_t c = 0; c < channels.size(); ++c) built += build_channel(merged, 1, (int)c + 1, channels[c], src, *mesh, t, log);
    }
    auto* skinMod = static_cast<Modifier*>(CreateInstance(OSM_CLASS_ID, SKIN_CLASSID));
    if (!skinMod || GetCOREInterface7()->AddModifier(*merged, *skinMod) != Interface7::kRES_SUCCESS) {
        theHold.Cancel();
        return fail("cannot add a Skin");
    }
    auto* import = static_cast<ISkinImportData*>(skinMod->GetInterface(I_SKINIMPORTDATA));
    auto* skin2 = static_cast<ISkin2*>(skinMod->GetInterface(I_SKIN2));
    for (const MergeBone& b : bones) import->AddBoneEx(b.node, FALSE);
    merged->EvalWorldState(t);  // Skin makes its data for the node
    import->SetSkinTm(merged, baseObj, baseNode);
    for (const MergeBone& b : bones) {
        import->SetBoneTm(b.node, b.objectTm, b.nodeTm);
        if (skin2) skin2->SetBoneStretchTm(b.node, b.stretchTm);  // after SetBoneTm, which clears it
    }
    skinMod->NotifyDependents(FOREVER, PART_ALL, REFMSG_CHANGE);
    merged->EvalWorldState(t);
    auto* skin = static_cast<ISkin*>(skinMod->GetInterface(I_SKIN));
    ISkinContextData* ctx = skin ? skin->GetContextInterface(merged) : nullptr;
    if (!ctx || ctx->GetNumPoints() != mesh->VNum()) {
        theHold.Cancel();
        return fail("the new Skin does not have the merged vertices");
    }
    Tab<INode*> wb;
    Tab<float> ww;
    size_t unweighted = 0;
    for (auto& s : src)
        for (int v = 0; v < (int)s->vertex.size(); ++v) {
            if (s->vertex[v] < 0) continue;
            wb.SetCount(0);
            ww.SetCount(0);
            for (int k = 0; k < s->ctx->GetNumAssignedBones(v); ++k) {
                INode* b = s->skin->GetBone(s->ctx->GetAssignedBone(v, k));
                float w = s->ctx->GetBoneWeight(v, k);
                if (!b) continue;
                wb.Append(1, &b);
                ww.Append(1, &w);
            }
            if (wb.Count()) import->AddWeights(merged, s->vertex[v], wb, ww);
            else ++unweighted;
        }
    // Dual quaternion blending: on when any object has it, each vertex with its own blend weight
    // (0, linear, for the vertices of an object without it).
    const bool dq = std::any_of(src.begin(), src.end(), [](const auto& s) { return s->skin->GetDQBlending() != FALSE; });
    skin->SetDQBlending(dq);
    if (dq)
        for (auto& s : src) {
            const bool on = s->skin->GetDQBlending() != FALSE;
            for (int v = 0; v < (int)s->vertex.size(); ++v)
                if (s->vertex[v] >= 0) ctx->SetDQBlendWeight(s->vertex[v], on ? s->ctx->GetDQBlendWeight(v) : 0.0f);
        }
    skinMod->NotifyDependents(FOREVER, PART_ALL, REFMSG_CHANGE);

    // Skin's bone limit (the largest of the objects'), the layer, the objects hidden.
    std::wstring code = L"(local n = maxOps.getNodeByHandle " + std::to_wstring(merged->GetHandle()) + L"\nlocal src = #(";
    for (size_t i = 0; i < src.size(); ++i) code += (i ? L", " : L"") + std::to_wstring(src[i]->node->GetHandle());
    code += L")\nsrc = for h in src collect maxOps.getNodeByHandle h\nlocal skins = #(";
    for (size_t i = 0; i < src.size(); ++i) code += (i ? L", " : L"") + std::to_wstring(src[i]->skinIndex);
    code += L")\ntry (n.modifiers[1].bone_Limit = amax (for i = 1 to src.count collect src[i].modifiers[skins[i]].bone_Limit)) catch ()\n"
            L"src[1].layer.addNode n\nhide src\nselect n)";
    run_mxs(code);
    theHold.Accept(_M("Merge skinned"));
    merged->EvalWorldState(t);
    mergedOut = merged;

    char line[512];
    std::snprintf(line, sizeof line, "merged %zu objects into %s: %d vertices, %d faces, %zu bones, %d of %zu morph channels\n", src.size(),
                  utf8(merged->GetName()).c_str(), mesh->VNum(), mesh->FNum(), bones.size(), built, channels.size());
    std::string report = line;
    if (unweighted) report += "  ! " + std::to_string(unweighted) + " vertices had no weights (Skin gives them envelope weights)\n";
    if (mtl && mtl->IsMultiMtl() && mtl != first->GetMtl()) report += "  materials combined in " + utf8(mtl->GetName()) + "\n";
    report += log;
    report += "the objects are hidden; delete them when the merged one is right\n";
    out.write(out.ctx, report.data(), report.size());
    return true;
}

const ExodusCoreApi kApi{kExodusCoreApiVersion, kBuild, inspect, unsubdivide_modify, decimate_modify, write_weights,
                       pack_modify, pack_bake, pack_preview, pack_detect, pack_forget, merge};

}  // namespace

extern "C" __declspec(dllexport) const ExodusCoreApi* exodus_core_api() { return &kApi; }
