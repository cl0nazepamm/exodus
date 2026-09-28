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
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "maxcore_api.h"

namespace {

std::string utf8(const MSTR& s) { return std::string(s.ToUTF8().data()); }

std::string quoted(const std::string& s) {
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
    std::string j = "{\"name\": " + quoted(utf8(m->GetName())) + ", \"class\": " + quoted(class_name(m)) + ", \"maps\": [";
    bool first = true;
    for (int i = 0; i < m->NumSubTexmaps(); ++i) {
        Texmap* tm = m->GetSubTexmap(i);
        if (!tm) continue;
        std::vector<std::string> files;
        bitmap_files(tm, files);
        j += std::string(first ? "" : ", ") + "{\"slot\": " + quoted(utf8(m->GetSubTexmapSlotName(i, false))) +
             ", \"texmap\": " + quoted(class_name(tm)) + ", \"files\": [";
        for (size_t k = 0; k < files.size(); ++k) j += (k ? ", " : "") + quoted(files[k]);
        j += "]}";
        first = false;
    }
    return j + "]}";
}

bool inspect(INode* node, UdimSink out) {
    const TimeValue t = GetCOREInterface()->GetTime();
    std::string j = "{\"node\": " + quoted(utf8(MSTR(node->GetName())));
    const Stack st = stack_of(node);
    j += ", \"modifiers\": [";
    for (size_t i = 0; i < st.modifiers.size(); ++i) j += (i ? ", " : "") + quoted(st.modifiers[i]);
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
    j += ", \"mesh\": {\"source\": " + quoted(st.skin ? "input of Skin" : "final") + ", \"class\": " + quoted(class_name(os.obj)) +
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
        j += ", \"material\": {\"name\": " + quoted(utf8(mtl->GetName())) + ", \"class\": " + quoted(class_name(mtl)) +
             ", \"multi\": " + (mtl->IsMultiMtl() ? "true" : "false") + ", \"subs\": [";
        if (mtl->IsMultiMtl()) {
            for (int i = 0; i < mtl->NumSubMtls(); ++i) {
                Mtl* sub = mtl->GetSubMtl(i);
                j += std::string(i ? ", " : "") + "{\"index\": " + std::to_string(i + 1) + ", \"slot\": " +
                     quoted(utf8(mtl->GetSubMtlSlotName(i, false))) + ", \"material\": " + (sub ? material_json(sub) : "null") + "}";
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

const UdimCoreApi kApi{kUdimCoreApiVersion, __DATE__ " " __TIME__, inspect};

}  // namespace

extern "C" __declspec(dllexport) const UdimCoreApi* udim_core_api() { return &kApi; }
