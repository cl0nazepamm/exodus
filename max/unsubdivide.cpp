// udimpacker for 3ds Max: the Unsubdivide modifier. It sits below Skin; the work (plan, result,
// Skin weight copy) is in the core DLL (unsubdivide_modify / unsubdivide_write_weights, see
// maxcore.cpp for why Skin keeps its weights). This part holds the parameters and the per-node
// state, queues the Skin weight update for after the evaluation, and saves the weight copy with
// the file (bones by name).
#include <iparamm2.h>
#include <modstack.h>
#include <polyobj.h>
#include <Util/MainThreadTaskManager.h>

#include <set>

#include "plugin.h"
#include "resource.h"

namespace {

const Class_ID kUnsubdivideClassId(0x4a3f19c2, 0x7de20b61);

enum { kParams };
enum { kLevels, kSymmetry };

// Save chunks of the per-node data.
enum : USHORT { kChunkBones = 0x100, kChunkBone, kChunkWeights, kChunkSynced };

class UnsubdivideMod;
std::set<UnsubdivideMod*> instances;

class UnsubdivideData : public LocalModData {
public:
    UdimUnsubdivideState state;
    LocalModData* Clone() override { return new UnsubdivideData(*this); }
};

// The modifier's context on a node.
ModContext* context_of(INode* node, Modifier* mod) {
    for (Object* obj = node ? node->GetObjectRef() : nullptr; obj && obj->SuperClassID() == GEN_DERIVOB_CLASS_ID;
         obj = static_cast<IDerivedObject*>(obj)->GetObjRef()) {
        auto* d = static_cast<IDerivedObject*>(obj);
        for (int i = 0; i < d->NumModifiers(); ++i)
            if (d->GetModifier(i) == mod) return d->GetModContext(i);
    }
    return nullptr;
}

void post_weights(INode* node, UnsubdivideMod* mod, UnsubdivideData& data);

class UnsubdivideMod : public Modifier {
public:
    IParamBlock2* pblock = nullptr;
    MSTR status;

    UnsubdivideMod();
    ~UnsubdivideMod() override { instances.erase(this); }

    // Animatable
    void DeleteThis() override { delete this; }
    void GetClassName(MSTR& s, bool localized) const override { s = localized ? udim_string(IDS_UNSUBDIVIDE) : _M("Unsubdivide"); }
    Class_ID ClassID() override { return kUnsubdivideClassId; }
    int NumSubs() override { return 1; }
    Animatable* SubAnim(int) override { return pblock; }
    MSTR SubAnimName(int, bool localized) override { return localized ? udim_string(IDS_PARAMETERS) : _M("Parameters"); }
    int NumParamBlocks() override { return 1; }
    IParamBlock2* GetParamBlock(int) override { return pblock; }
    IParamBlock2* GetParamBlockByID(BlockID id) override { return pblock && pblock->ID() == id ? pblock : nullptr; }
    void BeginEditParams(IObjParam* ip, ULONG flags, Animatable* prev) override;
    void EndEditParams(IObjParam* ip, ULONG flags, Animatable* next) override;

    // References
    int NumRefs() override { return 1; }
    RefTargetHandle GetReference(int) override { return pblock; }
    RefTargetHandle Clone(RemapDir& remap) override;
    RefResult NotifyRefChanged(const Interval&, RefTargetHandle, PartID&, RefMessage, BOOL) override { return REF_SUCCEED; }

protected:
    void SetReference(int, RefTargetHandle target) override { pblock = static_cast<IParamBlock2*>(target); }

public:
    // BaseObject / Modifier
    const MCHAR* GetObjectName(bool localized) const override { return localized ? udim_string(IDS_UNSUBDIVIDE) : _M("Unsubdivide"); }
    CreateMouseCallBack* GetCreateMouseCallBack() override { return nullptr; }
    BOOL ChangeTopology() override { return TRUE; }
    ChannelMask ChannelsUsed() override { return GEOM_CHANNEL | TOPO_CHANNEL | TEXMAP_CHANNEL | VERTCOLOR_CHANNEL | SELECT_CHANNEL | SUBSEL_TYPE_CHANNEL; }
    ChannelMask ChannelsChanged() override { return GEOM_CHANNEL | TOPO_CHANNEL | TEXMAP_CHANNEL | VERTCOLOR_CHANNEL | SELECT_CHANNEL | SUBSEL_TYPE_CHANNEL; }
    Class_ID InputType() override { return polyObjectClassID; }
    Interval LocalValidity(TimeValue) override { return TestAFlag(A_MOD_BEING_EDITED) ? NEVER : FOREVER; }
    void ModifyObject(TimeValue t, ModContext& mc, ObjectState* os, INode* node) override;
    IOResult SaveLocalData(ISave* isave, LocalModData* ld) override;
    IOResult LoadLocalData(ILoad* iload, LocalModData** pld) override;

    void ShowStatus();
};

class UnsubdivideClassDesc : public ClassDesc2 {
public:
    int IsPublic() override { return TRUE; }
    void* Create(BOOL) override { return new UnsubdivideMod; }
    const MCHAR* ClassName() override { return udim_string(IDS_UNSUBDIVIDE); }
    const MCHAR* NonLocalizedClassName() override { return _M("Unsubdivide"); }
    SClass_ID SuperClassID() override { return OSM_CLASS_ID; }
    Class_ID ClassID() override { return kUnsubdivideClassId; }
    const MCHAR* Category() override { return udim_string(IDS_CATEGORY); }
    const MCHAR* InternalName() override { return _M("udimUnsubdivide"); }
    HINSTANCE HInstance() override { return hInstance; }
};

UnsubdivideClassDesc unsubdivideDesc;

// Shows the modifier's status line when the rollout opens.
class UnsubdivideDlgProc : public ParamMap2UserDlgProc {
public:
    INT_PTR DlgProc(TimeValue, IParamMap2* map, HWND, UINT msg, WPARAM, LPARAM) override {
        if (msg == WM_INITDIALOG)
            if (auto* mod = static_cast<UnsubdivideMod*>(map->GetParamBlock()->GetOwner())) mod->ShowStatus();
        return FALSE;
    }
    void DeleteThis() override {}
};

UnsubdivideDlgProc unsubdivideDlgProc;

// clang-format off
ParamBlockDesc2 unsubdividePbDesc(kParams, _M("parameters"), IDS_PARAMETERS, &unsubdivideDesc, P_AUTO_CONSTRUCT + P_AUTO_UI, 0,
    IDD_UNSUBDIVIDE, IDS_PARAMETERS, 0, 0, &unsubdivideDlgProc,
    kLevels, _M("levels"), TYPE_INT, 0, IDS_LEVELS,
        p_default, 1,
        p_range, 0, 4,
        p_ui, TYPE_SPINNER, EDITTYPE_INT, IDC_LEVELS_EDIT, IDC_LEVELS_SPIN, 1.0f,
        p_end,
    kSymmetry, _M("symmetry"), TYPE_BOOL, 0, IDS_SYMMETRY,
        p_default, TRUE,
        p_ui, TYPE_SINGLECHECKBOX, IDC_SYMMETRY,
        p_end,
    p_end);
// clang-format on

UnsubdivideMod::UnsubdivideMod() {
    instances.insert(this);
    unsubdivideDesc.MakeAutoParamBlocks(this);
}

RefTargetHandle UnsubdivideMod::Clone(RemapDir& remap) {
    auto* mod = new UnsubdivideMod;
    mod->ReplaceReference(0, remap.CloneRef(pblock));
    BaseClone(this, mod, remap);
    return mod;
}

void UnsubdivideMod::BeginEditParams(IObjParam* ip, ULONG flags, Animatable* prev) {
    unsubdivideDesc.BeginEditParams(ip, this, flags, prev);
}

void UnsubdivideMod::EndEditParams(IObjParam* ip, ULONG flags, Animatable* next) {
    unsubdivideDesc.EndEditParams(ip, this, flags, next);
}

void UnsubdivideMod::ShowStatus() {
    IParamMap2* map = unsubdivideDesc.GetParamMap(&unsubdividePbDesc);
    if (map && map->GetParamBlock() == pblock && map->GetHWnd()) SetDlgItemText(map->GetHWnd(), IDC_STATUS, status.data());
}

void UnsubdivideMod::ModifyObject(TimeValue t, ModContext& mc, ObjectState* os, INode* node) {
    if (!os->obj->IsSubClassOf(polyObjectClassID)) return;
    PolyObject* po = static_cast<PolyObject*>(os->obj);
    Interval valid = FOREVER;
    int levels = 1;
    BOOL symmetry = TRUE;
    pblock->GetValue(kLevels, t, levels, valid);
    pblock->GetValue(kSymmetry, t, symmetry, valid);

    auto* data = static_cast<UnsubdivideData*>(mc.localData);
    if (!data) mc.localData = data = new UnsubdivideData;
    if (const UdimCoreApi* core = udim_core()) {
        std::wstring text;
        INode* owner = nullptr;
        if (core->unsubdivide_modify(this, mc, node, po->GetMesh(), UdimUnsubdivide{levels, symmetry != FALSE}, data->state, text, owner) &&
            !data->state.pending)
            post_weights(owner, this, *data);
        status = text.c_str();
    } else {
        status = udim_core_error();
    }
    ShowStatus();

    po->UpdateValidity(GEOM_CHAN_NUM, valid);
    po->UpdateValidity(TOPO_CHAN_NUM, valid);
    po->UpdateValidity(TEXMAP_CHAN_NUM, valid);
    po->UpdateValidity(VERT_COLOR_CHAN_NUM, valid);
    po->UpdateValidity(SELECT_CHAN_NUM, valid);
}

// After the evaluation: the core writes the copied weights into Skin.
class WriteWeightsTask : public MainThreadTask {
public:
    WriteWeightsTask(ULONG node, UnsubdivideMod* mod) : node(node), mod(mod) {}
    void Execute() override {
        if (!instances.count(mod)) return;
        INode* n = GetCOREInterface()->GetINodeByHandle(node);
        ModContext* mc = context_of(n, mod);
        auto* data = mc ? static_cast<UnsubdivideData*>(mc->localData) : nullptr;
        if (!data) return;
        if (const UdimCoreApi* core = udim_core()) core->unsubdivide_write_weights(n, mod, data->state);  // `pending` holds off new posts meanwhile
        data->state.pending = false;
        mod->NotifyDependents(FOREVER, PART_DISPLAY, REFMSG_CHANGE);  // refresh the status line
        GetCOREInterface()->RedrawViews(GetCOREInterface()->GetTime());
    }

private:
    ULONG node;
    UnsubdivideMod* mod;
};

void post_weights(INode* node, UnsubdivideMod* mod, UnsubdivideData& data) {
    if (!node) return;
    data.state.pending = true;
    IMainThreadTaskManager::GetInstance()->PostTask(new WriteWeightsTask(node->GetHandle(), mod));
}

IOResult UnsubdivideMod::SaveLocalData(ISave* isave, LocalModData* ld) {
    auto* data = static_cast<UnsubdivideData*>(ld);
    if (!data || data->state.skin.empty()) return IO_OK;
    const UdimSkinCopy& c = data->state.skin;
    ULONG written;
    isave->BeginChunk(kChunkBones);
    for (size_t i = 0; i < c.bones.size(); ++i) {
        isave->BeginChunk(kChunkBone);
        isave->WriteWString(c.bones[i] ? c.bones[i]->GetName() : c.boneNames[i].c_str());
        isave->EndChunk();
    }
    isave->EndChunk();
    isave->BeginChunk(kChunkWeights);
    const uint32_t nv = (uint32_t)c.first.size() - 1, nw = (uint32_t)c.bone.size();
    isave->Write(&nv, sizeof nv, &written);
    isave->Write(&nw, sizeof nw, &written);
    isave->Write(c.first.data(), (ULONG)(c.first.size() * sizeof(uint32_t)), &written);
    isave->Write(c.bone.data(), (ULONG)(nw * sizeof(uint32_t)), &written);
    isave->Write(c.weight.data(), (ULONG)(nw * sizeof(float)), &written);
    isave->EndChunk();
    if (c.inSync) {
        isave->BeginChunk(kChunkSynced);
        const uint32_t ns = (uint32_t)c.synced.size();
        isave->Write(&ns, sizeof ns, &written);
        isave->Write(c.synced.data(), (ULONG)(ns * sizeof(uint32_t)), &written);
        isave->EndChunk();
    }
    return IO_OK;
}

IOResult UnsubdivideMod::LoadLocalData(ILoad* iload, LocalModData** pld) {
    auto* data = new UnsubdivideData;
    *pld = data;
    UdimSkinCopy& c = data->state.skin;
    ULONG read;
    IOResult res;
    while ((res = iload->OpenChunk()) == IO_OK) {
        switch (iload->CurChunkID()) {
        case kChunkBones:
            while (iload->OpenChunk() == IO_OK) {
                wchar_t* name = nullptr;
                if (iload->CurChunkID() == kChunkBone && iload->ReadWStringChunk(&name) == IO_OK) {
                    c.bones.push_back(nullptr);  // resolved by name when first used
                    c.boneNames.push_back(name);
                }
                iload->CloseChunk();
            }
            break;
        case kChunkWeights: {
            uint32_t nv = 0, nw = 0;
            iload->Read(&nv, sizeof nv, &read);
            iload->Read(&nw, sizeof nw, &read);
            c.first.resize(nv + 1);
            c.bone.resize(nw);
            c.weight.resize(nw);
            iload->Read(c.first.data(), (ULONG)(c.first.size() * sizeof(uint32_t)), &read);
            iload->Read(c.bone.data(), (ULONG)(nw * sizeof(uint32_t)), &read);
            iload->Read(c.weight.data(), (ULONG)(nw * sizeof(float)), &read);
            break;
        }
        case kChunkSynced: {
            uint32_t ns = 0;
            iload->Read(&ns, sizeof ns, &read);
            c.synced.resize(ns);
            iload->Read(c.synced.data(), (ULONG)(ns * sizeof(uint32_t)), &read);
            c.inSync = true;
            break;
        }
        }
        iload->CloseChunk();
    }
    return res == IO_END ? IO_OK : res;
}

}  // namespace

ClassDesc2* unsubdivide_desc() { return &unsubdivideDesc; }

void unsubdivide_refresh_all() {
    for (UnsubdivideMod* mod : instances) mod->NotifyDependents(FOREVER, PART_ALL, REFMSG_CHANGE);
    GetCOREInterface()->RedrawViews(GetCOREInterface()->GetTime());
}
