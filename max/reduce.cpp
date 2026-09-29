// Exodus for 3ds Max: the reducing modifiers, Unsubdivide and Decimate. Both sit below Skin
// and keep only input vertices at their input positions; the work (plan, result, Skin weight
// copy) is in the core DLL (see maxcore.cpp for why Skin keeps its weights). This part holds the
// parameters and the per-node state, queues the Skin weight update for after the evaluation, and
// saves the weight copy with the file (bones by name, vertices by position).
#include <iparamm2.h>
#include <modstack.h>
#include <polyobj.h>
#include <Util/MainThreadTaskManager.h>

#include <set>

#include "plugin.h"
#include "resource.h"

namespace {

const Class_ID kUnsubdivideClassId(0x4a3f19c2, 0x7de20b61);
const Class_ID kDecimateClassId(0x19b4e6d3, 0x2c85a47f);

enum { kParams };
enum { kLevels, kSymmetry };        // Unsubdivide
enum { kPercent, kLockBorder };     // Decimate

// Save chunks of the per-node data.
enum : USHORT { kChunkBones = 0x200, kChunkBone, kChunkWeights, kChunkSynced };

std::set<Modifier*> instances;  // every reducing modifier alive
bool is_reducer(Modifier* m) { return instances.count(m) > 0; }

class ReduceData : public LocalModData {
public:
    ExodusReduceState state;
    LocalModData* Clone() override { return new ReduceData(*this); }
};

void post_weights(INode* node, Modifier* mod, ReduceData& data);

// What Unsubdivide and Decimate share.
class ReduceMod : public Modifier {
public:
    IParamBlock2* pblock = nullptr;
    MSTR status;

    ReduceMod() { instances.insert(this); }
    ~ReduceMod() override { instances.erase(this); }

    virtual ClassDesc2& Desc() const = 0;
    virtual ParamBlockDesc2& PbDesc() const = 0;
    virtual int NameId() const = 0;
    virtual const MCHAR* Name() const = 0;
    // One evaluation: read the parameters and call the core.
    virtual bool Run(const ExodusCoreApi& core, TimeValue t, Interval& valid, ModContext& mc, INode* node, MNMesh& mesh,
                     ExodusReduceState& state, std::wstring& text, INode*& owner) = 0;

    // Animatable
    void DeleteThis() override { delete this; }
    void GetClassName(MSTR& s, bool localized) const override { s = localized ? exodus_string(NameId()) : Name(); }
    Class_ID ClassID() override { return Desc().ClassID(); }
    int NumSubs() override { return 1; }
    Animatable* SubAnim(int) override { return pblock; }
    MSTR SubAnimName(int, bool localized) override { return localized ? exodus_string(IDS_PARAMETERS) : _M("Parameters"); }
    int NumParamBlocks() override { return 1; }
    IParamBlock2* GetParamBlock(int) override { return pblock; }
    IParamBlock2* GetParamBlockByID(BlockID id) override { return pblock && pblock->ID() == id ? pblock : nullptr; }
    void BeginEditParams(IObjParam* ip, ULONG flags, Animatable* prev) override { Desc().BeginEditParams(ip, this, flags, prev); }
    void EndEditParams(IObjParam* ip, ULONG flags, Animatable* next) override { Desc().EndEditParams(ip, this, flags, next); }

    // References
    int NumRefs() override { return 1; }
    RefTargetHandle GetReference(int) override { return pblock; }
    RefResult NotifyRefChanged(const Interval&, RefTargetHandle, PartID&, RefMessage, BOOL) override { return REF_SUCCEED; }
    RefTargetHandle Clone(RemapDir& remap) override {
        auto* mod = static_cast<ReduceMod*>(Desc().Create(FALSE));
        mod->ReplaceReference(0, remap.CloneRef(pblock));
        BaseClone(this, mod, remap);
        return mod;
    }

protected:
    void SetReference(int, RefTargetHandle target) override { pblock = static_cast<IParamBlock2*>(target); }

public:
    // BaseObject / Modifier
    const MCHAR* GetObjectName(bool localized) const override { return localized ? exodus_string(NameId()) : Name(); }
    CreateMouseCallBack* GetCreateMouseCallBack() override { return nullptr; }
    BOOL ChangeTopology() override { return TRUE; }
    ChannelMask ChannelsUsed() override { return GEOM_CHANNEL | TOPO_CHANNEL | TEXMAP_CHANNEL | VERTCOLOR_CHANNEL | SELECT_CHANNEL | SUBSEL_TYPE_CHANNEL; }
    ChannelMask ChannelsChanged() override { return GEOM_CHANNEL | TOPO_CHANNEL | TEXMAP_CHANNEL | VERTCOLOR_CHANNEL | SELECT_CHANNEL | SUBSEL_TYPE_CHANNEL; }
    Class_ID InputType() override { return polyObjectClassID; }
    Interval LocalValidity(TimeValue) override { return TestAFlag(A_MOD_BEING_EDITED) ? NEVER : FOREVER; }

    void ModifyObject(TimeValue t, ModContext& mc, ObjectState* os, INode* node) override {
        if (!os->obj->IsSubClassOf(polyObjectClassID)) return;
        PolyObject* po = static_cast<PolyObject*>(os->obj);
        Interval valid = FOREVER;
        auto* data = static_cast<ReduceData*>(mc.localData);
        if (!data) mc.localData = data = new ReduceData;
        if (const ExodusCoreApi* core = exodus_core()) {
            std::wstring text;
            INode* owner = nullptr;
            if (Run(*core, t, valid, mc, node, po->GetMesh(), data->state, text, owner) && !data->state.pending) post_weights(owner, this, *data);
            status = text.c_str();
        } else {
            status = exodus_core_error();
        }
        ShowStatus();
        po->UpdateValidity(GEOM_CHAN_NUM, valid);
        po->UpdateValidity(TOPO_CHAN_NUM, valid);
        po->UpdateValidity(TEXMAP_CHAN_NUM, valid);
        po->UpdateValidity(VERT_COLOR_CHAN_NUM, valid);
        po->UpdateValidity(SELECT_CHAN_NUM, valid);
    }

    void ShowStatus() {
        IParamMap2* map = Desc().GetParamMap(&PbDesc());
        if (map && map->GetParamBlock() == pblock && map->GetHWnd()) SetDlgItemText(map->GetHWnd(), IDC_STATUS, status.data());
    }

    IOResult SaveLocalData(ISave* isave, LocalModData* ld) override;
    IOResult LoadLocalData(ILoad* iload, LocalModData** pld) override;
};

// Shows a modifier's status line when its rollout opens.
class StatusDlgProc : public ParamMap2UserDlgProc {
public:
    INT_PTR DlgProc(TimeValue, IParamMap2* map, HWND, UINT msg, WPARAM, LPARAM) override {
        if (msg == WM_INITDIALOG)
            if (auto* mod = static_cast<ReduceMod*>(map->GetParamBlock()->GetOwner())) mod->ShowStatus();
        return FALSE;
    }
    void DeleteThis() override {}
} statusDlgProc;

template <class Mod, int kName>
class ReduceClassDesc : public ClassDesc2 {
public:
    ReduceClassDesc(Class_ID id, const MCHAR* name, const MCHAR* internal) : id(id), name(name), internal(internal) {}
    int IsPublic() override { return TRUE; }
    void* Create(BOOL) override { return new Mod; }
    const MCHAR* ClassName() override { return exodus_string(kName); }
    const MCHAR* NonLocalizedClassName() override { return name; }
    SClass_ID SuperClassID() override { return OSM_CLASS_ID; }
    Class_ID ClassID() override { return id; }
    const MCHAR* Category() override { return exodus_string(IDS_CATEGORY); }
    const MCHAR* InternalName() override { return internal; }
    HINSTANCE HInstance() override { return hInstance; }

private:
    Class_ID id;
    const MCHAR* name;
    const MCHAR* internal;
};

// ---- Unsubdivide ----------------------------------------------------------------------------------

class UnsubdivideMod : public ReduceMod {
public:
    UnsubdivideMod();
    ClassDesc2& Desc() const override;
    ParamBlockDesc2& PbDesc() const override;
    int NameId() const override { return IDS_UNSUBDIVIDE; }
    const MCHAR* Name() const override { return _M("Unsubdivide"); }
    bool Run(const ExodusCoreApi& core, TimeValue t, Interval& valid, ModContext& mc, INode* node, MNMesh& mesh, ExodusReduceState& state,
             std::wstring& text, INode*& owner) override {
        int levels = 1;
        BOOL symmetry = TRUE;
        pblock->GetValue(kLevels, t, levels, valid);
        pblock->GetValue(kSymmetry, t, symmetry, valid);
        return core.unsubdivide_modify(this, mc, node, mesh, ExodusUnsubdivide{levels, symmetry != FALSE}, state, is_reducer, text, owner);
    }
};

ReduceClassDesc<UnsubdivideMod, IDS_UNSUBDIVIDE> unsubdivideDesc(kUnsubdivideClassId, _M("Unsubdivide"), _M("exodusUnsubdivide"));

// clang-format off
ParamBlockDesc2 unsubdividePbDesc(kParams, _M("parameters"), IDS_PARAMETERS, &unsubdivideDesc, P_AUTO_CONSTRUCT + P_AUTO_UI, 0,
    IDD_UNSUBDIVIDE, IDS_PARAMETERS, 0, 0, &statusDlgProc,
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

UnsubdivideMod::UnsubdivideMod() { unsubdivideDesc.MakeAutoParamBlocks(this); }
ClassDesc2& UnsubdivideMod::Desc() const { return unsubdivideDesc; }
ParamBlockDesc2& UnsubdivideMod::PbDesc() const { return unsubdividePbDesc; }

// ---- Decimate -------------------------------------------------------------------------------------

class DecimateMod : public ReduceMod {
public:
    DecimateMod();
    ClassDesc2& Desc() const override;
    ParamBlockDesc2& PbDesc() const override;
    int NameId() const override { return IDS_DECIMATE; }
    const MCHAR* Name() const override { return _M("Decimate"); }
    bool Run(const ExodusCoreApi& core, TimeValue t, Interval& valid, ModContext& mc, INode* node, MNMesh& mesh, ExodusReduceState& state,
             std::wstring& text, INode*& owner) override {
        float percent = 50;
        BOOL lockBorder = FALSE;
        pblock->GetValue(kPercent, t, percent, valid);
        pblock->GetValue(kLockBorder, t, lockBorder, valid);
        return core.decimate_modify(this, mc, node, mesh, ExodusDecimate{percent, lockBorder != FALSE}, state, is_reducer, text, owner);
    }
};

ReduceClassDesc<DecimateMod, IDS_DECIMATE> decimateDesc(kDecimateClassId, _M("Decimate"), _M("exodusDecimate"));

// clang-format off
ParamBlockDesc2 decimatePbDesc(kParams, _M("parameters"), IDS_PARAMETERS, &decimateDesc, P_AUTO_CONSTRUCT + P_AUTO_UI, 0,
    IDD_DECIMATE, IDS_PARAMETERS, 0, 0, &statusDlgProc,
    kPercent, _M("percent"), TYPE_FLOAT, 0, IDS_PERCENT,
        p_default, 50.0f,
        p_range, 1.0f, 100.0f,
        p_ui, TYPE_SPINNER, EDITTYPE_FLOAT, IDC_PERCENT_EDIT, IDC_PERCENT_SPIN, 0.5f,
        p_end,
    kLockBorder, _M("lockBorder"), TYPE_BOOL, 0, IDS_LOCK_BORDER,
        p_default, FALSE,
        p_ui, TYPE_SINGLECHECKBOX, IDC_LOCK_BORDER,
        p_end,
    p_end);
// clang-format on

DecimateMod::DecimateMod() { decimateDesc.MakeAutoParamBlocks(this); }
ClassDesc2& DecimateMod::Desc() const { return decimateDesc; }
ParamBlockDesc2& DecimateMod::PbDesc() const { return decimatePbDesc; }

// ---- Skin weight update after the evaluation ------------------------------------------------------

// The state of every reducing modifier on a node, in stack order (top first), with its modifier.
std::vector<std::pair<Modifier*, ReduceData*>> reducers_on(INode* node) {
    std::vector<std::pair<Modifier*, ReduceData*>> found;
    for (Object* obj = node ? node->GetObjectRef() : nullptr; obj && obj->SuperClassID() == GEN_DERIVOB_CLASS_ID;
         obj = static_cast<IDerivedObject*>(obj)->GetObjRef()) {
        auto* d = static_cast<IDerivedObject*>(obj);
        for (int i = 0; i < d->NumModifiers(); ++i)
            if (is_reducer(d->GetModifier(i)))
                if (ModContext* mc = d->GetModContext(i); mc && mc->localData)
                    found.push_back({d->GetModifier(i), static_cast<ReduceData*>(mc->localData)});
    }
    return found;
}

class WriteWeightsTask : public MainThreadTask {
public:
    WriteWeightsTask(ULONG node, Modifier* mod) : node(node), mod(mod) {}
    void Execute() override {
        if (!is_reducer(mod)) return;
        INode* n = GetCOREInterface()->GetINodeByHandle(node);
        const auto reducers = reducers_on(n);
        ReduceData* data = nullptr;
        std::vector<ExodusSkinCopy*> below;  // copies of the reducers under this one
        for (const auto& [m, d] : reducers) {
            if (m == mod) data = d;
            else if (data && !d->state.skin.empty()) below.push_back(&d->state.skin);
        }
        if (!data) return;
        const ExodusCoreApi* core = exodus_core();
        const bool changed = core && core->write_weights(n, mod, data->state, below);  // `pending` holds off new posts meanwhile
        data->state.pending = false;
        if (changed) {
            mod->NotifyDependents(FOREVER, PART_DISPLAY, REFMSG_CHANGE);  // refresh the status line
            GetCOREInterface()->RedrawViews(GetCOREInterface()->GetTime());
        }
    }

private:
    ULONG node;
    Modifier* mod;
};

void post_weights(INode* node, Modifier* mod, ReduceData& data) {
    if (!node) return;
    data.state.pending = true;
    IMainThreadTaskManager::GetInstance()->PostTask(new WriteWeightsTask(node->GetHandle(), mod));
}

// ---- Save / load of the weight copy -----------------------------------------------------------------

IOResult ReduceMod::SaveLocalData(ISave* isave, LocalModData* ld) {
    auto* data = static_cast<ReduceData*>(ld);
    if (!data || data->state.skin.empty()) return IO_OK;
    const ExodusSkinCopy& c = data->state.skin;
    ULONG written;
    isave->BeginChunk(kChunkBones);
    for (size_t i = 0; i < c.bones.size(); ++i) {
        isave->BeginChunk(kChunkBone);
        isave->WriteWString(c.bones[i] ? c.bones[i]->GetName() : c.boneNames[i].c_str());
        isave->EndChunk();
    }
    isave->EndChunk();
    isave->BeginChunk(kChunkWeights);
    const uint32_t nv = (uint32_t)c.pos.size(), nw = (uint32_t)c.bone.size();
    isave->Write(&nv, sizeof nv, &written);
    isave->Write(&nw, sizeof nw, &written);
    isave->Write(reinterpret_cast<const float*>(c.pos.data()), (ULONG)(nv * sizeof(ExodusPos)), &written);
    isave->Write(c.first.data(), (ULONG)(c.first.size() * sizeof(uint32_t)), &written);
    isave->Write(c.bone.data(), (ULONG)(nw * sizeof(uint32_t)), &written);
    isave->Write(c.weight.data(), (ULONG)(nw * sizeof(float)), &written);
    isave->EndChunk();
    if (c.inSync && !c.synced.empty()) {
        isave->BeginChunk(kChunkSynced);
        const uint32_t ns = (uint32_t)c.synced.size();
        isave->Write(&ns, sizeof ns, &written);
        isave->Write(reinterpret_cast<const float*>(c.synced.data()), (ULONG)(ns * sizeof(ExodusPos)), &written);
        isave->EndChunk();
    }
    return IO_OK;
}

IOResult ReduceMod::LoadLocalData(ILoad* iload, LocalModData** pld) {
    auto* data = new ReduceData;
    *pld = data;
    ExodusSkinCopy& c = data->state.skin;
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
            c.pos.resize(nv);
            c.first.resize(nv + 1);
            c.bone.resize(nw);
            c.weight.resize(nw);
            iload->Read(reinterpret_cast<float*>(c.pos.data()), (ULONG)(nv * sizeof(ExodusPos)), &read);
            iload->Read(c.first.data(), (ULONG)(c.first.size() * sizeof(uint32_t)), &read);
            iload->Read(c.bone.data(), (ULONG)(nw * sizeof(uint32_t)), &read);
            iload->Read(c.weight.data(), (ULONG)(nw * sizeof(float)), &read);
            break;
        }
        case kChunkSynced: {
            uint32_t ns = 0;
            iload->Read(&ns, sizeof ns, &read);
            c.synced.resize(ns);
            iload->Read(reinterpret_cast<float*>(c.synced.data()), (ULONG)(ns * sizeof(ExodusPos)), &read);
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
ClassDesc2* decimate_desc() { return &decimateDesc; }

void reduce_refresh_all() {
    for (Modifier* mod : instances) mod->NotifyDependents(FOREVER, PART_ALL, REFMSG_CHANGE);
    GetCOREInterface()->RedrawViews(GetCOREInterface()->GetTime());
}
