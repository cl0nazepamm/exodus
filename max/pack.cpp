// udimpacker for 3ds Max: the UDIM Pack modifier. One instance goes on every part of a character
// (select them all, add the modifier once). The maps are given by hand per UDIM tile: pick a tile
// in the list, pick a file for each channel. Tiles with maps are packed into one atlas, the packed
// UVs go into the target map channel and Bake writes the atlas maps. The layout lives in the core
// DLL (see maxcore.cpp); this part holds the parameters and the rollout.
#include <iparamm2.h>
#include <modstack.h>
#include <polyobj.h>
#include <Util/MainThreadTaskManager.h>
#include <commdlg.h>  // after windows.h (from the Max headers)

#include <algorithm>
#include <filesystem>
#include <set>
#include <string>

#include "plugin.h"
#include "resource.h"

namespace {

const Class_ID kPackClassId(0x5c2e8a17, 0x0b94d3f6);

enum { kParams };
enum : ParamID {
    kSourceChannel = 0, kTargetChannel = 1, kSize = 2, kMaxSize = 3, kPadding = 4, kRotate = 5, kOutDir = 6, kPrefix = 7,
    kChannel1 = 10,  // .. 15: channel names
    kKind1 = 20,     // .. 25: channel types
    kMapTiles = 30, kMapChannels = 31, kMapFiles = 32,  // one row per map: tile, channel, file
};

class PackMod;
std::set<PackMod*> packInstances;
void post_refresh(PackMod* mod);

class PackMod : public Modifier {
public:
    IParamBlock2* pblock = nullptr;
    MSTR status;
    std::vector<UdimPackTile> tiles;  // from the last evaluation
    int selectedTile = 0;             // tile shown in the rollout's map rows
    std::vector<std::wstring> shownLines;  // the tile list as shown
    bool refreshPending = false;

    PackMod();
    ~PackMod() override {
        packInstances.erase(this);
        if (const UdimCoreApi* core = udim_core_loaded()) core->pack_forget(this);
    }

    ClassDesc2& Desc() const;
    ParamBlockDesc2& PbDesc() const;

    UdimPackSettings Settings(TimeValue t, Interval& valid) {
        UdimPackSettings s;
        BOOL rotate = TRUE;
        pblock->GetValue(kSourceChannel, t, s.sourceChannel, valid);
        pblock->GetValue(kTargetChannel, t, s.targetChannel, valid);
        pblock->GetValue(kSize, t, s.size, valid);
        pblock->GetValue(kMaxSize, t, s.maxSize, valid);
        pblock->GetValue(kPadding, t, s.padding, valid);
        pblock->GetValue(kRotate, t, rotate, valid);
        s.rotate = rotate != FALSE;
        auto str = [&](ParamID id, int i = 0) {
            const MCHAR* v = pblock->GetStr(id, t, i);
            return std::wstring(v ? v : L"");
        };
        for (int c = 0; c < kUdimPackChannels; ++c) {
            s.channel[c] = str(kChannel1 + c);
            pblock->GetValue(kKind1 + c, t, s.kind[c], valid);
        }
        const int n = std::min({pblock->Count(kMapTiles), pblock->Count(kMapChannels), pblock->Count(kMapFiles)});
        for (int i = 0; i < n; ++i) s.maps.push_back({pblock->GetInt(kMapTiles, t, i), pblock->GetInt(kMapChannels, t, i), str(kMapFiles, i)});
        s.outDir = str(kOutDir);
        s.prefix = str(kPrefix);
        return s;
    }

    int FindMap(int tile, int channel) {
        const int n = std::min({pblock->Count(kMapTiles), pblock->Count(kMapChannels), pblock->Count(kMapFiles)});
        for (int i = 0; i < n; ++i)
            if (pblock->GetInt(kMapTiles, 0, i) == tile && pblock->GetInt(kMapChannels, 0, i) == channel) return i;
        return -1;
    }

    // Sets (or with an empty file, removes) the map of one channel in one tile, as one undo step.
    void SetMap(int tile, int channel, const MCHAR* file) {
        const int row = FindMap(tile, channel);
        theHold.Begin();
        if (file && *file) {
            if (row >= 0) {
                pblock->SetValue(kMapFiles, 0, file, row);
            } else {
                int t = tile, c = channel;
                const MCHAR* f = file;
                pblock->Append(kMapTiles, 1, &t);
                pblock->Append(kMapChannels, 1, &c);
                pblock->Append(kMapFiles, 1, &f);
            }
        } else if (row >= 0) {
            pblock->Delete(kMapFiles, row, 1);
            pblock->Delete(kMapChannels, row, 1);
            pblock->Delete(kMapTiles, row, 1);
        }
        theHold.Accept(udim_string(IDS_ASSIGN_MAP));
    }

    // Bake: the atlas maps into the output folder. Returns the packer's report.
    MSTR Bake() {
        const UdimCoreApi* core = udim_core();
        if (!core) return status = udim_core_error();
        Interval valid = FOREVER;
        const UdimPackSettings s = Settings(GetCOREInterface()->GetTime(), valid);
        std::string log;
        UdimSink sink{[](void* ctx, const char* text, size_t n) { static_cast<std::string*>(ctx)->append(text, n); }, &log};
        std::wstring text;
        bool relayout = false;
        HCURSOR cursor = SetCursor(LoadCursor(nullptr, IDC_WAIT));
        core->pack_bake(this, s, sink, text, relayout);
        SetCursor(cursor);
        status = text.c_str();
        if (relayout) NotifyDependents(FOREVER, PART_TEXMAP, REFMSG_CHANGE);
        UpdateUI();
        return MSTR::FromUTF8(log.c_str());
    }

    IParamMap2* Map() {
        IParamMap2* map = Desc().GetParamMap(&PbDesc());
        return map && map->GetParamBlock() == pblock && map->GetHWnd() ? map : nullptr;
    }

    void UpdateUI();

    // Animatable
    void DeleteThis() override { delete this; }
    void GetClassName(MSTR& s, bool localized) const override { s = localized ? udim_string(IDS_UDIMPACK) : _M("UDIM Pack"); }
    Class_ID ClassID() override { return kPackClassId; }
    int NumSubs() override { return 1; }
    Animatable* SubAnim(int) override { return pblock; }
    MSTR SubAnimName(int, bool localized) override { return localized ? udim_string(IDS_PARAMETERS) : _M("Parameters"); }
    int NumParamBlocks() override { return 1; }
    IParamBlock2* GetParamBlock(int) override { return pblock; }
    IParamBlock2* GetParamBlockByID(BlockID id) override { return pblock && pblock->ID() == id ? pblock : nullptr; }
    void BeginEditParams(IObjParam* ip, ULONG flags, Animatable* prev) override { Desc().BeginEditParams(ip, this, flags, prev); }
    void EndEditParams(IObjParam* ip, ULONG flags, Animatable* next) override { Desc().EndEditParams(ip, this, flags, next); }

    // References
    int NumRefs() override { return 1; }
    RefTargetHandle GetReference(int) override { return pblock; }
    RefResult NotifyRefChanged(const Interval&, RefTargetHandle target, PartID&, RefMessage msg, BOOL) override {
        if (target == pblock && msg == REFMSG_CHANGE) UpdateUI();  // maps set from the rollout, a script or undo
        return REF_SUCCEED;
    }
    RefTargetHandle Clone(RemapDir& remap) override {
        auto* mod = new PackMod;
        mod->ReplaceReference(0, remap.CloneRef(pblock));
        BaseClone(this, mod, remap);
        return mod;
    }

protected:
    void SetReference(int, RefTargetHandle target) override { pblock = static_cast<IParamBlock2*>(target); }

public:
    // BaseObject / Modifier
    const MCHAR* GetObjectName(bool localized) const override { return localized ? udim_string(IDS_UDIMPACK) : _M("UDIM Pack"); }
    CreateMouseCallBack* GetCreateMouseCallBack() override { return nullptr; }
    BOOL ChangeTopology() override { return FALSE; }
    ChannelMask ChannelsUsed() override { return TOPO_CHANNEL | TEXMAP_CHANNEL; }
    ChannelMask ChannelsChanged() override { return TEXMAP_CHANNEL; }
    Class_ID InputType() override { return polyObjectClassID; }
    Interval LocalValidity(TimeValue) override { return FOREVER; }

    void ModifyObject(TimeValue t, ModContext& mc, ObjectState* os, INode* node) override {
        if (!os->obj->IsSubClassOf(polyObjectClassID)) return;
        auto* po = static_cast<PolyObject*>(os->obj);
        Interval valid = FOREVER;
        const UdimPackSettings s = Settings(t, valid);
        if (const UdimCoreApi* core = udim_core()) {
            std::wstring text;
            if (core->pack_modify(this, mc, node, po->GetMesh(), s, text, tiles)) post_refresh(this);
            status = text.c_str();
        } else {
            status = udim_core_error();
        }
        UpdateUI();
        po->UpdateValidity(TEXMAP_CHAN_NUM, valid);
    }
};

// After an evaluation changed the layout: the other parts evaluate again.
class RefreshTask : public MainThreadTask {
public:
    explicit RefreshTask(PackMod* mod) : mod(mod) {}
    void Execute() override {
        if (!packInstances.count(mod)) return;
        mod->refreshPending = false;
        mod->NotifyDependents(FOREVER, PART_TEXMAP, REFMSG_CHANGE);
        GetCOREInterface()->RedrawViews(GetCOREInterface()->GetTime());
    }

private:
    PackMod* mod;
};

void post_refresh(PackMod* mod) {
    if (mod->refreshPending) return;
    mod->refreshPending = true;
    IMainThreadTaskManager::GetInstance()->PostTask(new RefreshTask(mod));
}

std::wstring file_name(const std::wstring& path) { return std::filesystem::path(path).filename().wstring(); }

void PackMod::UpdateUI() {
    IParamMap2* map = Map();
    if (!map) return;
    HWND hw = map->GetHWnd();
    SetDlgItemText(hw, IDC_STATUS, status.data());

    // Tiles: the ones in the UVs and the ones with maps.
    std::vector<int> list;
    for (const UdimPackTile& t : tiles) list.push_back(t.tile);
    for (int i = 0; i < pblock->Count(kMapTiles); ++i) list.push_back(pblock->GetInt(kMapTiles, 0, i));
    std::sort(list.begin(), list.end());
    list.erase(std::unique(list.begin(), list.end()), list.end());
    if (!list.empty() && std::find(list.begin(), list.end(), selectedTile) == list.end()) selectedTile = list.front();

    std::vector<std::wstring> lines;
    for (int tile : list) {
        int faces = 0, maps = 0;
        for (const UdimPackTile& t : tiles)
            if (t.tile == tile) faces = t.faces;
        for (int c = 0; c < kUdimPackChannels; ++c) maps += FindMap(tile, c) >= 0;
        wchar_t line[128];
        swprintf_s(line, L"%d   %d faces   %s", tile, faces,
                   maps ? (std::to_wstring(maps) + (maps == 1 ? L" map" : L" maps")).c_str() : L"not packed");
        lines.push_back(line);
    }
    HWND box = GetDlgItem(hw, IDC_TILES);
    if (lines != shownLines) {  // evaluations are frequent: rebuild the list only when it changed
        const int top = (int)SendMessage(box, LB_GETTOPINDEX, 0, 0);
        SendMessage(box, WM_SETREDRAW, FALSE, 0);
        SendMessage(box, LB_RESETCONTENT, 0, 0);
        for (size_t i = 0; i < lines.size(); ++i) {
            const int item = (int)SendMessage(box, LB_ADDSTRING, 0, (LPARAM)lines[i].c_str());
            SendMessage(box, LB_SETITEMDATA, item, list[i]);
        }
        SendMessage(box, LB_SETTOPINDEX, top, 0);
        SendMessage(box, WM_SETREDRAW, TRUE, 0);
        InvalidateRect(box, nullptr, TRUE);
        shownLines = lines;
    }
    for (size_t i = 0; i < list.size(); ++i)
        if (list[i] == selectedTile && (int)SendMessage(box, LB_GETCURSEL, 0, 0) != (int)i) SendMessage(box, LB_SETCURSEL, i, 0);

    // The selected tile's map per channel.
    SetDlgItemText(hw, IDC_TILE_LABEL, list.empty() ? L"" : (L"Maps of tile " + std::to_wstring(selectedTile) + L":").c_str());
    for (int c = 0; c < kUdimPackChannels; ++c) {
        const MCHAR* name = pblock->GetStr(kChannel1 + c, 0);
        const bool used = name && *name && !list.empty();
        const int row = FindMap(selectedTile, c);
        const MCHAR* file = row >= 0 ? pblock->GetStr(kMapFiles, 0, row) : nullptr;
        SetDlgItemText(hw, IDC_FILE_LABEL1 + c, name ? name : L"");
        SetDlgItemText(hw, IDC_FILE1 + c, !used ? L"" : file && *file ? file_name(file).c_str() : L"(none)");
        EnableWindow(GetDlgItem(hw, IDC_FILE1 + c), used);
        EnableWindow(GetDlgItem(hw, IDC_CLEAR1 + c), used && row >= 0);
    }
}

std::wstring pick_image(HWND owner, const std::wstring& current) {
    static std::wstring lastDir;
    wchar_t file[MAX_PATH] = L"";
    std::wstring dir = current.empty() ? lastDir : std::filesystem::path(current).parent_path().wstring();
    OPENFILENAMEW ofn = {sizeof ofn};
    ofn.hwndOwner = owner;
    ofn.lpstrFilter = L"Images (png, tga, jpg, bmp, psd)\0*.png;*.tga;*.jpg;*.jpeg;*.bmp;*.psd\0All files\0*.*\0";
    ofn.lpstrFile = file;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrInitialDir = dir.empty() ? nullptr : dir.c_str();
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY | OFN_NOCHANGEDIR;
    if (!GetOpenFileNameW(&ofn)) return L"";
    lastDir = std::filesystem::path(file).parent_path().wstring();
    return file;
}

class PackDlgProc : public ParamMap2UserDlgProc {
public:
    INT_PTR DlgProc(TimeValue, IParamMap2* map, HWND hw, UINT msg, WPARAM wp, LPARAM) override {
        auto* mod = static_cast<PackMod*>(map->GetParamBlock()->GetOwner());
        if (!mod) return FALSE;
        switch (msg) {
        case WM_INITDIALOG:
            mod->shownLines.clear();
            mod->UpdateUI();
            return TRUE;
        case WM_COMMAND: {
            const int id = LOWORD(wp), code = HIWORD(wp);
            if (id == IDC_TILES && code == LBN_SELCHANGE) {
                HWND box = GetDlgItem(hw, IDC_TILES);
                const int item = (int)SendMessage(box, LB_GETCURSEL, 0, 0);
                if (item >= 0) mod->selectedTile = (int)SendMessage(box, LB_GETITEMDATA, item, 0);
                mod->UpdateUI();
                return TRUE;
            }
            if (code != BN_CLICKED) return FALSE;
            if (id >= IDC_FILE1 && id < IDC_FILE1 + kUdimPackChannels) {
                const int c = id - IDC_FILE1, row = mod->FindMap(mod->selectedTile, c);
                const MCHAR* current = row >= 0 ? mod->pblock->GetStr(kMapFiles, 0, row) : nullptr;
                const std::wstring file = pick_image(GetCOREInterface()->GetMAXHWnd(), current ? current : L"");
                if (!file.empty()) mod->SetMap(mod->selectedTile, c, file.c_str());
                return TRUE;
            }
            if (id >= IDC_CLEAR1 && id < IDC_CLEAR1 + kUdimPackChannels) {
                mod->SetMap(mod->selectedTile, id - IDC_CLEAR1, nullptr);
                return TRUE;
            }
            if (id == IDC_OUT_DIR_BROWSE) {
                MCHAR dir[MAX_PATH] = L"";
                GetCOREInterface()->ChooseDirectory(GetCOREInterface()->GetMAXHWnd(), udim_string(IDS_CHOOSE_OUT_DIR), dir);
                if (*dir) {
                    theHold.Begin();
                    mod->pblock->SetValue(kOutDir, 0, dir);
                    theHold.Accept(udim_string(IDS_OUT_DIR));
                }
                return TRUE;
            }
            if (id == IDC_BAKE) {
                mod->Bake();
                return TRUE;
            }
            return FALSE;
        }
        }
        return FALSE;
    }
    void DeleteThis() override {}
} packDlgProc;

class PackClassDesc : public ClassDesc2 {
public:
    int IsPublic() override { return TRUE; }
    void* Create(BOOL) override { return new PackMod; }
    const MCHAR* ClassName() override { return udim_string(IDS_UDIMPACK); }
    const MCHAR* NonLocalizedClassName() override { return _M("UDIM Pack"); }
    SClass_ID SuperClassID() override { return OSM_CLASS_ID; }
    Class_ID ClassID() override { return kPackClassId; }
    const MCHAR* Category() override { return udim_string(IDS_CATEGORY); }
    const MCHAR* InternalName() override { return _M("udimPack"); }
    HINSTANCE HInstance() override { return hInstance; }
} packDesc;

// Channel c (0-based) is `channel<n>` and `channelType<n>` in MAXScript, n = c + 1.
#define UDIM_CHANNEL_PARAMS(c, n, name)                                                                \
    kChannel1 + c, _M("channel" #n), TYPE_STRING, 0, IDS_CHANNEL,                                     \
        p_default, name,                                                                               \
        p_ui, TYPE_EDITBOX, IDC_CHANNEL1 + c,                                                          \
        p_end,                                                                                         \
    kKind1 + c, _M("channelType" #n), TYPE_INT, 0, IDS_KIND,                                          \
        p_default, 0,                                                                                  \
        p_range, 0, 2,                                                                                 \
        p_ui, TYPE_INTLISTBOX, IDC_KIND1 + c, 3, IDS_KIND_COLOR, IDS_KIND_NORMAL_GL, IDS_KIND_NORMAL_DX, \
        p_end,

// clang-format off
ParamBlockDesc2 packPbDesc(kParams, _M("parameters"), IDS_PARAMETERS, &packDesc, P_AUTO_CONSTRUCT + P_AUTO_UI, 0,
    IDD_UDIMPACK, IDS_PARAMETERS, 0, 0, &packDlgProc,
    kSourceChannel, _M("sourceChannel"), TYPE_INT, 0, IDS_SOURCE_CHANNEL,
        p_default, 1,
        p_range, 1, 99,
        p_ui, TYPE_SPINNER, EDITTYPE_INT, IDC_SOURCE_EDIT, IDC_SOURCE_SPIN, 1.0f,
        p_end,
    kTargetChannel, _M("targetChannel"), TYPE_INT, 0, IDS_TARGET_CHANNEL,
        p_default, 2,
        p_range, 1, 99,
        p_ui, TYPE_SPINNER, EDITTYPE_INT, IDC_TARGET_EDIT, IDC_TARGET_SPIN, 1.0f,
        p_end,
    kSize, _M("size"), TYPE_INT, 0, IDS_SIZE,
        p_default, 0,
        p_range, 0, 16384,
        p_ui, TYPE_SPINNER, EDITTYPE_INT, IDC_SIZE_EDIT, IDC_SIZE_SPIN, 256.0f,
        p_end,
    kMaxSize, _M("maxSize"), TYPE_INT, 0, IDS_MAX_SIZE,
        p_default, 8192,
        p_range, 256, 16384,
        p_ui, TYPE_SPINNER, EDITTYPE_INT, IDC_MAXSIZE_EDIT, IDC_MAXSIZE_SPIN, 256.0f,
        p_end,
    kPadding, _M("padding"), TYPE_INT, 0, IDS_PADDING,
        p_default, 8,
        p_range, 0, 64,
        p_ui, TYPE_SPINNER, EDITTYPE_INT, IDC_PADDING_EDIT, IDC_PADDING_SPIN, 1.0f,
        p_end,
    kRotate, _M("rotate"), TYPE_BOOL, 0, IDS_ROTATE,
        p_default, TRUE,
        p_ui, TYPE_SINGLECHECKBOX, IDC_ROTATE,
        p_end,
    kOutDir, _M("outputFolder"), TYPE_STRING, 0, IDS_OUT_DIR,
        p_default, _M(""),
        p_ui, TYPE_EDITBOX, IDC_OUT_DIR,
        p_end,
    kPrefix, _M("prefix"), TYPE_STRING, 0, IDS_PREFIX,
        p_default, _M("atlas"),
        p_ui, TYPE_EDITBOX, IDC_PREFIX,
        p_end,
    UDIM_CHANNEL_PARAMS(0, 1, _M("BaseColor"))
    UDIM_CHANNEL_PARAMS(1, 2, _M(""))
    UDIM_CHANNEL_PARAMS(2, 3, _M(""))
    UDIM_CHANNEL_PARAMS(3, 4, _M(""))
    UDIM_CHANNEL_PARAMS(4, 5, _M(""))
    UDIM_CHANNEL_PARAMS(5, 6, _M(""))
    kMapTiles, _M("mapTiles"), TYPE_INT_TAB, 0, P_VARIABLE_SIZE, IDS_MAP_TILES,
        p_end,
    kMapChannels, _M("mapChannels"), TYPE_INT_TAB, 0, P_VARIABLE_SIZE, IDS_MAP_CHANNELS,
        p_end,
    kMapFiles, _M("mapFiles"), TYPE_STRING_TAB, 0, P_VARIABLE_SIZE, IDS_MAP_FILES,
        p_end,
    p_end);
// clang-format on

PackMod::PackMod() {
    packInstances.insert(this);
    packDesc.MakeAutoParamBlocks(this);
}
ClassDesc2& PackMod::Desc() const { return packDesc; }
ParamBlockDesc2& PackMod::PbDesc() const { return packPbDesc; }

}  // namespace

ClassDesc2* pack_desc() { return &packDesc; }

void pack_refresh_all() {
    for (PackMod* mod : packInstances) mod->NotifyDependents(FOREVER, PART_ALL, REFMSG_CHANGE);
    GetCOREInterface()->RedrawViews(GetCOREInterface()->GetTime());
}

MSTR pack_bake(ReferenceTarget* target) {
    auto* mod = dynamic_cast<PackMod*>(target);
    if (!mod || !packInstances.count(mod)) return _M("not a UDIM Pack modifier");
    MSTR log = mod->Bake();
    return log + _M("\n") + mod->status;
}
