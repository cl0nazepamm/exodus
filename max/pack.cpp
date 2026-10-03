// Exodus for 3ds Max: the UDIM Pack modifier. One instance goes on every part of a character
// (select them all, add the modifier once). The maps are given by hand per UDIM tile, for all
// objects or for one object: pick an object (or all), a tile in the list, a file for each channel.
// An object's own maps in a tile replace the ones for all objects there, so objects that each use
// tile 1001 with their own textures pack side by side. Tiles with maps go into one atlas, the packed
// UVs go into the target map channel and Bake writes the atlas maps. The layout lives in the core
// DLL (see maxcore.cpp); this part holds the parameters and the rollout.
#include <iparamm2.h>
#include <modstack.h>
#include <notify.h>
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

const Class_ID kPackClassId(0xdcdf3ee7, 0x9ad18e13);

enum { kParams };
enum : ParamID {
    kSourceChannel = 0, kTargetChannel = 1, kSize = 2, kMaxSize = 3, kPadding = 4, kRotate = 5, kOutDir = 6, kPrefix = 7,
    kChannel1 = 10,  // .. 15: channel names
    kKind1 = 20,     // .. 25: channel types
    // One row per map: tile, channel (1-based), file and object (node name, empty = all objects;
    // missing rows at the end, as scripts written for the first three leave them, are empty).
    kMapTiles = 30, kMapChannels = 31, kMapFiles = 32, kMapObjects = 33,
    kCollapsed = 34,  // names of the objects whose stack was collapsed with the modifier: they keep their place
};

class PackMod;
std::set<PackMod*> packInstances;
void post_refresh(PackMod* mod);

// The names of the objects the modifier is on, sorted (deleted ones, kept for undo, left out).
std::vector<std::wstring> object_names(ReferenceTarget* mod) {
    struct Nodes : DependentEnumProc {
        std::vector<std::wstring> names;
        int proc(ReferenceMaker* rm) override {
            if (rm->SuperClassID() != BASENODE_CLASS_ID) return DEP_ENUM_CONTINUE;
            auto* n = static_cast<INode*>(rm);
            if (n->GetParentNode()) names.push_back(n->GetName() ? n->GetName() : L"");
            return DEP_ENUM_SKIP;
        }
    } nodes;
    mod->DoEnumDependents(&nodes);
    std::sort(nodes.names.begin(), nodes.names.end());
    nodes.names.erase(std::unique(nodes.names.begin(), nodes.names.end()), nodes.names.end());
    return nodes.names;
}

class PackMod : public Modifier {
public:
    IParamBlock2* pblock = nullptr;
    MSTR status;
    std::vector<ExodusPackTile> tiles;  // from the last evaluation
    std::wstring selectedObject;      // object shown in the rollout's tile list; empty = all objects
    int selectedTile = 0;             // tile shown in the rollout's map rows
    std::vector<std::wstring> shownObjects, shownLines;  // the object and tile lists as shown
    std::vector<std::wstring> objectList;  // per object list item: the name (item 0, all objects: empty)
    bool refreshPending = false;

    PackMod();
    ~PackMod() override {
        packInstances.erase(this);
        if (const ExodusCoreApi* core = exodus_core_loaded()) core->pack_forget(this);
    }

    ClassDesc2& Desc() const;
    ParamBlockDesc2& PbDesc() const;

    ExodusPackSettings Settings(TimeValue t, Interval& valid) {
        ExodusPackSettings s;
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
        for (int c = 0; c < kExodusPackChannels; ++c) {
            s.channel[c] = str(kChannel1 + c);
            pblock->GetValue(kKind1 + c, t, s.kind[c], valid);
        }
        for (int i = 0; i < Rows(); ++i)
            s.maps.push_back({MapObject(i), pblock->GetInt(kMapTiles, t, i), pblock->GetInt(kMapChannels, t, i) - 1, str(kMapFiles, i)});
        for (int i = 0; i < pblock->Count(kCollapsed); ++i)
            if (const std::wstring name = Collapsed(i); !name.empty()) s.collapsed.push_back(name);
        s.outDir = str(kOutDir);
        s.prefix = str(kPrefix);
        return s;
    }

    int Rows() { return std::min({pblock->Count(kMapTiles), pblock->Count(kMapChannels), pblock->Count(kMapFiles)}); }

    std::wstring MapObject(int row) {
        const MCHAR* v = row < pblock->Count(kMapObjects) ? pblock->GetStr(kMapObjects, 0, row) : nullptr;
        return v ? v : L"";
    }

    std::wstring Collapsed(int i) {
        const MCHAR* v = pblock->GetStr(kCollapsed, 0, i);
        return v ? v : L"";
    }

    std::wstring MapFile(int row) {
        const MCHAR* v = row >= 0 ? pblock->GetStr(kMapFiles, 0, row) : nullptr;
        return v ? v : L"";
    }

    int FindMap(const std::wstring& object, int tile, int channel) {
        for (int i = 0; i < Rows(); ++i)
            if (pblock->GetInt(kMapTiles, 0, i) == tile && pblock->GetInt(kMapChannels, 0, i) == channel + 1 && MapObject(i) == object) return i;
        return -1;
    }

    // The maps of an object (empty: all objects) in a tile that are packed: named channel, file set.
    int CountMaps(const std::wstring& object, int tile) {
        int n = 0;
        for (int c = 0; c < kExodusPackChannels; ++c) {
            const MCHAR* name = pblock->GetStr(kChannel1 + c, 0);
            n += name && *name && !MapFile(FindMap(object, tile, c)).empty();
        }
        return n;
    }

    void AppendMap(const std::wstring& object, int tile, int channel, const std::wstring& file) {
        int t = tile, c = channel + 1;  // 1-based, as channel1 .. channel6
        const MCHAR* f = file.c_str();
        const MCHAR* o = object.c_str();
        pblock->Append(kMapTiles, 1, &t);
        pblock->Append(kMapChannels, 1, &c);
        pblock->Append(kMapFiles, 1, &f);
        pblock->Append(kMapObjects, 1, &o);
    }

    // Sets (or with an empty file, removes) the map of one channel in one tile for an object (empty:
    // all objects), as one undo step. An object's first own map in a tile starts from the maps for
    // all objects there, which its own replace.
    void SetMap(const std::wstring& object, int tile, int channel, const MCHAR* file) {
        theHold.Begin();
        // The rows as one table: the object column as long as the others.
        const int rows = Rows();
        for (ParamID id : {kMapTiles, kMapChannels, kMapFiles})
            if (pblock->Count(id) > rows) pblock->Delete(id, rows, pblock->Count(id) - rows);
        if (pblock->Count(kMapObjects) > rows) pblock->Delete(kMapObjects, rows, pblock->Count(kMapObjects) - rows);
        for (const MCHAR* none = L""; pblock->Count(kMapObjects) < rows;) pblock->Append(kMapObjects, 1, &none);

        const int row = FindMap(object, tile, channel);
        if (file && *file) {
            if (!object.empty() && CountMaps(object, tile) == 0)
                for (int c = 0; c < kExodusPackChannels; ++c)
                    if (const std::wstring shared = MapFile(FindMap(L"", tile, c)); c != channel && !shared.empty() && FindMap(object, tile, c) < 0)
                        AppendMap(object, tile, c, shared);
            if (row >= 0) pblock->SetValue(kMapFiles, 0, file, row);
            else AppendMap(object, tile, channel, file);
        } else if (row >= 0) {
            for (ParamID id : {kMapObjects, kMapFiles, kMapChannels, kMapTiles}) pblock->Delete(id, row, 1);
        }
        theHold.Accept(exodus_string(IDS_ASSIGN_MAP));
    }

    // A node was renamed. If it is one of this modifier's objects (or a collapsed one), its maps
    // follow the new name (unless another object still has the old one) and the tile list is made again.
    void RenameObject(const std::wstring& from, const std::wstring& to) {
        bool ours = false;
        if (!GetCOREInterface()->GetINodeByName(from.c_str()))
            for (int i = 0; i < pblock->Count(kCollapsed); ++i)
                if (Collapsed(i) == from) {
                    pblock->SetValue(kCollapsed, 0, to.c_str(), i);
                    ours = true;
                }
        const std::vector<std::wstring> names = object_names(this);
        if (!ours && !std::binary_search(names.begin(), names.end(), to)) return;
        if (!std::binary_search(names.begin(), names.end(), from)) {
            for (int i = 0; i < pblock->Count(kMapObjects); ++i)
                if (MapObject(i) == from) pblock->SetValue(kMapObjects, 0, to.c_str(), i);
            if (selectedObject == from) selectedObject = to;
        }
        NotifyDependents(FOREVER, PART_TEXMAP, REFMSG_CHANGE);
    }

    // Bake: the atlas maps into the output folder. Returns the packer's report.
    MSTR Bake() {
        const ExodusCoreApi* core = exodus_core();
        if (!core) return status = exodus_core_error();
        Interval valid = FOREVER;
        const ExodusPackSettings s = Settings(GetCOREInterface()->GetTime(), valid);
        std::string log;
        ExodusSink sink{[](void* ctx, const char* text, size_t n) { static_cast<std::string*>(ctx)->append(text, n); }, &log};
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

    // Refreshes the rollout (`hw` while it opens: Max registers the map only afterwards).
    void UpdateUI(HWND hw = nullptr);

    // Animatable
    void DeleteThis() override { delete this; }
    void GetClassName(MSTR& s, bool localized) const override { s = localized ? exodus_string(IDS_UDIMPACK) : _M("UDIM Pack"); }
    Class_ID ClassID() override { return kPackClassId; }
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
    const MCHAR* GetObjectName(bool localized) const override { return localized ? exodus_string(IDS_UDIMPACK) : _M("UDIM Pack"); }
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
        const ExodusPackSettings s = Settings(t, valid);
        if (const ExodusCoreApi* core = exodus_core()) {
            std::wstring text;
            if (core->pack_modify(this, mc, node, po->GetMesh(), s, text, tiles)) post_refresh(this);
            status = text.c_str();
        } else {
            status = exodus_core_error();
        }
        UpdateUI();
        po->UpdateValidity(TEXMAP_CHAN_NUM, valid);
    }

    // The node's stack is collapsed with this modifier in it: its packed UVs become part of its
    // mesh, so it keeps its place in the layout and the other objects do not move.
    void NotifyPreCollapse(INode* node, IDerivedObject*, int) override {
        const std::wstring name = node && node->GetName() ? node->GetName() : L"";
        if (name.empty()) return;
        for (int i = 0; i < pblock->Count(kCollapsed); ++i)
            if (Collapsed(i) == name) return;
        const MCHAR* n = name.c_str();
        pblock->Append(kCollapsed, 1, &n);
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

void PackMod::UpdateUI(HWND hw) {
    if (!hw) {
        IParamMap2* map = Map();
        if (!map) return;
        hw = map->GetHWnd();
    }
    SetDlgItemText(hw, IDC_STATUS, status.data());

    // Objects: the ones the modifier is on, the collapsed ones, then the ones with maps that are
    // neither (removed, or renamed by a script).
    const std::vector<std::wstring> present = object_names(this);
    std::vector<std::wstring> collapsed, missing;
    for (int i = 0; i < pblock->Count(kCollapsed); ++i)
        if (const std::wstring o = Collapsed(i);
            !o.empty() && !std::binary_search(present.begin(), present.end(), o) && GetCOREInterface()->GetINodeByName(o.c_str()))
            collapsed.push_back(o);
    std::sort(collapsed.begin(), collapsed.end());
    collapsed.erase(std::unique(collapsed.begin(), collapsed.end()), collapsed.end());
    for (int i = 0; i < Rows(); ++i)
        if (const std::wstring o = MapObject(i); !o.empty() && !std::binary_search(present.begin(), present.end(), o) &&
                                                 !std::binary_search(collapsed.begin(), collapsed.end(), o))
            missing.push_back(o);
    std::sort(missing.begin(), missing.end());
    missing.erase(std::unique(missing.begin(), missing.end()), missing.end());
    std::vector<std::wstring> items{L"All objects"};
    objectList = {L""};
    for (const std::wstring& o : present) {
        items.push_back(o);
        objectList.push_back(o);
    }
    for (const std::wstring& o : collapsed) {
        items.push_back(o + L" (collapsed)");
        objectList.push_back(o);
    }
    for (const std::wstring& o : missing) {
        items.push_back(o + L" (not found)");
        objectList.push_back(o);
    }
    auto selected = std::find(objectList.begin(), objectList.end(), selectedObject);
    if (selected == objectList.end()) selectedObject.clear(), selected = objectList.begin();
    HWND objects = GetDlgItem(hw, IDC_OBJECT);
    if (items != shownObjects) {
        SendMessage(objects, CB_RESETCONTENT, 0, 0);
        for (const std::wstring& item : items) SendMessage(objects, CB_ADDSTRING, 0, (LPARAM)item.c_str());
        shownObjects = items;
    }
    const int objectItem = (int)(selected - objectList.begin());
    if ((int)SendMessage(objects, CB_GETCURSEL, 0, 0) != objectItem) SendMessage(objects, CB_SETCURSEL, objectItem, 0);

    // Tiles of the selected object (all objects: of any): the ones in the UVs and the ones with maps.
    const std::wstring& object = selectedObject;
    std::vector<int> list;
    for (const ExodusPackTile& t : tiles)
        if (object.empty() || t.object == object) list.push_back(t.tile);
    for (int i = 0; i < Rows(); ++i)
        if (object.empty() || MapObject(i) == object) list.push_back(pblock->GetInt(kMapTiles, 0, i));
    std::sort(list.begin(), list.end());
    list.erase(std::unique(list.begin(), list.end()), list.end());
    if (!list.empty() && std::find(list.begin(), list.end(), selectedTile) == list.end()) selectedTile = list.front();

    auto n_maps = [](int n) { return std::to_wstring(n) + (n == 1 ? L" map" : L" maps"); };
    std::vector<std::wstring> lines;
    for (int tile : list) {
        int faces = 0, users = 0;
        for (const ExodusPackTile& t : tiles)
            if (t.tile == tile && (object.empty() || t.object == object)) faces += t.faces, ++users;
        const int shared = CountMaps(L"", tile);
        std::wstring maps;
        if (object.empty()) {
            bool own = false;
            for (int i = 0; i < Rows() && !own; ++i) own = pblock->GetInt(kMapTiles, 0, i) == tile && !MapObject(i).empty();
            maps = shared ? n_maps(shared) : own ? L"per object" : L"not packed";
        } else {
            const int own = CountMaps(object, tile);
            maps = own ? n_maps(own) : shared ? n_maps(shared) + L" (all)" : L"not packed";
        }
        // Several objects in one tile: their count (they share the tile's maps unless they have their own).
        wchar_t line[256];
        swprintf_s(line, L"%d   %s   %s", tile, (users > 1 ? std::to_wstring(users) + L" objects" : std::to_wstring(faces) + L" faces").c_str(),
                   maps.c_str());
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

    // The selected tile's map per channel: the object's own, or while it has none there the ones
    // for all objects ("all: ").
    SetDlgItemText(hw, IDC_TILE_LABEL,
                   list.empty() ? L"" : ((object.empty() ? L"All objects" : object) + L", tile " + std::to_wstring(selectedTile) + L":").c_str());
    const bool inherits = !object.empty() && CountMaps(object, selectedTile) == 0;
    for (int c = 0; c < kExodusPackChannels; ++c) {
        const MCHAR* name = pblock->GetStr(kChannel1 + c, 0);
        const bool used = name && *name && !list.empty();
        const int row = FindMap(object, selectedTile, c);
        const std::wstring file = MapFile(row), shared = inherits ? MapFile(FindMap(L"", selectedTile, c)) : L"";
        SetDlgItemText(hw, IDC_FILE_LABEL1 + c, name ? name : L"");
        SetDlgItemText(hw, IDC_FILE1 + c,
                       (!used ? L"" : !file.empty() ? file_name(file) : !shared.empty() ? L"all: " + file_name(shared) : L"(none)").c_str());
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
            mod->shownObjects.clear();
            mod->shownLines.clear();
            mod->UpdateUI(hw);
            return TRUE;
        case WM_COMMAND: {
            const int id = LOWORD(wp), code = HIWORD(wp);
            if (id == IDC_OBJECT && code == CBN_SELCHANGE) {
                const int item = (int)SendMessage(GetDlgItem(hw, IDC_OBJECT), CB_GETCURSEL, 0, 0);
                if (item >= 0 && item < (int)mod->objectList.size()) mod->selectedObject = mod->objectList[item];
                mod->UpdateUI(hw);
                return TRUE;
            }
            if (id == IDC_TILES && code == LBN_SELCHANGE) {
                HWND box = GetDlgItem(hw, IDC_TILES);
                const int item = (int)SendMessage(box, LB_GETCURSEL, 0, 0);
                if (item >= 0) mod->selectedTile = (int)SendMessage(box, LB_GETITEMDATA, item, 0);
                mod->UpdateUI(hw);
                return TRUE;
            }
            if (code != BN_CLICKED) return FALSE;
            if (id >= IDC_FILE1 && id < IDC_FILE1 + kExodusPackChannels) {
                const int c = id - IDC_FILE1;
                std::wstring current = mod->MapFile(mod->FindMap(mod->selectedObject, mod->selectedTile, c));
                if (current.empty()) current = mod->MapFile(mod->FindMap(L"", mod->selectedTile, c));  // start where the shared one is
                const std::wstring file = pick_image(GetCOREInterface()->GetMAXHWnd(), current);
                if (!file.empty()) mod->SetMap(mod->selectedObject, mod->selectedTile, c, file.c_str());
                return TRUE;
            }
            if (id >= IDC_CLEAR1 && id < IDC_CLEAR1 + kExodusPackChannels) {
                mod->SetMap(mod->selectedObject, mod->selectedTile, id - IDC_CLEAR1, nullptr);
                return TRUE;
            }
            if (id == IDC_OUT_DIR_BROWSE) {
                MCHAR dir[MAX_PATH] = L"";
                GetCOREInterface()->ChooseDirectory(GetCOREInterface()->GetMAXHWnd(), exodus_string(IDS_CHOOSE_OUT_DIR), dir);
                if (*dir) {
                    theHold.Begin();
                    mod->pblock->SetValue(kOutDir, 0, dir);
                    theHold.Accept(exodus_string(IDS_OUT_DIR));
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
    const MCHAR* ClassName() override { return exodus_string(IDS_UDIMPACK); }
    const MCHAR* NonLocalizedClassName() override { return _M("UDIM Pack"); }
    SClass_ID SuperClassID() override { return OSM_CLASS_ID; }
    Class_ID ClassID() override { return kPackClassId; }
    const MCHAR* Category() override { return exodus_string(IDS_CATEGORY); }
    const MCHAR* InternalName() override { return _M("exodusPack"); }
    HINSTANCE HInstance() override { return hInstance; }
} packDesc;

// Channel c (0-based) is `channel<n>` and `channelType<n>` in MAXScript, n = c + 1.
#define PACK_CHANNEL_PARAMS(c, n, name)                                                                \
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
    PACK_CHANNEL_PARAMS(0, 1, _M("BaseColor"))
    PACK_CHANNEL_PARAMS(1, 2, _M(""))
    PACK_CHANNEL_PARAMS(2, 3, _M(""))
    PACK_CHANNEL_PARAMS(3, 4, _M(""))
    PACK_CHANNEL_PARAMS(4, 5, _M(""))
    PACK_CHANNEL_PARAMS(5, 6, _M(""))
    kMapTiles, _M("mapTiles"), TYPE_INT_TAB, 0, P_VARIABLE_SIZE, IDS_MAP_TILES,
        p_end,
    kMapChannels, _M("mapChannels"), TYPE_INT_TAB, 0, P_VARIABLE_SIZE, IDS_MAP_CHANNELS,
        p_end,
    kMapFiles, _M("mapFiles"), TYPE_STRING_TAB, 0, P_VARIABLE_SIZE, IDS_MAP_FILES,
        p_end,
    kMapObjects, _M("mapObjects"), TYPE_STRING_TAB, 0, P_VARIABLE_SIZE, IDS_MAP_OBJECTS,
        p_end,
    kCollapsed, _M("collapsedObjects"), TYPE_STRING_TAB, 0, P_VARIABLE_SIZE, IDS_COLLAPSED_OBJECTS,
        p_end,
    p_end);
// clang-format on

// Maps are set per object by name, so they follow a renamed object.
void on_node_renamed(void*, NotifyInfo* info) {
    const NameChange* change = GetNotifyParam<NOTIFY_NODE_RENAMED>(info);
    if (!change || !change->oldname || !change->newname) return;
    const std::vector<PackMod*> mods(packInstances.begin(), packInstances.end());  // a rename can notify
    for (PackMod* mod : mods)
        if (packInstances.count(mod)) mod->RenameObject(change->oldname, change->newname);
}

PackMod::PackMod() {
    static const bool watching = RegisterNotification(on_node_renamed, nullptr, NOTIFY_NODE_RENAMED) != 0;
    (void)watching;
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
