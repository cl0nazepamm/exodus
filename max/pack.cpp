// Exodus for 3ds Max: the UDIM Pack modifier. One instance goes on every part of a character
// (select them all, add the modifier once). The maps are given by hand per UDIM tile, for all
// objects or for one object: pick an object (or all), a tile in the list, a file for each channel.
// An object's own maps in a tile replace the ones for all objects there, so objects that each use
// tile 1001 with their own textures pack side by side. Tiles with maps go into one atlas, the packed
// UVs go into the target map channel, Preview shows the layout in a window of its own and Bake
// writes the atlas maps. Auto detect fills every object's maps from its material (by slot name);
// they stay editable by hand. With Keep target UV layout the target channel already holds the layout
// (a clean pack of the objects made by hand): nothing is packed and Bake resamples the maps into
// it. The layout lives in the core DLL (see maxcore.cpp); this part holds the parameters, the
// rollout and the preview window.
#include <iparamm2.h>
#include <modstack.h>
#include <notify.h>
#include <polyobj.h>
#include <Util/MainThreadTaskManager.h>
#include <commdlg.h>  // after windows.h (from the Max headers)

#include <algorithm>
#include <cmath>
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
    kKeepLayout = 8,  // the target channel already holds the layout: no packing, the maps are resampled into it
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
void show_preview(ExodusPackPreview&& preview, bool open);

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
    std::wstring detectNote;          // what the last Auto detect found, shown under the status until Preview or Bake
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
        BOOL rotate = TRUE, keep = FALSE;
        pblock->GetValue(kSourceChannel, t, s.sourceChannel, valid);
        pblock->GetValue(kTargetChannel, t, s.targetChannel, valid);
        pblock->GetValue(kKeepLayout, t, keep, valid);
        s.keepLayout = keep != FALSE;
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

    // Auto detect: every object's maps from its material, as one undo step (the channels it names
    // and the rows of the objects it found maps for). Returns the whole report.
    MSTR Detect() {
        const ExodusCoreApi* core = exodus_core();
        if (!core) return status = exodus_core_error();
        Interval valid = FOREVER;
        ExodusPackSettings s = Settings(GetCOREInterface()->GetTime(), valid);
        std::wstring report;
        HCURSOR cursor = SetCursor(LoadCursor(nullptr, IDC_WAIT));
        const bool found = core->pack_detect(this, s, report);
        SetCursor(cursor);
        if (found) {
            theHold.Begin();
            for (int c = 0; c < kExodusPackChannels; ++c) {
                const MCHAR* name = pblock->GetStr(kChannel1 + c, 0);
                if (s.channel[c] == (name ? name : L"")) continue;
                pblock->SetValue(kChannel1 + c, 0, s.channel[c].c_str());
                pblock->SetValue(kKind1 + c, 0, s.kind[c]);
            }
            for (ParamID id : {kMapObjects, kMapFiles, kMapChannels, kMapTiles})
                if (pblock->Count(id) > 0) pblock->Delete(id, 0, pblock->Count(id));
            for (const ExodusPackMap& m : s.maps) AppendMap(m.object, m.tile, m.channel, m.file);
            theHold.Accept(exodus_string(IDS_AUTO_DETECT));
        }
        // In the panel: the summary and the first two problems.
        detectNote.clear();
        int notes = 0;
        for (size_t from = 0; from < report.size() && notes < 3;) {
            const size_t end = std::min(report.find(L'\n', from), report.size());
            std::wstring line = report.substr(from, end - from);
            from = end + 1;
            if (notes && line.rfind(L"  ! ", 0) != 0) continue;
            if (notes) line = line.substr(4);
            detectNote += (notes++ ? L"\n" : L"") + line;
        }
        NotifyDependents(FOREVER, PART_TEXMAP, REFMSG_CHANGE);
        UpdateUI();
        return report.c_str();
    }

    // Bake: the atlas maps into the output folder. Returns the packer's report.
    MSTR Bake() {
        const ExodusCoreApi* core = exodus_core();
        if (!core) return status = exodus_core_error();
        detectNote.clear();
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

    // Preview: Bake's dry run. The layout as Bake would make it, shown in the preview window;
    // nothing is written. Returns the packer's report.
    MSTR Preview() {
        const ExodusCoreApi* core = exodus_core();
        if (!core) return status = exodus_core_error();
        detectNote.clear();
        Interval valid = FOREVER;
        const ExodusPackSettings s = Settings(GetCOREInterface()->GetTime(), valid);
        std::string log;
        ExodusSink sink{[](void* ctx, const char* text, size_t n) { static_cast<std::string*>(ctx)->append(text, n); }, &log};
        std::wstring text;
        bool relayout = false;
        ExodusPackPreview preview;
        HCURSOR cursor = SetCursor(LoadCursor(nullptr, IDC_WAIT));
        const bool ok = core->pack_preview(this, s, sink, text, relayout, preview);
        SetCursor(cursor);
        status = text.c_str();
        if (relayout) NotifyDependents(FOREVER, PART_TEXMAP, REFMSG_CHANGE);
        show_preview(std::move(preview), ok);  // without a layout: an open window says why, none opens
        UpdateUI();
        return MSTR::FromUTF8(log.c_str());
    }

    IParamMap2* Map() {
        IParamMap2* map = Desc().GetParamMap(&PbDesc());
        return map && map->GetParamBlock() == pblock && map->GetHWnd() ? map : nullptr;
    }

    // Refreshes the rollout (`hw` while it opens: Max registers the map only afterwards).
    void UpdateUI(HWND hw = nullptr);

    // A kept layout is not packed: padding and rotation do not apply.
    void EnablePacking(IParamMap2* map) {
        const bool packs = !pblock->GetInt(kKeepLayout, 0);
        map->Enable(kPadding, packs);
        map->Enable(kRotate, packs);
    }

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

// The preview window: a plain window of the plugin's own (so it outlives a core reload) showing the
// last Preview of any UDIM Pack modifier: the atlas, the packed faces' edges in one color per object.
// The wheel zooms at the cursor, a drag pans, a double click fits the atlas to the window again.
class PreviewWindow {
public:
    void Show(ExodusPackPreview&& preview, bool open);

private:
    HWND hwnd = nullptr;
    ExodusPackPreview data;
    double scale = 1, x0 = 0, y0 = 0;  // atlas pixel (x, y) is drawn at (x0 + x * scale, y0 + y * scale)
    bool fitted = true;                // the atlas follows the window's size until zoomed or panned
    bool dragging = false;
    POINT drag{};

    void Fit();
    void Paint(HDC target, const RECT& rc);
    LRESULT Message(HWND hw, UINT msg, WPARAM wp, LPARAM lp);
    static LRESULT CALLBACK Proc(HWND hw, UINT msg, WPARAM wp, LPARAM lp);
} previewWindow;

void show_preview(ExodusPackPreview&& preview, bool open) { previewWindow.Show(std::move(preview), open); }

POINT point_of(LPARAM lp) { return {(short)LOWORD(lp), (short)HIWORD(lp)}; }

// Hues well apart for any number of objects (golden ratio steps).
COLORREF object_color(size_t i) {
    const double h = std::fmod(i * 0.618034, 1.0) * 6, s = 0.55;
    const int k = std::min((int)h, 5);
    const double f = h - k, p = 1 - s, q = 1 - s * f, t = 1 - s * (1 - f);
    const double rgb[6][3] = {{1, t, p}, {q, 1, p}, {p, 1, t}, {p, q, 1}, {t, p, 1}, {1, p, q}};
    return RGB((int)(rgb[k][0] * 255), (int)(rgb[k][1] * 255), (int)(rgb[k][2] * 255));
}

// Opens the window (open: only when it is not open yet), or shows `preview` in the open one. A new
// atlas size is fitted to the window; at the same size the view stays, to compare.
void PreviewWindow::Show(ExodusPackPreview&& preview, bool open) {
    static const wchar_t kClass[] = L"ExodusPackPreview";
    if (!hwnd && !open) return;
    if (preview.width != data.width || preview.height != data.height) fitted = true;
    data = std::move(preview);
    if (!hwnd) {
        static const bool registered = [] {
            WNDCLASSEXW wc = {sizeof wc};
            wc.style = CS_DBLCLKS;
            wc.lpfnWndProc = Proc;
            wc.hInstance = hInstance;
            wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
            wc.lpszClassName = kClass;
            return RegisterClassExW(&wc) != 0;
        }();
        (void)registered;
        HWND owner = GetCOREInterface()->GetMAXHWnd();  // owned: stays above Max, minimizes with it
        RECT r;
        GetWindowRect(owner, &r);
        const int side = MaxSDK::UIScaled(720);
        CreateWindowExW(0, kClass, L"", WS_OVERLAPPEDWINDOW, (r.left + r.right - side) / 2, (r.top + r.bottom - side) / 2, side, side,
                        owner, nullptr, hInstance, this);
        if (!hwnd) return;
        fitted = true;
    }
    std::wstring title = L"UDIM Pack preview";
    if (data.width) title += L": " + data.report.substr(0, data.report.find(L'\n'));
    SetWindowTextW(hwnd, title.c_str());
    if (fitted) Fit();
    InvalidateRect(hwnd, nullptr, FALSE);
    ShowWindow(hwnd, IsIconic(hwnd) ? SW_RESTORE : SW_SHOW);
}

void PreviewWindow::Fit() {
    RECT rc;
    GetClientRect(hwnd, &rc);
    const double w = std::max(data.width, 1), h = std::max(data.height, 1), margin = MaxSDK::UIScaled(12);
    scale = std::max(std::min((rc.right - 2 * margin) / w, (rc.bottom - 2 * margin) / h), 1e-6);
    x0 = (rc.right - w * scale) / 2;
    y0 = (rc.bottom - h * scale) / 2;
}

void PreviewWindow::Paint(HDC target, const RECT& rc) {
    HDC dc = CreateCompatibleDC(target);  // drawn off screen, then copied: no flicker while zooming
    HBITMAP bitmap = CreateCompatibleBitmap(target, std::max<LONG>(rc.right, 1), std::max<LONG>(rc.bottom, 1));
    HGDIOBJ oldBitmap = SelectObject(dc, bitmap);
    auto fill = [&](const RECT& r, COLORREF c) {
        SetBkColor(dc, c);
        ExtTextOutW(dc, 0, 0, ETO_OPAQUE, &r, nullptr, 0, nullptr);
    };
    fill(rc, RGB(45, 45, 45));

    if (data.width > 0) {
        const double w = data.width * scale, h = data.height * scale;
        const RECT atlas = {(LONG)std::lround(x0), (LONG)std::lround(y0), (LONG)std::lround(x0 + w), (LONG)std::lround(y0 + h)};
        fill(atlas, RGB(18, 18, 18));
        HBRUSH frame = CreateSolidBrush(RGB(100, 100, 100));
        FrameRect(dc, &atlas, frame);
        DeleteObject(frame);
        // One PolyPolyline per object, of the edges that reach into the window.
        std::vector<POINT> points;
        std::vector<DWORD> counts;
        for (size_t o = 0; o + 1 < data.objectFirst.size(); ++o) {
            points.clear();
            for (uint32_t e = data.objectFirst[o]; e < data.objectFirst[o + 1]; ++e) {
                const auto& [u0, v0, u1, v1] = data.edges[e];
                const POINT a{(LONG)std::lround(x0 + u0 * w), (LONG)std::lround(y0 + (1 - v0) * h)};
                const POINT b{(LONG)std::lround(x0 + u1 * w), (LONG)std::lround(y0 + (1 - v1) * h)};
                if ((a.x < 0 && b.x < 0) || (a.y < 0 && b.y < 0) || (a.x >= rc.right && b.x >= rc.right) || (a.y >= rc.bottom && b.y >= rc.bottom))
                    continue;
                points.push_back(a);
                points.push_back(b);
            }
            if (points.empty()) continue;
            counts.assign(points.size() / 2, 2);
            HPEN pen = CreatePen(PS_SOLID, 1, object_color(o));
            HGDIOBJ oldPen = SelectObject(dc, pen);
            PolyPolyline(dc, points.data(), counts.data(), (DWORD)counts.size());
            SelectObject(dc, oldPen);
            DeleteObject(pen);
        }
    }

    // Top left: the report (all of it without a layout, its warnings with one), the objects in their
    // colors as far as they fit. Bottom left: how to move the view.
    HGDIOBJ oldFont = SelectObject(dc, GetCOREInterface()->GetAppHFont());
    TEXTMETRICW tm;
    GetTextMetricsW(dc, &tm);
    const int pad = MaxSDK::UIScaled(6), line = tm.tmHeight;
    int y = pad;
    SetBkColor(dc, RGB(45, 45, 45));
    auto text = [&](const std::wstring& s, COLORREF c, int at) {
        SetTextColor(dc, c);
        TextOutW(dc, pad, at, s.c_str(), (int)s.size());
    };
    const size_t nl = data.report.find(L'\n');
    const std::wstring notes = !data.width ? data.report : nl == std::wstring::npos ? L"" : data.report.substr(nl + 1);  // the first line is the title
    for (size_t from = 0; from < notes.size(); y += line) {
        const size_t end = std::min(notes.find(L'\n', from), notes.size());
        text(notes.substr(from, end - from), data.width ? RGB(235, 185, 90) : RGB(220, 220, 220), y);
        from = end + 1;
    }
    const size_t room = (size_t)std::max(1, (int)(rc.bottom - pad - 2 * line - y) / line);  // object lines above the hint
    for (size_t o = 0; o < data.objects.size(); ++o, y += line) {
        if (o + 1 == room && data.objects.size() > room) {
            text(L"+ " + std::to_wstring(data.objects.size() - o) + L" more", RGB(160, 160, 160), y);
            break;
        }
        text(data.objects[o], object_color(o), y);
    }
    text(L"wheel: zoom   drag: pan   double click: fit", RGB(130, 130, 130), rc.bottom - pad - line);
    SelectObject(dc, oldFont);

    BitBlt(target, 0, 0, rc.right, rc.bottom, dc, 0, 0, SRCCOPY);
    SelectObject(dc, oldBitmap);
    DeleteObject(bitmap);
    DeleteDC(dc);
}

LRESULT PreviewWindow::Message(HWND hw, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_SIZE:
        if (fitted) Fit();
        InvalidateRect(hw, nullptr, FALSE);
        return 0;
    case WM_ERASEBKGND:
        return 1;
    case WM_PAINT: {
        PAINTSTRUCT ps;
        HDC dc = BeginPaint(hw, &ps);
        RECT rc;
        GetClientRect(hw, &rc);
        Paint(dc, rc);
        EndPaint(hw, &ps);
        return 0;
    }
    case WM_MOUSEWHEEL: {
        POINT p = point_of(lp);
        ScreenToClient(hw, &p);
        const double next = std::clamp(scale * std::pow(1.2, GET_WHEEL_DELTA_WPARAM(wp) / (double)WHEEL_DELTA),
                                       32.0 / std::max({data.width, data.height, 1}), 64.0);
        x0 = p.x - (p.x - x0) * next / scale;
        y0 = p.y - (p.y - y0) * next / scale;
        scale = next;
        fitted = false;
        InvalidateRect(hw, nullptr, FALSE);
        return 0;
    }
    case WM_LBUTTONDOWN:
    case WM_MBUTTONDOWN:
        SetCapture(hw);
        dragging = true;
        drag = point_of(lp);
        return 0;
    case WM_MOUSEMOVE:
        if (dragging) {
            const POINT p = point_of(lp);
            x0 += p.x - drag.x;
            y0 += p.y - drag.y;
            drag = p;
            fitted = false;
            InvalidateRect(hw, nullptr, FALSE);
        }
        return 0;
    case WM_LBUTTONUP:
    case WM_MBUTTONUP:
        ReleaseCapture();
        return 0;
    case WM_CAPTURECHANGED:
        dragging = false;
        return 0;
    case WM_LBUTTONDBLCLK:
        fitted = true;
        Fit();
        InvalidateRect(hw, nullptr, FALSE);
        return 0;
    case WM_NCDESTROY:
        hwnd = nullptr;
        data = {};
        break;
    }
    return DefWindowProcW(hw, msg, wp, lp);
}

LRESULT CALLBACK PreviewWindow::Proc(HWND hw, UINT msg, WPARAM wp, LPARAM lp) {
    if (msg == WM_NCCREATE) {
        auto* self = static_cast<PreviewWindow*>(reinterpret_cast<CREATESTRUCTW*>(lp)->lpCreateParams);
        self->hwnd = hw;
        SetWindowLongPtrW(hw, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(self));
    }
    auto* self = reinterpret_cast<PreviewWindow*>(GetWindowLongPtrW(hw, GWLP_USERDATA));
    return self ? self->Message(hw, msg, wp, lp) : DefWindowProcW(hw, msg, wp, lp);
}

std::wstring file_name(const std::wstring& path) { return std::filesystem::path(path).filename().wstring(); }

void PackMod::UpdateUI(HWND hw) {
    if (!hw) {
        IParamMap2* map = Map();
        if (!map) return;
        hw = map->GetHWnd();
        EnablePacking(map);
    }
    SetDlgItemText(hw, IDC_STATUS, detectNote.empty() ? status.data() : (std::wstring(status.data()) + L"\n" + detectNote).c_str());

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
            mod->EnablePacking(map);
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
            if (id == IDC_DETECT) {
                mod->Detect();
                return TRUE;
            }
            if (id == IDC_PREVIEW) {
                mod->Preview();
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
    kKeepLayout, _M("keepLayout"), TYPE_BOOL, 0, IDS_KEEP_LAYOUT,
        p_default, FALSE,
        p_ui, TYPE_SINGLECHECKBOX, IDC_KEEP_LAYOUT,
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

MSTR pack_detect(ReferenceTarget* target) {
    auto* mod = dynamic_cast<PackMod*>(target);
    if (!mod || !packInstances.count(mod)) return _M("not a UDIM Pack modifier");
    return mod->Detect();
}

MSTR pack_preview(ReferenceTarget* target) {
    auto* mod = dynamic_cast<PackMod*>(target);
    if (!mod || !packInstances.count(mod)) return _M("not a UDIM Pack modifier");
    MSTR log = mod->Preview();
    return log + _M("\n") + mod->status;
}
