// Exodus for 3ds Max 2027: the plugin Max loads at startup. It registers the modifiers,
// publishes the `exodus` MAXScript interface and hands the work to exodus_maxcore.dll.
// That DLL is loaded from a copy in the temp folder, so a rebuild can replace it while Max runs
// and exodus.reload() picks up the new one without a restart.
#include <gup.h>
#include <ifnpub.h>
#include <Util/MainThreadTaskManager.h>

#include <filesystem>
#include <map>
#include <mutex>
#include <string>

#include "plugin.h"

HINSTANCE hInstance = nullptr;

namespace {

namespace fs = std::filesystem;

const Class_ID kGupClassId(0x61d84c2a, 0x3f0b7e95);
const Interface_ID kInterfaceId(0x2b7e5d31, 0x46c19a08);

// The core DLL: next to the plugin's folder in the build tree (../core), or where the
// EXODUS_MAXCORE environment variable points.
fs::path core_source() {
    wchar_t buf[MAX_PATH];
    if (DWORD n = GetEnvironmentVariableW(L"EXODUS_MAXCORE", buf, MAX_PATH); n && n < MAX_PATH) return fs::path(buf);
    const DWORD n = GetModuleFileNameW(hInstance, buf, MAX_PATH);
    return fs::path(std::wstring(buf, n)).parent_path().parent_path() / L"core" / L"exodus_maxcore.dll";
}

struct Core {
    std::mutex lock;
    HMODULE module = nullptr;
    const ExodusCoreApi* api = nullptr;
    fs::path copy;
    MSTR loadedFrom, error;

    void unload() {
        if (module) FreeLibrary(module);
        module = nullptr;
        api = nullptr;
        std::error_code ec;
        if (!copy.empty()) fs::remove(copy, ec);
        copy.clear();
    }

    bool load() {
        unload();
        error = _M("");
        const fs::path src = core_source();
        std::error_code ec;
        const fs::path dir = fs::temp_directory_path(ec) / L"exodus";
        fs::create_directories(dir, ec);
        for (const auto& old : fs::directory_iterator(dir, ec)) {  // copies left behind; loaded ones stay locked
            std::error_code ignore;
            fs::remove(old.path(), ignore);
        }
        copy = dir / (L"exodus_maxcore-" + std::to_wstring(GetCurrentProcessId()) + L"-" + std::to_wstring(GetTickCount64()) + L".dll");
        if (!fs::copy_file(src, copy, fs::copy_options::overwrite_existing, ec)) {
            error = MSTR(_M("cannot copy ")) + src.c_str() + _M(": ") + MSTR::FromACP(ec.message().c_str());
            copy.clear();
            return false;
        }
        module = LoadLibraryW(copy.c_str());
        if (!module) {
            error = MSTR(_M("cannot load ")) + copy.c_str();
            return false;
        }
        const auto fn = reinterpret_cast<ExodusCoreApiFn>(GetProcAddress(module, EXODUS_CORE_API_EXPORT));
        api = fn ? fn() : nullptr;
        if (!api || api->version != kExodusCoreApiVersion) {
            error = MSTR(src.c_str()) + _M(" is not a Exodus core of interface version ") + MSTR::FromACP(std::to_string(kExodusCoreApiVersion).c_str());
            unload();
            return false;
        }
        loadedFrom = src.c_str();
        return true;
    }

    const ExodusCoreApi* get() {
        std::lock_guard guard(lock);
        return api || load() ? api : nullptr;
    }
} core;

// Runs a core function that writes UTF-8 text and returns that text.
template <class F>
MSTR collect(F&& call) {
    std::string text;
    ExodusSink sink{[](void* ctx, const char* s, size_t n) { static_cast<std::string*>(ctx)->append(s, n); }, &text};
    call(sink);
    return MSTR::FromUTF8(text.c_str());
}

MSTR json_error(const MSTR& what) { return MSTR(_M("{\"error\": \"")) + what + _M("\"}"); }

class ExodusInterface : public FPStaticInterface {
public:
    DECLARE_DESCRIPTOR(ExodusInterface)
    enum { kVersion, kReload, kInspect, kSync, kBake };

    BEGIN_FUNCTION_MAP
    FN_0(kVersion, TYPE_TSTR_BV, Version)
    FN_0(kReload, TYPE_TSTR_BV, Reload)
    FN_1(kInspect, TYPE_TSTR_BV, Inspect, TYPE_INODE)
    VFN_0(kSync, Sync)
    FN_1(kBake, TYPE_TSTR_BV, Bake, TYPE_REFTARG)
    END_FUNCTION_MAP

    // Bake of a UDIM Pack modifier, as its button does; returns the report.
    MSTR Bake(ReferenceTarget* modifier) { return pack_bake(modifier); }

    // Runs the work the modifiers left for after the evaluation (Skin weight updates) right now,
    // for scripts that change a modifier and read Skin in the same go.
    void Sync() { IMainThreadTaskManager::GetInstance()->ExecutePendingTasks(); }

    MSTR Version() {
        const ExodusCoreApi* api = exodus_core();
        if (!api) return exodus_core_error();
        return MSTR(_M("Exodus core ")) + MSTR::FromUTF8(api->build) + _M(" from ") + core.loadedFrom;
    }

    MSTR Reload() {
        {
            std::lock_guard guard(core.lock);
            core.unload();
        }
        MSTR v = Version();
        reduce_refresh_all();
        pack_refresh_all();
        return v;
    }

    MSTR Inspect(INode* node) {
        if (!node) return json_error(_M("no node"));
        const ExodusCoreApi* api = exodus_core();
        if (!api) return json_error(exodus_core_error());
        return collect([&](ExodusSink sink) { api->inspect(node, sink); });
    }
};

// clang-format off
ExodusInterface theInterface(kInterfaceId, _M("exodus"), 0, nullptr, FP_CORE,
    ExodusInterface::kVersion, _M("version"), 0, TYPE_TSTR_BV, FP_NO_REDRAW, 0,
    ExodusInterface::kReload, _M("reload"), 0, TYPE_TSTR_BV, FP_NO_REDRAW, 0,
    ExodusInterface::kInspect, _M("inspect"), 0, TYPE_TSTR_BV, FP_NO_REDRAW, 1,
        _M("node"), 0, TYPE_INODE,
    ExodusInterface::kSync, _M("sync"), 0, TYPE_VOID, 0, 0,
    ExodusInterface::kBake, _M("bake"), 0, TYPE_TSTR_BV, 0, 1,
        _M("modifier"), 0, TYPE_REFTARG,
    p_end);
// clang-format on

// A global utility plugin whose only job is to keep this DLL loaded from startup on.
class ExodusGup : public GUP {
public:
    DWORD Start() override { return GUPRESULT_KEEP; }
    void Stop() override { core.unload(); }
    void DeleteThis() override { delete this; }
};

class ExodusGupClassDesc : public ClassDesc2 {
public:
    int IsPublic() override { return FALSE; }
    void* Create(BOOL) override { return new ExodusGup; }
    // Not "exodus": MAXScript would resolve that name to this class instead of the interface.
    const MCHAR* ClassName() override { return _M("exodusStartup"); }
    const MCHAR* NonLocalizedClassName() override { return _M("exodusStartup"); }
    SClass_ID SuperClassID() override { return GUP_CLASS_ID; }
    Class_ID ClassID() override { return kGupClassId; }
    const MCHAR* Category() override { return _M("Exodus"); }
    const MCHAR* InternalName() override { return _M("exodusStartup"); }
    HINSTANCE HInstance() override { return hInstance; }
};

ExodusGupClassDesc gupDesc;

}  // namespace

const MCHAR* exodus_string(int id) {  // one stored string per id: Max may keep the pointer
    static std::map<int, MSTR> strings;
    auto [it, fresh] = strings.try_emplace(id);
    if (fresh) {
        MCHAR buf[256];
        if (hInstance && LoadStringW(hInstance, id, buf, _countof(buf))) it->second = buf;
    }
    return it->second.data();
}

const ExodusCoreApi* exodus_core() { return core.get(); }

const ExodusCoreApi* exodus_core_loaded() {
    std::lock_guard guard(core.lock);
    return core.api;
}

MSTR exodus_core_error() { return MSTR(_M("Exodus core: ")) + core.error; }

BOOL WINAPI DllMain(HINSTANCE instance, ULONG reason, LPVOID) {
    if (reason == DLL_PROCESS_ATTACH) {
        hInstance = instance;
        DisableThreadLibraryCalls(instance);
    }
    return TRUE;
}

__declspec(dllexport) const MCHAR* LibDescription() { return _M("Exodus: unsubdivide, decimate and UDIM packing"); }
__declspec(dllexport) int LibNumberClasses() { return 4; }
__declspec(dllexport) ClassDesc* LibClassDesc(int i) {
    switch (i) {
    case 0: return &gupDesc;
    case 1: return unsubdivide_desc();
    case 2: return decimate_desc();
    case 3: return pack_desc();
    default: return nullptr;
    }
}
__declspec(dllexport) ULONG LibVersion() { return VERSION_3DSMAX; }
__declspec(dllexport) ULONG CanAutoDefer() { return 0; }  // load at startup: the interface must exist from the start
