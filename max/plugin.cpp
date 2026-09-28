// udimpacker for 3ds Max 2027: the plugin Max loads at startup. It publishes the `udimpacker`
// MAXScript interface and hands the work to udimpacker_maxcore.dll. That DLL is loaded from a copy
// in the temp folder, so a rebuild can replace it while Max runs and udimpacker.reload() picks up
// the new one without a restart.
#include <max.h>
#include <gup.h>
#include <iparamb2.h>
#include <ifnpub.h>

#include <filesystem>
#include <string>

#include "maxcore_api.h"

HINSTANCE hInstance = nullptr;

namespace {

namespace fs = std::filesystem;

const Class_ID kGupClassId(0x61d84c2a, 0x3f0b7e95);
const Interface_ID kInterfaceId(0x2b7e5d31, 0x46c19a08);

// The core DLL: next to the plugin's folder in the build tree (../core), or where the
// UDIMPACKER_MAXCORE environment variable points.
fs::path core_source() {
    wchar_t buf[MAX_PATH];
    if (DWORD n = GetEnvironmentVariableW(L"UDIMPACKER_MAXCORE", buf, MAX_PATH); n && n < MAX_PATH) return fs::path(buf);
    const DWORD n = GetModuleFileNameW(hInstance, buf, MAX_PATH);
    return fs::path(std::wstring(buf, n)).parent_path().parent_path() / L"core" / L"udimpacker_maxcore.dll";
}

struct Core {
    HMODULE module = nullptr;
    const UdimCoreApi* api = nullptr;
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
        const fs::path dir = fs::temp_directory_path(ec) / L"udimpacker";
        fs::create_directories(dir, ec);
        for (const auto& old : fs::directory_iterator(dir, ec)) {  // copies left behind; loaded ones stay locked
            std::error_code ignore;
            fs::remove(old.path(), ignore);
        }
        copy = dir / (L"udimpacker_maxcore-" + std::to_wstring(GetCurrentProcessId()) + L"-" + std::to_wstring(GetTickCount64()) + L".dll");
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
        const auto fn = reinterpret_cast<UdimCoreApiFn>(GetProcAddress(module, UDIM_CORE_API_EXPORT));
        api = fn ? fn() : nullptr;
        if (!api || api->version != kUdimCoreApiVersion) {
            error = MSTR(src.c_str()) + _M(" is not a udimpacker core of interface version ") + MSTR::FromACP(std::to_string(kUdimCoreApiVersion).c_str());
            unload();
            return false;
        }
        loadedFrom = src.c_str();
        return true;
    }

    bool ready() { return api || load(); }
} core;

// Runs a core function that writes UTF-8 text and returns that text.
template <class F>
MSTR collect(F&& call) {
    std::string text;
    UdimSink sink{[](void* ctx, const char* s, size_t n) { static_cast<std::string*>(ctx)->append(s, n); }, &text};
    call(sink);
    return MSTR::FromUTF8(text.c_str());
}

MSTR core_error() { return MSTR(_M("{\"error\": \"udimpacker core: ")) + core.error + _M("\"}"); }

class UdimInterface : public FPStaticInterface {
public:
    DECLARE_DESCRIPTOR(UdimInterface)
    enum { kVersion, kReload, kInspect };

    BEGIN_FUNCTION_MAP
    FN_0(kVersion, TYPE_TSTR_BV, Version)
    FN_0(kReload, TYPE_TSTR_BV, Reload)
    FN_1(kInspect, TYPE_TSTR_BV, Inspect, TYPE_INODE)
    END_FUNCTION_MAP

    MSTR Version() {
        if (!core.ready()) return core_error();
        return MSTR(_M("udimpacker core ")) + MSTR::FromUTF8(core.api->build) + _M(" from ") + core.loadedFrom;
    }

    MSTR Reload() {
        core.unload();
        return Version();
    }

    MSTR Inspect(INode* node) {
        if (!node) return MSTR(_M("{\"error\": \"no node\"}"));
        if (!core.ready()) return core_error();
        return collect([&](UdimSink sink) { core.api->inspect(node, sink); });
    }
};

// clang-format off
UdimInterface theInterface(kInterfaceId, _M("udimpacker"), 0, nullptr, FP_CORE,
    UdimInterface::kVersion, _M("version"), 0, TYPE_TSTR_BV, FP_NO_REDRAW, 0,
    UdimInterface::kReload, _M("reload"), 0, TYPE_TSTR_BV, FP_NO_REDRAW, 0,
    UdimInterface::kInspect, _M("inspect"), 0, TYPE_TSTR_BV, FP_NO_REDRAW, 1,
        _M("node"), 0, TYPE_INODE,
    p_end);
// clang-format on

// A global utility plugin whose only job is to keep this DLL loaded from startup on.
class UdimGup : public GUP {
public:
    DWORD Start() override { return GUPRESULT_KEEP; }
    void Stop() override { core.unload(); }
    void DeleteThis() override { delete this; }
};

class UdimGupClassDesc : public ClassDesc2 {
public:
    int IsPublic() override { return FALSE; }
    void* Create(BOOL) override { return new UdimGup; }
    // Not "udimpacker": MAXScript would resolve that name to this class instead of the interface.
    const MCHAR* ClassName() override { return _M("udimpackerStartup"); }
    const MCHAR* NonLocalizedClassName() override { return _M("udimpackerStartup"); }
    SClass_ID SuperClassID() override { return GUP_CLASS_ID; }
    Class_ID ClassID() override { return kGupClassId; }
    const MCHAR* Category() override { return _M("udimpacker"); }
    const MCHAR* InternalName() override { return _M("udimpackerStartup"); }
    HINSTANCE HInstance() override { return hInstance; }
};

UdimGupClassDesc gupDesc;

}  // namespace

BOOL WINAPI DllMain(HINSTANCE instance, ULONG reason, LPVOID) {
    if (reason == DLL_PROCESS_ATTACH) {
        hInstance = instance;
        DisableThreadLibraryCalls(instance);
    }
    return TRUE;
}

__declspec(dllexport) const MCHAR* LibDescription() { return _M("udimpacker: unsubdivide, decimate and UDIM packing"); }
__declspec(dllexport) int LibNumberClasses() { return 1; }
__declspec(dllexport) ClassDesc* LibClassDesc(int i) { return i == 0 ? &gupDesc : nullptr; }
__declspec(dllexport) ULONG LibVersion() { return VERSION_3DSMAX; }
__declspec(dllexport) ULONG CanAutoDefer() { return 0; }  // load at startup: the interface must exist from the start
