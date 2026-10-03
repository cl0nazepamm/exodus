// Exodus - image I/O (stb_image in, lodepng out).
#include "image.h"
#include "exodus.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>

#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_GIF
#define STBI_NO_PIC
#define STBI_NO_PNM
#include "stb_image.h"
#include "lodepng.h"

namespace fs = std::filesystem;

bool read_file(const std::string& path, std::vector<uint8_t>& bytes) {
    std::ifstream f(upath(path), std::ios::binary | std::ios::ate);
    if (!f) return false;
    std::streamsize n = f.tellg();
    f.seekg(0);
    bytes.resize((size_t)n);
    return (bool)f.read(reinterpret_cast<char*>(bytes.data()), n);
}

bool image_info(const std::vector<uint8_t>& bytes, ImageInfo& info) {
    int w, h, c;
    if (!stbi_info_from_memory(bytes.data(), (int)bytes.size(), &w, &h, &c)) return false;
    info.w = w; info.h = h; info.c = c;
    info.bps = stbi_is_16_bit_from_memory(bytes.data(), (int)bytes.size()) ? 2 : 1;
    return true;
}

bool image_decode(const std::vector<uint8_t>& bytes, Image& out, std::string& err) {
    ImageInfo info;
    if (!image_info(bytes, info)) { err = stbi_failure_reason(); return false; }
    int w, h, c;
    void* px = info.bps == 2
        ? (void*)stbi_load_16_from_memory(bytes.data(), (int)bytes.size(), &w, &h, &c, 0)
        : (void*)stbi_load_from_memory(bytes.data(), (int)bytes.size(), &w, &h, &c, 0);
    if (!px) { err = stbi_failure_reason(); return false; }
    out.alloc(w, h, c, info.bps);
    memcpy(out.data.data(), px, out.data.size());
    stbi_image_free(px);
    return true;
}

bool save_png(const std::string& path, const Image& img, std::string& err) {
    static const LodePNGColorType types[5] = {LCT_GREY, LCT_GREY, LCT_GREY_ALPHA, LCT_RGB, LCT_RGBA};
    std::vector<uint8_t> raw;
    const std::vector<uint8_t>* src = &img.data;
    if (img.bps == 2) {  // lodepng wants big-endian 16-bit samples
        raw.resize(img.data.size());
        for (size_t i = 0; i < raw.size(); i += 2) { raw[i] = img.data[i + 1]; raw[i + 1] = img.data[i]; }
        src = &raw;
    }
    lodepng::State state;
    state.info_raw.colortype = types[img.c];
    state.info_raw.bitdepth = img.bps * 8;
    state.info_png.color.colortype = types[img.c];
    state.info_png.color.bitdepth = img.bps * 8;
    state.encoder.auto_convert = 0;
    std::vector<uint8_t> png;
    unsigned e = lodepng::encode(png, *src, (unsigned)img.w, (unsigned)img.h, state);
    if (e) { err = lodepng_error_text(e); return false; }
    std::ofstream f(upath(path), std::ios::binary);
    if (!f.write(reinterpret_cast<const char*>(png.data()), png.size())) { err = "write failed"; return false; }
    return true;
}

void promote_pixel(const Image& src, size_t px, int dstC, int dstBps, uint32_t out[4]) {
    const uint32_t mul = (dstBps == 2 && src.bps == 1) ? 257u : 1u;
    const uint32_t opaque = dstBps == 2 ? 65535u : 255u;
    uint32_t v[4];
    switch (src.c) {
        case 1: v[0] = v[1] = v[2] = src.get(px, 0) * mul; v[3] = opaque; break;
        case 2: v[0] = v[1] = v[2] = src.get(px, 0) * mul; v[3] = src.get(px, 1) * mul; break;
        case 3: for (int i = 0; i < 3; ++i) v[i] = src.get(px, i) * mul; v[3] = opaque; break;
        default: for (int i = 0; i < 4; ++i) v[i] = src.get(px, i) * mul; break;
    }
    switch (dstC) {
        case 1: out[0] = v[0]; break;
        case 2: out[0] = v[0]; out[1] = v[3]; break;
        case 3: out[0] = v[0]; out[1] = v[1]; out[2] = v[2]; break;
        default: for (int i = 0; i < 4; ++i) out[i] = v[i]; break;
    }
}

Image downsample2(const Image& src) {
    Image d;
    d.alloc(src.w / 2, src.h / 2, src.c, src.bps);
    for (int y = 0; y < d.h; ++y)
        for (int x = 0; x < d.w; ++x) {
            size_t a = (size_t)(2 * y) * src.w + 2 * x, b = a + src.w;
            for (int ch = 0; ch < src.c; ++ch) {
                uint32_t s = src.get(a, ch) + src.get(a + 1, ch) + src.get(b, ch) + src.get(b + 1, ch);
                d.set((size_t)y * d.w + x, ch, (s + 2) / 4);
            }
        }
    return d;
}

// The source samples behind each of `m` output samples along an axis of `n`, with their weights:
// the two nearest sample centres when enlarging, the area each source sample covers when shrinking.
static std::vector<std::vector<std::pair<int, float>>> axis_taps(int n, int m) {
    std::vector<std::vector<std::pair<int, float>>> taps(m);
    const double s = (double)n / m;
    for (int o = 0; o < m; ++o) {
        auto& t = taps[o];
        if (s <= 1) {
            const double x = (o + 0.5) * s - 0.5;
            const int i = (int)std::floor(x);
            const double f = x - i;
            t.push_back({std::clamp(i, 0, n - 1), (float)(1 - f)});
            t.push_back({std::clamp(i + 1, 0, n - 1), (float)f});
        } else {
            const double x0 = o * s, x1 = (o + 1) * s;
            for (int i = (int)std::floor(x0); i < n && i < x1; ++i) {
                const double w = std::min(x1, i + 1.0) - std::max(x0, (double)i);
                if (w > 0) t.push_back({i, (float)(w / s)});
            }
        }
    }
    return taps;
}

Image resample(const Image& src, int w, int h) {
    const auto tx = axis_taps(src.w, w), ty = axis_taps(src.h, h);
    std::vector<float> rows((size_t)w * src.h * src.c);  // resampled across, not yet down
    for (int y = 0; y < src.h; ++y)
        for (int x = 0; x < w; ++x)
            for (int ch = 0; ch < src.c; ++ch) {
                float v = 0;
                for (auto [i, wt] : tx[x]) v += wt * (float)src.get((size_t)y * src.w + i, ch);
                rows[((size_t)y * w + x) * src.c + ch] = v;
            }
    Image d;
    d.alloc(w, h, src.c, src.bps);
    const long maxv = (long)src.maxv();
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            for (int ch = 0; ch < src.c; ++ch) {
                float v = 0;
                for (auto [i, wt] : ty[y]) v += wt * rows[((size_t)i * w + x) * src.c + ch];
                d.set((size_t)y * w + x, ch, (uint32_t)std::clamp(std::lround(v), 0L, maxv));
            }
    return d;
}

Image upscale_nearest(const Image& src, int fx, int fy) {
    Image d;
    d.alloc(src.w * fx, src.h * fy, src.c, src.bps);
    for (int y = 0; y < d.h; ++y)
        for (int x = 0; x < d.w; ++x)
            for (int ch = 0; ch < src.c; ++ch)
                d.set((size_t)y * d.w + x, ch, src.get((size_t)(y / fy) * src.w + x / fx, ch));
    return d;
}
