// Exodus - minimal image container and I/O. Samples are stored at their native depth
// (8 or 16 bit) and channel count (1-4); nothing is color-converted.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct Image {
    int w = 0, h = 0, c = 0;
    int bps = 1;  // bytes per sample: 1 or 2
    std::vector<uint8_t> data;

    void alloc(int w_, int h_, int c_, int bps_) {
        w = w_; h = h_; c = c_; bps = bps_;
        data.assign((size_t)w * h * c * bps, 0);
    }
    uint32_t maxv() const { return bps == 1 ? 255u : 65535u; }
    uint32_t get(size_t px, int ch) const {
        size_t i = px * c + ch;
        return bps == 1 ? data[i] : reinterpret_cast<const uint16_t*>(data.data())[i];
    }
    void set(size_t px, int ch, uint32_t v) {
        size_t i = px * c + ch;
        if (bps == 1) data[i] = (uint8_t)v;
        else reinterpret_cast<uint16_t*>(data.data())[i] = (uint16_t)v;
    }
    bool empty() const { return w == 0; }
};

struct ImageInfo { int w = 0, h = 0, c = 0, bps = 1; };

bool read_file(const std::string& path, std::vector<uint8_t>& bytes);
bool image_info(const std::vector<uint8_t>& bytes, ImageInfo& info);
bool image_decode(const std::vector<uint8_t>& bytes, Image& out, std::string& err);
bool save_png(const std::string& path, const Image& img, std::string& err);

// Read one pixel of `src` promoted to `dstC` channels / `dstBps` depth. Promotion is exact:
// gray is replicated, a missing alpha is opaque, 8-bit values scale by 257.
void promote_pixel(const Image& src, size_t px, int dstC, int dstBps, uint32_t out[4]);

Image downsample2(const Image& src);                  // 2x2 box filter, dims must be even
Image upscale_nearest(const Image& src, int fx, int fy);  // integer replication
