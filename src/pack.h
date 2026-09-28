// udimpacker - true-shape raster packer. Islands are packed by their dilated pixel footprint,
// not their bounding box, on a coarse cell grid. Rotations are quarter turns only, so every
// placement stays on the pixel grid.
#pragma once

#include <cstdint>
#include <utility>
#include <vector>

// Rotate pixel (x, y) of a w x h grid by k clockwise quarter turns (image space, y down).
inline void rot_px(int k, int w, int h, int x, int y, int& X, int& Y) {
    switch (k & 3) {
        case 0: X = x; Y = y; break;
        case 1: X = h - 1 - y; Y = x; break;
        case 2: X = w - 1 - x; Y = h - 1 - y; break;
        default: X = y; Y = w - 1 - x; break;
    }
}

// Same rotation for continuous coordinates (pixel corners on integers).
inline void rot_pt(int k, double w, double h, double x, double y, double& X, double& Y) {
    switch (k & 3) {
        case 0: X = x; Y = y; break;
        case 1: X = h - y; Y = x; break;
        case 2: X = w - x; Y = h - y; break;
        default: X = y; Y = w - x; break;
    }
}

struct Footprint {
    int w = 0, h = 0;                                   // cells
    std::vector<std::vector<std::pair<int, int>>> rows;  // occupied cell runs [a, b) per row
    size_t cells = 0;
};

struct PackItem {
    Footprint rot[4];
};

struct Placement {
    int x = 0, y = 0;  // footprint origin, cells
    int rot = 0;
};

// Footprint = island mask grown by `pad` pixels on every side, rasterized to `cell`-sized cells.
// The island itself sits at offset (pad, pad) inside the footprint for every rotation.
PackItem build_item(const std::vector<uint8_t>& mask, int w, int h, int pad, int cell);

// Tries several greedy orders/scorings and returns the first layout that fits.
// `attempts` (optional) receives the number of passes used.
bool pack_items(const std::vector<PackItem>& items, int gridW, int gridH, bool allowRotate,
                std::vector<Placement>& out, int* attempts = nullptr);
