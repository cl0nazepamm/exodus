// Exodus - background bleed. Every pixel no island owns is filled ring by ring from the
// islands outward (each new pixel = average of its already-filled 8-neighbours), so the atlas
// has no solid background for mips or bilinear filtering to pull in.
#pragma once

#include <cstdint>
#include <vector>

#include "image.h"

struct BleedPlan {
    int w = 0, h = 0;
    std::vector<uint32_t> order;  // empty pixels in fill order (by ring)
    std::vector<uint16_t> ring;   // 0 = island pixel, n = filled in ring n
};

BleedPlan make_bleed_plan(const std::vector<uint8_t>& owned, int w, int h);

// `normalMap`: renormalize the averaged RGB as a unit vector (0.5-biased encoding).
void apply_bleed(Image& img, const BleedPlan& plan, bool normalMap);
