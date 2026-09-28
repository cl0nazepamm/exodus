// udimpacker - background bleed.
#include "bleed.h"

#include <cmath>

static constexpr int kDx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
static constexpr int kDy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

BleedPlan make_bleed_plan(const std::vector<uint8_t>& owned, int w, int h) {
    BleedPlan plan;
    plan.w = w; plan.h = h;
    plan.ring.assign((size_t)w * h, 0xFFFF);
    for (size_t i = 0; i < owned.size(); ++i)
        if (owned[i]) plan.ring[i] = 0;

    std::vector<uint32_t> cur, next;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            size_t i = (size_t)y * w + x;
            if (plan.ring[i] != 0xFFFF) continue;
            for (int n = 0; n < 8; ++n) {
                int nx = x + kDx[n], ny = y + kDy[n];
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                if (plan.ring[(size_t)ny * w + nx] == 0) { cur.push_back((uint32_t)i); break; }
            }
        }
    for (uint32_t i : cur) plan.ring[i] = 1;

    for (uint16_t r = 1; !cur.empty(); ++r) {
        plan.order.insert(plan.order.end(), cur.begin(), cur.end());
        next.clear();
        for (uint32_t i : cur) {
            int x = (int)(i % w), y = (int)(i / w);
            for (int n = 0; n < 8; ++n) {
                int nx = x + kDx[n], ny = y + kDy[n];
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                size_t j = (size_t)ny * w + nx;
                if (plan.ring[j] != 0xFFFF) continue;
                plan.ring[j] = r + 1;
                next.push_back((uint32_t)j);
            }
        }
        cur.swap(next);
    }
    return plan;
}

void apply_bleed(Image& img, const BleedPlan& plan, bool normalMap) {
    const int w = plan.w, h = plan.h;
    const double maxv = img.maxv();
    for (uint32_t i : plan.order) {
        const int x = (int)(i % w), y = (int)(i / w);
        const uint16_t r = plan.ring[i];
        uint64_t sum[4] = {0, 0, 0, 0};
        uint32_t n = 0;
        for (int k = 0; k < 8; ++k) {
            int nx = x + kDx[k], ny = y + kDy[k];
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            size_t j = (size_t)ny * w + nx;
            if (plan.ring[j] >= r) continue;
            for (int c = 0; c < img.c; ++c) sum[c] += img.get(j, c);
            ++n;
        }
        if (!n) continue;
        double v[4];
        for (int c = 0; c < img.c; ++c) v[c] = (double)sum[c] / n;
        if (normalMap && img.c >= 3) {
            double nx = v[0] / maxv * 2 - 1, ny = v[1] / maxv * 2 - 1, nz = v[2] / maxv * 2 - 1;
            double len = std::sqrt(nx * nx + ny * ny + nz * nz);
            if (len > 1e-8) {
                v[0] = (nx / len + 1) * 0.5 * maxv;
                v[1] = (ny / len + 1) * 0.5 * maxv;
                v[2] = (nz / len + 1) * 0.5 * maxv;
            }
        }
        for (int c = 0; c < img.c; ++c) img.set(i, c, (uint32_t)std::lround(v[c]));
    }
}
