// Exodus - true-shape raster packer.
#include "pack.h"

#include <algorithm>
#include <bit>
#include <climits>
#include <numeric>
#include <random>

PackItem build_item(const std::vector<uint8_t>& mask, int w, int h, int pad, int cell) {
    const int pw = w + 2 * pad, ph = h + 2 * pad;

    // Square dilation by `pad`, separable, with difference arrays so cost is independent of pad.
    std::vector<uint8_t> hdil((size_t)pw * ph, 0);
    std::vector<int> diff(pw + 1);
    for (int y = 0; y < h; ++y) {
        std::fill(diff.begin(), diff.end(), 0);
        const uint8_t* m = &mask[(size_t)y * w];
        for (int x = 0; x < w; ++x)
            if (m[x]) { diff[x] += 1; diff[x + 2 * pad + 1] -= 1; }
        uint8_t* o = &hdil[(size_t)(y + pad) * pw];
        for (int x = 0, run = 0; x < pw; ++x) { run += diff[x]; o[x] = run > 0; }
    }
    std::vector<uint8_t> dil((size_t)pw * ph, 0);
    std::vector<int> vdiff(ph + 1);
    for (int x = 0; x < pw; ++x) {
        std::fill(vdiff.begin(), vdiff.end(), 0);
        for (int y = pad; y < pad + h; ++y)
            if (hdil[(size_t)y * pw + x]) { vdiff[y - pad] += 1; vdiff[y + pad + 1] -= 1; }
        for (int y = 0, run = 0; y < ph; ++y) { run += vdiff[y]; dil[(size_t)y * pw + x] = run > 0; }
    }

    PackItem item;
    for (int k = 0; k < 4; ++k) {
        const int rw = (k & 1) ? ph : pw, rh = (k & 1) ? pw : ph;
        Footprint& fp = item.rot[k];
        fp.w = (rw + cell - 1) / cell;
        fp.h = (rh + cell - 1) / cell;
        std::vector<uint8_t> cells((size_t)fp.w * fp.h, 0);
        for (int y = 0; y < ph; ++y)
            for (int x = 0; x < pw; ++x) {
                if (!dil[(size_t)y * pw + x]) continue;
                int X, Y;
                rot_px(k, pw, ph, x, y, X, Y);
                cells[(size_t)(Y / cell) * fp.w + X / cell] = 1;
            }
        fp.rows.resize(fp.h);
        for (int y = 0; y < fp.h; ++y) {
            const uint8_t* r = &cells[(size_t)y * fp.w];
            for (int x = 0; x < fp.w;) {
                if (!r[x]) { ++x; continue; }
                int a = x;
                while (x < fp.w && r[x]) ++x;
                fp.rows[y].push_back({a, x});
                fp.cells += x - a;
            }
        }
    }
    return item;
}

namespace {

struct Grid {
    int w, h, words;
    std::vector<uint64_t> bits;
    Grid(int w_, int h_) : w(w_), h(h_), words((w_ + 63) / 64), bits((size_t)h_ * ((w_ + 63) / 64), 0) {}
    const uint64_t* row(int y) const { return &bits[(size_t)y * words]; }
    uint64_t* row(int y) { return &bits[(size_t)y * words]; }
};

// First index in [lo, hi) whose bit equals `want`, or -1.
int find_bit(const uint64_t* row, int lo, int hi, bool want) {
    if (lo >= hi) return -1;
    int wi = lo >> 6;
    const int we = (hi - 1) >> 6;
    uint64_t m = (want ? row[wi] : ~row[wi]) & (~0ull << (lo & 63));
    for (;;) {
        if (wi == we) {
            const int e = hi & 63;
            if (e) m &= (1ull << e) - 1;
            return m ? (wi << 6) + std::countr_zero(m) : -1;
        }
        if (m) return (wi << 6) + std::countr_zero(m);
        ++wi;
        m = want ? row[wi] : ~row[wi];
    }
}

// -1 if `fp` fits at (x, y); otherwise the next x worth testing on this row. `hint` remembers
// the footprint row that blocked last time, which usually blocks again.
int test_fit(const Grid& g, const Footprint& fp, int x, int y, int& hint) {
    auto blocked = [&](int r) -> int {
        const uint64_t* row = g.row(y + r);
        for (const auto& [a, b] : fp.rows[r]) {
            int p = find_bit(row, x + a, x + b, true);
            if (p < 0) continue;
            int q = find_bit(row, p, g.w, false);  // end of the occupied run
            if (q < 0) return INT_MAX;             // occupied to the edge: nothing fits on this row
            return std::max(q - a, x + 1);
        }
        return -1;
    };
    if (hint < fp.h) {
        int n = blocked(hint);
        if (n >= 0) return n;
    }
    for (int r = 0; r < fp.h; ++r) {
        if (r == hint) continue;
        int n = blocked(r);
        if (n >= 0) { hint = r; return n; }
    }
    return -1;
}

void stamp(Grid& g, const Footprint& fp, int x, int y) {
    for (int r = 0; r < fp.h; ++r) {
        uint64_t* row = g.row(y + r);
        for (const auto& [a, b] : fp.rows[r])
            for (int i = x + a; i < x + b; ++i) row[i >> 6] |= 1ull << (i & 63);
    }
}

// One greedy pass in the given order. score 0: lowest bottom edge, then leftmost.
// score 1: topmost, then leftmost.
bool pack_pass(const std::vector<PackItem>& items, const std::vector<size_t>& order, int gridW, int gridH,
               bool allowRotate, int score, std::vector<Placement>& out) {
    Grid g(gridW, gridH);
    out.assign(items.size(), {});
    for (size_t idx : order) {
        int64_t bestKey = INT64_MAX;
        int bestX = 0, bestY = 0, bestK = -1;
        for (int k = 0; k < (allowRotate ? 4 : 1); ++k) {
            const Footprint& fp = items[idx].rot[k];
            if (fp.w > gridW || fp.h > gridH) continue;
            int hint = 0;
            for (int y = 0; y + fp.h <= gridH; ++y) {
                const int64_t rowKey = (int64_t)(score == 0 ? y + fp.h : y) * gridW;
                if (rowKey >= bestKey) break;  // nothing on this row or below can beat the best
                bool found = false;
                for (int x = 0; x + fp.w <= gridW;) {
                    int next = test_fit(g, fp, x, y, hint);
                    if (next < 0) {
                        if (rowKey + x < bestKey) { bestKey = rowKey + x; bestX = x; bestY = y; bestK = k; }
                        found = true;
                        break;
                    }
                    if (next == INT_MAX) break;
                    x = next;
                }
                if (found) break;
            }
        }
        if (bestK < 0) return false;
        stamp(g, items[idx].rot[bestK], bestX, bestY);
        out[idx] = {bestX, bestY, bestK};
    }
    return true;
}

}  // namespace

bool pack_items(const std::vector<PackItem>& items, int gridW, int gridH, bool allowRotate,
                std::vector<Placement>& out, int* attempts) {
    const size_t n = items.size();
    std::vector<size_t> base(n);
    std::iota(base.begin(), base.end(), 0);
    auto sorted = [&](auto key) {
        std::vector<size_t> o = base;
        std::stable_sort(o.begin(), o.end(), [&](size_t a, size_t b) { return key(a) > key(b); });
        return o;
    };
    auto cells = [&](size_t i) { return (double)items[i].rot[0].cells; };
    auto side = [&](size_t i) { return (double)std::max(items[i].rot[0].w, items[i].rot[0].h); };
    auto box = [&](size_t i) { return (double)items[i].rot[0].w * items[i].rot[0].h; };

    // Greedy passes are cheap and each fails fast, so try several orders and scorings, then
    // jittered orders, and keep the first layout that fits.
    std::vector<std::vector<size_t>> orders = {sorted(cells), sorted(side), sorted(box)};
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> jitter(0.7, 1.3);
    for (int i = 0; i < 40; ++i) {
        std::vector<double> k(n);
        for (size_t j = 0; j < n; ++j) k[j] = cells(j) * jitter(rng);
        orders.push_back(sorted([&](size_t j) { return k[j]; }));
    }
    for (size_t a = 0; a < orders.size(); ++a)
        for (int score = 0; score < 2; ++score)
            if (pack_pass(items, orders[a], gridW, gridH, allowRotate, score, out)) {
                if (attempts) *attempts = (int)(a * 2 + score + 1);
                return true;
            }
    if (attempts) *attempts = (int)orders.size() * 2;
    return false;
}
