#include <vector>
#include <unordered_map>
#include <cstdint>
#include <algorithm>
#include <array>
#include <chrono>
#include <fmt/core.h>
#include <atomic>

using LayerOuts = std::array<uint8_t, 16>;
using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;

struct LayerInfo {
    uint8_t left_ss;
    uint8_t right_ss;
    bool left_sub;
    bool right_sub;
};

void layer_an(LayerInfo& layer, std::string& str) {
    if (layer.left_sub) str.push_back('*');
    fmt::format_to(std::back_inserter(str), "{},", layer.left_ss);
    if (layer.right_sub) str.push_back('*');
    fmt::format_to(std::back_inserter(str), "{};", layer.right_ss);
}

uint8_t comparator(bool sub, uint8_t base, uint8_t side) {
    uint8_t out;
    if (base >= side) {
        out = base;
        if (sub) {
            out -= side;
        }
    } else {
        out = 0;
    }
    return out;
}

auto layer_outs(LayerInfo& info) {
    LayerOuts outs;
    for (int i = 0; i < 16; i++) {
        uint8_t left = comparator(info.left_sub, i, info.left_ss);
        uint8_t right = comparator(info.right_sub, info.right_ss, i);
        outs[i] = std::max(left, right);
    }
    return outs;
}

struct UniqueLayers {
    std::vector<LayerOuts> lut;
    std::unordered_map<size_t, LayerInfo> layers;
};

void res_an(std::vector<size_t>& res, UniqueLayers& un, std::string& str) {
    for (size_t idx : res) {
        LayerInfo& layer = un.layers[idx];
        layer_an(layer, str);
        str.push_back(' ');
    }
}

UniqueLayers find_unique_layers() {
    UniqueLayers unique_layers;
    for (size_t i = 0; i < 1024; i++) {
        LayerInfo layer_info {
            .left_ss = static_cast<uint8_t>(i >> 6),
            .right_ss = static_cast<uint8_t>((i >> 2) & 0xF),
            .left_sub = (i & 2) != 0,
            .right_sub = (i & 1) != 0,
        };
        auto outs = layer_outs(layer_info);
        if (std::find(unique_layers.lut.begin(), unique_layers.lut.end(), outs) == unique_layers.lut.end()) {
            unique_layers.layers.insert({unique_layers.lut.size(), layer_info});
            unique_layers.lut.push_back(outs);
        }
    }
    return unique_layers;
}

auto compose(LayerOuts& x, LayerOuts& y) {
    LayerOuts out;
    for (int i = 0; i < 16; i++) {
        out[i] = y[x[i]];
    }
    return out;
}

std::atomic_uint64_t end_comparisons = 0;

// Helper function for printing ending statistics and solution that can be called from any thread
void finish(TimePoint start_time) {
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed_time_ms = std::chrono::duration<double, std::milli>(now - start_time).count();
    fmt::println("Done at {}ms", elapsed_time_ms);
    fmt::println("Total end comparisons: {}", end_comparisons.load());
    std::exit(0);
}

bool search(UniqueLayers& ul, int depth, LayerOuts& base, LayerOuts& target, std::vector<size_t>& stack) {
    // Invalid mapping check
    int8_t predicted_mapping[16] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    for (int i = 0; i < 16; i++) {
        uint8_t in = base[i];
        if (predicted_mapping[in] < 0) predicted_mapping[in] = target[i];
        else if (predicted_mapping[in] != target[i]) return false;
    }

    if (depth > 1) {
        for (size_t i = 0; i < ul.lut.size(); i++) {
            auto composed = compose(base, ul.lut[i]);
            if (search(ul, depth - 1, composed, target, stack)) {
                stack.push_back(i);
                return true;
            }
        }
        return false;
    } else {
        for (size_t i = 0; i < ul.lut.size(); i++) {
            auto composed = compose(base, ul.lut[i]);

            // Counting for debugging (may double execution time)
            // end_comparisons.fetch_add(1, std::memory_order_relaxed);

            if (composed == target) {
                // TODO: figure out why removing this print somehow adds 2 seconds
                fmt::println("found!");

                stack.push_back(i);
                return true;
            }
        }
        return false;
    }
}

void bench_depth(UniqueLayers& ul, LayerOuts& base, int depth) {
    fmt::println("Starting benchmark for {} layers", depth);
    auto t_start = std::chrono::high_resolution_clock::now();

    LayerOuts target = {0, 2, 4, 6, 8, 10, 12, 14, 0, 2, 4, 6, 8, 10, 12, 14};
    std::vector<size_t> res;
    search(ul, depth, base, target, res);

    auto t_end = std::chrono::high_resolution_clock::now();

    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
    fmt::println("Searching {} layers took {}ms", depth, elapsed_time_ms);
}

int main() {
    // Repeater
    // LayerOuts target = {0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};

    // Les challenge
    LayerOuts target = {7, 7, 2, 3, 9,10,11, 3, 4, 5, 6, 7, 8, 7, 7,14};

    // Pi
    // LayerOuts target = {3,1,4,1,5,9,2,6,5,3,5,8,9,7,9,3};

    int max_depth = 9;
    int max_threads = 1;

    fmt::print("Starting search for [");
    for (size_t i = 0; i < target.size(); i++) {
        if (i != 0) fmt::print(", ");
        fmt::print("{}", target[i]);
    }
    fmt::print("]\n");
    fmt::println("max_depth: {}, max_threads: {}", max_depth, max_threads);

    UniqueLayers unique_layers = find_unique_layers();
    fmt::println("Found {} unique layers", unique_layers.lut.size());

    LayerOuts base;
    for (int i = 0; i < 16; i++) base[i] = i;

    // bench_depth(unique_layers, base, 3);

    auto start_time = std::chrono::high_resolution_clock::now();
    for (int i = 1; i <= max_depth; i++) {
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(now - start_time).count();

        fmt::println("Searching depth {} ({}ms elapsed)", i, elapsed_time_ms);
        std::vector<size_t> res;
        bool found = search(unique_layers, i, base, target, res);
        if (found) {
            std::reverse(res.begin(), res.end());
            std::string an;
            res_an(res, unique_layers, an);
            fmt::println("Found solution at depth {}: {}", i, an);
            finish(start_time);
        }
    }
    finish(start_time);
}
