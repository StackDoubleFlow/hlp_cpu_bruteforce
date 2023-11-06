#include <vector>
#include <unordered_map>
#include <cstdint>
#include <algorithm>
#include <array>
#include <chrono>
#include <fmt/core.h>
#include <atomic>
#include <thread>

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

// Helper function for printing ending statistics that can be called from any thread
void finish(TimePoint start_time) {
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed_time_ms = std::chrono::duration<double, std::milli>(now - start_time).count();
    fmt::println("Done at {}ms", elapsed_time_ms);
    if (end_comparisons.load() > 0)
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

// Search using a range of the depth 1 branches
bool search_range(UniqueLayers& ul, int depth, LayerOuts& base, LayerOuts& target, std::vector<size_t>& stack, size_t start, size_t end) {
    for (size_t i = start; i < end; i++) {
        auto composed = compose(base, ul.lut[i]);
        if (search(ul, depth - 1, composed, target, stack)) {
            stack.push_back(i);
            return true;
        }
    }
    return false;
}

void thread_entry(UniqueLayers& ul, int depth, LayerOuts& base, LayerOuts& target, size_t start, size_t end, TimePoint start_time) {
    std::vector<size_t> res;
    bool found = search_range(ul, depth, base, target, res, 0, ul.lut.size());
    if (found) {
        std::reverse(res.begin(), res.end());
        std::string an;
        res_an(res, ul, an);
        fmt::println("Found solution at depth {}: {}", depth, an);
        finish(start_time);
    }
}

void search_entry(UniqueLayers& ul, int max_depth, int max_threads, LayerOuts& base, LayerOuts& target) {
    size_t work_per_thread = ul.lut.size() / max_threads;
    size_t work_rem = ul.lut.size() % max_threads;

    auto start_time = std::chrono::high_resolution_clock::now();
    for (int i = 1; i <= max_depth; i++) {
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(now - start_time).count();

        fmt::println("Searching depth {} ({}ms elapsed)", i, elapsed_time_ms);
        std::vector<std::thread> threads;
        for (int tid = 0; tid < max_threads; tid++) {
            size_t start = tid * work_per_thread;
            size_t end = start + work_per_thread;
            // We give the last thread the remainder work
            if (tid == max_threads - 1) end += work_rem;

            // fmt::println("starting thread with range {}-{}", start, end);
            threads.emplace_back(thread_entry, std::ref(ul), i, std::ref(base), std::ref(target), start, end, start_time);
        }
        for (auto& thread : threads) {
            thread.join();
        }
    }
    finish(start_time);
}

int main() {
    // Repeater
    // LayerOuts target = {{0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15}};

    // From Amino's test sequences: https://discord.com/channels/721120380424814692/721127492970086483/1019269510336884759
    LayerOuts target = {{7, 7, 2, 3, 9,10,11, 3, 4, 5, 6, 7, 8, 7, 7,14}};

    // Pi
    // LayerOuts target = {{3,1,4,1,5,9,2,6,5,3,5,8,9,7,9,3}};

    int max_depth = 9;
    int max_threads = 25;

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

    search_entry(unique_layers, max_depth, max_threads, base, target);
}
