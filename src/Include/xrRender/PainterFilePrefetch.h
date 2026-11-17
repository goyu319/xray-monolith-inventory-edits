#pragma once
#include <algorithm>
#include <atomic>
#include <string>
#include <thread>
#include <vector>
namespace painter_prefetch {
inline void files(const std::vector<std::string>& paths, unsigned threads = 16)
{
    if (paths.size() < 64) return;
    std::atomic<std::size_t> next{0};
    auto work = [&] {
        std::vector<char> buffer(std::size_t(1) << 16);
        for (std::size_t i; (i = next.fetch_add(1, std::memory_order_relaxed)) < paths.size();)
        {
            const HANDLE h = CreateFileA(paths[i].c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
                OPEN_EXISTING, FILE_FLAG_SEQUENTIAL_SCAN, nullptr);
            if (h == INVALID_HANDLE_VALUE) continue;
            DWORD got = 0;
            while (ReadFile(h, buffer.data(), DWORD(buffer.size()), &got, nullptr) && got) {}
            CloseHandle(h);
        }
    };
    const unsigned n = std::max(1u, std::min<unsigned>(threads, unsigned((paths.size() + 63) / 64)));
    std::vector<std::thread> pool;
    try { pool.reserve(n); for (unsigned t = 1; t < n; ++t) pool.emplace_back(work); }
    catch (...) {} // fewer helpers: the calling thread still reads everything that is left
    work();
    for (auto& t : pool) t.join();
}
}
