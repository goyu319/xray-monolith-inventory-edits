#pragma once
#include <array>
#include <cstdint>

// Private native packet sidecars. Never written to saves or network packets.
namespace painter_stream {
struct Birth {
    std::uint64_t world{},activation{},server_birth{};
    std::uint16_t id{0xffff};
};
struct Stamp {
    Birth owner;
    std::uint64_t arrival{};
    std::array<Birth,4> dependencies{};
    std::uint8_t dependency_count{};
    std::array<Birth,4> optional_dependencies{};
    std::uint8_t optional_count{};
    bool authenticated{},ordinary{},critical{};
};
}
