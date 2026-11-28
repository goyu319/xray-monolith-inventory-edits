#pragma once
#include <bcrypt.h>
#pragma comment(lib, "bcrypt.lib")

namespace PainterLeafContent {
struct Entry {
    unsigned bytes;
    unsigned char probe[10], source[32], mask[32];
    unsigned maskBytes;
    const char* maskName;
};
static constexpr Entry catalog[] = {
#include "PainterLeafMaskTable.inl"
};
inline bool digest(const void* ptr, unsigned size, unsigned char (&result)[32]) {
    struct Algorithm {
        BCRYPT_ALG_HANDLE handle = nullptr;
        Algorithm() { BCryptOpenAlgorithmProvider(&handle, BCRYPT_SHA256_ALGORITHM, nullptr, 0); }
        ~Algorithm() { if(handle) BCryptCloseAlgorithmProvider(handle, 0); }
    };
    static const Algorithm algorithm;
    if (!algorithm.handle) return false;
    BCRYPT_HASH_HANDLE hash = nullptr;
    if (BCryptCreateHash(algorithm.handle, &hash, nullptr, 0, nullptr, 0, 0) < 0) return false;
    bool ok = BCryptHashData(hash, (PUCHAR)ptr, size, 0) >= 0 &&
              BCryptFinishHash(hash, result, sizeof(result), 0) >= 0;
    BCryptDestroyHash(hash);
    return ok;
}
inline const Entry* identify(const void* bits, unsigned size) {
    if (!bits || !size) return nullptr;
    const auto* data = static_cast<const unsigned char*>(bits);
    unsigned char actual[32];
    bool hashed = false;
    for (const auto& row : catalog) {
        if (row.bytes!=size) continue;
        unsigned i=0;
        for (;i<sizeof(row.probe);++i) {
            const size_t offset=(size_t(size)-1)*(i+1)/11;
            if (data[offset]!=row.probe[i]) break;
        }
        if (i!=sizeof(row.probe)) continue;
        if (!hashed) {
            if (!digest(bits,size,actual)) return nullptr;
            hashed=true;
        }
        if (memcmp(actual,row.source,sizeof(actual))==0) return &row;
    }
    return nullptr;
}
}