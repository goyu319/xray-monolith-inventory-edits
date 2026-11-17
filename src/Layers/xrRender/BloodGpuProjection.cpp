#include "stdafx.h"
#include "BloodGpuProjection.h"

#ifdef USE_DX11
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <new>
#include <d3dcompiler.h>

namespace BloodGpuProjection
{
namespace
{
// sourceFirstByte addresses either the ordered index stream or identity vertex
// bytes. The high bit marks indexing; the remaining bits are the vertex base.
constexpr std::uint32_t IndexedSourceBit = 0x80000000u;
static_assert(Backend::MaxSourceBytes < IndexedSourceBit && sizeof(InputVertex) == 60, "source byte address ABI");
struct Group { std::uint32_t sourceFirstByte, candidateFirstTriangle, triangleCount, vertexBaseAndIndexed; };
struct RenderJob { Rows world; std::uint32_t paletteOffset, firstVertex, argsByteOffset, color, dents, pad[3]; }; // melee: + dents
static_assert(sizeof(RenderJob) == 80, "GPU render job ABI");
// melee (2026-11-12, value 3): a body's dents in this batch (8 floats = 2 texels each), one block per body reused by its next
// jobs; code = (dents << 20) | first texel, 0 = none or no room (the caller's job is then refused: its CPU fallback dents too)
constexpr std::uint32_t MaxDentTexels = 65536u;
struct DentTexel { float v[4]; };
struct DentBatch
{
    std::array<DentTexel, MaxDentTexels> texels;
    std::uint32_t count = 0, lastBase = 0, lastDents = 0;
    void Clear() noexcept { count = lastBase = lastDents = 0; }
    std::uint32_t Append(const float* source, std::uint32_t dents) noexcept
    {
        if (!source || !dents || dents > 4095u) return 0;
        const std::uint32_t n = dents * 2u;
        if (lastDents == dents && !std::memcmp(texels.data() + lastBase, source, n * sizeof(DentTexel))) return (dents << 20) | lastBase;
        if (n > MaxDentTexels - count) return 0;
        std::memcpy(texels.data() + count, source, n * sizeof(DentTexel));
        lastBase = count; lastDents = dents; count += n;
        return (dents << 20) | lastBase;
    }
};
static_assert(sizeof(DentTexel) == 16 && MaxDentTexels <= (1u << 20), "dent batch ABI");
struct Scratch { float uv[6]; std::uint32_t rank, accepted; };
struct Constants { Projection projection; Rows world; std::uint32_t groupCount, firstVertex, argsByteOffset, color; };
static_assert(sizeof(Group) == 16 && sizeof(Scratch) == 32 && sizeof(Constants) == 144, "GPU projection constants ABI");
static_assert(sizeof(FVF::LIT) == 24 && offsetof(FVF::LIT, color) == 12 && offsetof(FVF::LIT, t) == 16, "GPU projection vertex ABI");

// Exact region formula transcribed from cl_intersect.h::MgcSqrDistance.
// IEEE strict/precise arithmetic prevents reassociation; CPU/GPU rounding can
// still differ at a floating-point boundary and is measured by offline tests.
constexpr char ShaderSource[] = R"BLOOD_PROJECT(

struct InputVertex { float3 p; float2 uv; uint4 ids; float3 weights; uint links; uint2 ownership; };
struct Rows { float4 x; float4 y; float4 z; };
struct Group { uint sourceFirstByte; uint candidateFirstTriangle; uint triangleCount; uint vertexBaseAndIndexed; };
struct RenderJob { Rows world; uint paletteOffset; uint firstVertex; uint argsByteOffset; uint color; uint dents; uint3 pad; };
struct ProjectedVertex { uint sourceIndex; float2 uv; };
struct Scratch { float2 uv0; float2 uv1; float2 uv2; uint rank; uint accepted; };
ByteAddressBuffer source : register(t0);
StructuredBuffer<Rows> bones : register(t1);
StructuredBuffer<Group> groups : register(t2);
StructuredBuffer<ProjectedVertex> generated : register(t3);
ByteAddressBuffer arguments : register(t4);
StructuredBuffer<RenderJob> renderJobs : register(t5);
StructuredBuffer<float4> dentTexels : register(t6); // melee (value 3): a render job's dents
RWStructuredBuffer<Scratch> scratch : register(u0);
RWStructuredBuffer<uint> groupCounts : register(u1);
RWStructuredBuffer<ProjectedVertex> generatedOutput : register(u2);
RWByteAddressBuffer argumentsOutput : register(u3);
RWByteAddressBuffer verticesOutput : register(u4);
RWStructuredBuffer<uint> renderGroupEnds : register(u5);
RWByteAddressBuffer renderArguments : register(u6);
cbuffer Constants : register(b0) {
    Rows projectionView;
    float3 contact; float radius;
    float3 normal; float unused;
    Rows world;
    uint groupCount; uint destinationFirstVertex; uint argsByteOffset; uint color;
};
float dot3(float3 a, float3 b) {
    precise float value = a.x * b.x + a.y * b.y + a.z * b.z;
    return value;
}
InputVertex load_vertex(uint address) {
    InputVertex v;
    v.p = asfloat(source.Load3(address));
    v.uv = asfloat(source.Load2(address + 12));
    v.ids = source.Load4(address + 20);
    v.weights = asfloat(source.Load3(address + 36));
    v.links = source.Load(address + 48);
    v.ownership=source.Load2(address+52);
    return v;
}
uint source_address(Group g, uint logicalVertex) {
    if (g.vertexBaseAndIndexed & 0x80000000u)
        return (g.vertexBaseAndIndexed & 0x7fffffffu) + source.Load(g.sourceFirstByte + logicalVertex * 4) * 60;
    return g.sourceFirstByte + logicalVertex * 60;
}
float3 transform_point(Rows m, float3 p) {
    precise float3 result;
    result.x = p.x * m.x.x + p.y * m.x.y + p.z * m.x.z + m.x.w;
    result.y = p.x * m.y.x + p.y * m.y.y + p.z * m.y.z + m.y.w;
    result.z = p.x * m.z.x + p.y * m.z.y + p.z * m.z.z + m.z.w;
    return result;
}
float3 skin_point(InputVertex v, bool creation, uint paletteBase) {
    uint links = v.links; // the registered creation link count (1..4), never guessed from repeated ids
    precise float3 p0 = transform_point(bones[paletteBase + v.ids.x], v.p);
    if (links == 1) return p0;
    precise float3 p1 = transform_point(bones[paletteBase + v.ids.y], v.p);
    if (links == 2) {
        precise float inverse = 1.0f - v.weights.x;
        precise float3 p = p0 * inverse + p1 * v.weights.x;
        return p;
    }
    precise float3 p2 = transform_point(bones[paletteBase + v.ids.z], v.p);
    p0 *= v.weights.x; p1 *= v.weights.y;
    if (links == 3) {
        p2 *= (1.0f - v.weights.x - v.weights.y);
        precise float3 p = p0; p += p1; p += p2;
        return p;
    }
    precise float3 p3 = transform_point(bones[paletteBase + v.ids.w], v.p);
    p2 *= v.weights.z;
    precise float remainder;
    if (creation) remainder = 1.0f - v.weights.x - v.weights.y - v.weights.z;
    else {
        precise float sum = 0.0f;
        sum += v.weights.x; sum += v.weights.y; sum += v.weights.z;
        remainder = 1.0f - sum;
    }
    p3 *= remainder;
    precise float3 p = p0; p += p1; p += p2; p += p3;
    return p;
}
	float distance_squared(float3 rkPoint, float3 orig, float3 e0, float3 e1)
	{
		precise float3 kDiff = orig - rkPoint;

		precise float fA00 = dot3(e0, e0);
		precise float fA01 = dot3(e0, e1);
		precise float fA11 = dot3(e1, e1);
		precise float fB0 = dot3(kDiff, e0);
		precise float fB1 = dot3(kDiff, e1);
		precise float fC = dot3(kDiff, kDiff);
		precise float fDet = abs(fA00 * fA11 - fA01 * fA01);
		precise float fS = fA01 * fB1 - fA11 * fB0;
		precise float fT = fA01 * fB0 - fA00 * fB1;
		precise float fSqrDist;

		if (fS + fT <= fDet)
		{
			if (fS < 0.0f)
			{
				if (fT < 0.0f)
				{
					// region 4
					if (fB0 < 0.0f)
					{
						fT = 0.0f;
						if (-fB0 >= fA00)
						{
							fS = 1.0f;
							fSqrDist = fA00 + 2.0f * fB0 + fC;
						}
						else
						{
							fS = -fB0 / fA00;
							fSqrDist = fB0 * fS + fC;
						}
					}
					else
					{
						fS = 0.0f;
						if (fB1 >= 0.0f)
						{
							fT = 0.0f;
							fSqrDist = fC;
						}
						else if (-fB1 >= fA11)
						{
							fT = 1.0f;
							fSqrDist = fA11 + 2.0f * fB1 + fC;
						}
						else
						{
							fT = -fB1 / fA11;
							fSqrDist = fB1 * fT + fC;
						}
					}
				}
				else
				{
					// region 3
					fS = 0.0f;
					if (fB1 >= 0.0f)
					{
						fT = 0.0f;
						fSqrDist = fC;
					}
					else if (-fB1 >= fA11)
					{
						fT = 1;
						fSqrDist = fA11 + 2.0f * fB1 + fC;
					}
					else
					{
						fT = -fB1 / fA11;
						fSqrDist = fB1 * fT + fC;
					}
				}
			}
			else if (fT < 0.0f)
			{
				// region 5
				fT = 0.0f;
				if (fB0 >= 0.0f)
				{
					fS = 0.0f;
					fSqrDist = fC;
				}
				else if (-fB0 >= fA00)
				{
					fS = 1.0;
					fSqrDist = fA00 + 2.0f * fB0 + fC;
				}
				else
				{
					fS = -fB0 / fA00;
					fSqrDist = fB0 * fS + fC;
				}
			}
			else
			{
				// region 0
				// minimum at interior point
				precise float fInvDet = 1.0f / fDet;
				fS *= fInvDet;
				fT *= fInvDet;
				fSqrDist = fS * (fA00 * fS + fA01 * fT + 2.0f * fB0) +
					fT * (fA01 * fS + fA11 * fT + 2.0f * fB1) + fC;
			}
		}
		else
		{
			precise float fTmp0, fTmp1, fNumer, fDenom;

			if (fS < 0.0f)
			{
				// region 2
				fTmp0 = fA01 + fB0;
				fTmp1 = fA11 + fB1;
				if (fTmp1 > fTmp0)
				{
					fNumer = fTmp1 - fTmp0;
					fDenom = fA00 - 2.0f * fA01 + fA11;
					if (fNumer >= fDenom)
					{
						fS = 1.0f;
						fT = 0.0f;
						fSqrDist = fA00 + 2.0f * fB0 + fC;
					}
					else
					{
						fS = fNumer / fDenom;
						fT = 1.0f - fS;
						fSqrDist = fS * (fA00 * fS + fA01 * fT + 2.0f * fB0) +
							fT * (fA01 * fS + fA11 * fT + 2.0f * fB1) + fC;
					}
				}
				else
				{
					fS = 0.0f;
					if (fTmp1 <= 0.0f)
					{
						fT = 1.0f;
						fSqrDist = fA11 + 2.0f * fB1 + fC;
					}
					else if (fB1 >= 0.0f)
					{
						fT = 0.0f;
						fSqrDist = fC;
					}
					else
					{
						fT = -fB1 / fA11;
						fSqrDist = fB1 * fT + fC;
					}
				}
			}
			else if (fT < 0.0)
			{
				// region 6
				fTmp0 = fA01 + fB1;
				fTmp1 = fA00 + fB0;
				if (fTmp1 > fTmp0)
				{
					fNumer = fTmp1 - fTmp0;
					fDenom = fA00 - 2.0f * fA01 + fA11;
					if (fNumer >= fDenom)
					{
						fT = 1.0f;
						fS = 0.0f;
						fSqrDist = fA11 + 2.0f * fB1 + fC;
					}
					else
					{
						fT = fNumer / fDenom;
						fS = 1.0f - fT;
						fSqrDist = fS * (fA00 * fS + fA01 * fT + 2.0f * fB0) +
							fT * (fA01 * fS + fA11 * fT + 2.0f * fB1) + fC;
					}
				}
				else
				{
					fT = 0.0f;
					if (fTmp1 <= 0.0f)
					{
						fS = 1.0f;
						fSqrDist = fA00 + 2.0f * fB0 + fC;
					}
					else if (fB0 >= 0.0f)
					{
						fS = 0.0f;
						fSqrDist = fC;
					}
					else
					{
						fS = -fB0 / fA00;
						fSqrDist = fB0 * fS + fC;
					}
				}
			}
			else
			{
				// region 1
				fNumer = fA11 + fB1 - fA01 - fB0;
				if (fNumer <= 0.0f)
				{
					fS = 0.0f;
					fT = 1.0f;
					fSqrDist = fA11 + 2.0f * fB1 + fC;
				}
				else
				{
					fDenom = fA00 - 2.0f * fA01 + fA11;
					if (fNumer >= fDenom)
					{
						fS = 1.0f;
						fT = 0.0f;
						fSqrDist = fA00 + 2.0f * fB0 + fC;
					}
					else
					{
						fS = fNumer / fDenom;
						fT = 1.0f - fS;
						fSqrDist = fS * (fA00 * fS + fA01 * fT + 2.0f * fB0) +
							fT * (fA01 * fS + fA11 * fT + 2.0f * fB1) + fC;
					}
				}
			}
		}

		return abs(fSqrDist);
	}

bool sphere_triangle(float3 p0, float3 p1, float3 p2) {
    precise float r2 = radius * radius;
    precise float3 e0 = p1 - p0, e1 = p2 - p0, delta = p0 - contact;
    if (dot3(delta, delta) <= r2) return true;
    precise float3 d1 = delta + e0, d2 = delta + e1;
    if (dot3(d1, d1) <= r2 || dot3(d2, d2) <= r2) return true;
    return distance_squared(contact, p0, e0, e1) < r2;
}
// Mirrors BloodProjectionFootprint.h; admission only, never UV rescaling.
bool footprint_separated(float3 a, float3 p0, float3 p1, float3 p2) {
    precise float p = dot3(p0,a), q = dot3(p1,a), r = dot3(p2,a);
    precise float extent = abs(a.x) + abs(a.y) + 1.1111112f*abs(a.z);
    return min(p,min(q,r)) > extent || max(p,max(q,r)) < -extent;
}
bool projector_triangle(float3 source0, float3 source1, float3 source2) {
    precise float3 p0 = transform_point(projectionView,source0);
    precise float3 p1 = transform_point(projectionView,source1);
    precise float3 p2 = transform_point(projectionView,source2);
    if (!all(isfinite(p0)) || !all(isfinite(p1)) || !all(isfinite(p2))) return false;
    if (min(p0.x,min(p1.x,p2.x)) > 1.f || max(p0.x,max(p1.x,p2.x)) < -1.f ||
        min(p0.y,min(p1.y,p2.y)) > 1.f || max(p0.y,max(p1.y,p2.y)) < -1.f ||
        min(p0.z,min(p1.z,p2.z)) > 1.1111112f || max(p0.z,max(p1.z,p2.z)) < -1.1111112f) return false;
    precise float3 edges[3]; edges[0]=p1-p0; edges[1]=p2-p1; edges[2]=p0-p2;
    precise float3 axis = cross(edges[0],edges[1]);
    if (footprint_separated(axis,p0,p1,p2)) return false;
    [unroll] for (uint i=0; i<3; ++i) {
        precise float3 e=edges[i];
        if (footprint_separated(float3(0.f,-e.z,e.y),p0,p1,p2)) return false;
        if (footprint_separated(float3(e.z,0.f,-e.x),p0,p1,p2)) return false;
        if (footprint_separated(float3(-e.y,e.x,0.f),p0,p1,p2)) return false;
    }
    return true;
}
float2 project_uv(float3 p) {
    precise float3 v = transform_point(projectionView, p);
    precise float2 uv = float2((1.0f + v.x) * 0.5f, (1.0f - v.y) * 0.5f);
    return uv;
}
)BLOOD_PROJECT"
// Keep individual raw literals below MSVC's limit; concatenation is byte exact.
R"BLOOD_PROJECT(// melee (2026-11-12, value 3): the bodies' dents (2 texels each, rest space); a job's `dents` = (count << 20) | first texel
static uint dent_n = 0, dent_base = 0;
#define PAINTER_DENT_COUNT dent_n
#define PAINTER_DENT_TEXEL(i) dentTexels[dent_base + (i)]
// melee dent displacement - ONE text, used by painter_dent.h (the exterior's vertex programs) and by the two wallmark skinning kernels
// (BloodGpuSkinning.cpp, BloodGpuProjection.cpp: the blood decals on a dented body), so exterior and decals move by the same function
// of the rest position. The includer defines PAINTER_DENT_COUNT (dents) and PAINTER_DENT_TEXEL(i) (float4 texel i: 2 per dent,
// centre + radius, push + depth; rest space). The C++ twin (CPU wallmarks) is Include/xrRender/PainterDent.h.
// A dent pushes the surface in along the blow (push) by depth * (1 - x^2)^2 (x = distance / radius: a bowl with zero slope at its
// rim) * g, g = saturate(1 - (p - centre) . push / (0.55 radius)): points the blow would have to pass through first (the far side
// and the flanks of a thin part) fade out, so no part is pushed through itself. Returns the displacement; `tilt` (for the normal,
// n' = normalize(n - tilt)) is the depth's slope along the surface times how much the push runs along the normal.
float3 painter_dent_offset(float3 p, float3 n, out float3 tilt)
{
    float3 disp = 0;
    tilt = 0;
    [loop] for (uint i = 0; i < PAINTER_DENT_COUNT; ++i)
    {
        const float4 a = PAINTER_DENT_TEXEL(2 * i);
        const float4 q = PAINTER_DENT_TEXEL(2 * i + 1);
        const float3 d = p - a.xyz;
        const float r = a.w, d2 = dot(d, d);
        [branch] if (!(r > 0) || d2 >= r * r) continue;
        const float x2 = d2 / (r * r), f = (1 - x2) * (1 - x2);
        const float gr = 1 - dot(d, q.xyz) / (0.55 * r), g = saturate(gr);
        disp += q.xyz * (q.w * f * g);
        const float3 gh = q.w * (d * (-4 * (1 - x2) * g / (r * r)) - q.xyz * ((gr > 0 && gr < 1) ? f / (0.55 * r) : 0));
        tilt += (gh - n * dot(gh, n)) * dot(q.xyz, n);
    }
    return disp;
}
InputVertex dented(InputVertex v) { float3 tilt; v.p += painter_dent_offset(v.p, float3(0, 0, 1), tilt); return v; }
)BLOOD_PROJECT"
R"BLOOD_PROJECT(groupshared uint prefix[2048];
groupshared uint renderPrefix[4096];
[numthreads(128, 1, 1)]
void project(uint3 gid : SV_GroupID, uint3 tid : SV_GroupThreadID) {
    Group g = groups[gid.x];
    Scratch s = (Scratch)0;
    if (tid.x < g.triangleCount) {
        uint i = tid.x * 3;
        precise float3 p0 = skin_point(load_vertex(source_address(g, i)), true, 0);
        precise float3 p1 = skin_point(load_vertex(source_address(g, i + 1)), true, 0);
        precise float3 p2 = skin_point(load_vertex(source_address(g, i + 2)), true, 0);
        precise float3 e01 = p1 - p0, e12 = p2 - p1;
        precise float3 n;
        n.x = e01.y * e12.z - e01.z * e12.y;
        n.y = e01.z * e12.x - e01.x * e12.z;
        n.z = e01.x * e12.y - e01.y * e12.x;
        precise float magnitude = dot3(n, n);
        if (magnitude > 1.175494351e-38f) n *= sqrt(1.0f / magnitude);
        precise float facing = dot3(n, normal);
        // Outer surface only (BloodProjectionFootprint::Outer): a corner within 2-4 cm behind the contact, or within the
        // surface call's own limit, sent as |unused| = 2 + limit (unused 0/1 = no own limit).
        precise float outerLimit = abs(unused) > 1.5f ? abs(unused) - 2.0f : max(0.02f, min(0.04f, radius * 0.35f));
        bool outer = min(dot3(contact - p0, normal), min(dot3(contact - p1, normal), dot3(contact - p2, normal))) <= outerLimit;
        if (!(facing < 0.0000100f) && outer && (unused > 0.5f ? projector_triangle(p0, p1, p2) : sphere_triangle(p0, p1, p2))) {
            s.accepted = 1;
            s.uv0 = project_uv(p0); s.uv1 = project_uv(p1); s.uv2 = project_uv(p2);
        }
    }
    prefix[tid.x] = s.accepted;
    GroupMemoryBarrierWithGroupSync();
    [unroll] for (uint step = 1; step < 128; step <<= 1) {
        uint add = tid.x >= step ? prefix[tid.x - step] : 0;
        GroupMemoryBarrierWithGroupSync();
        prefix[tid.x] += add;
        GroupMemoryBarrierWithGroupSync();
    }
    s.rank = prefix[tid.x] - s.accepted;
    if (tid.x < g.triangleCount) scratch[g.candidateFirstTriangle + tid.x] = s;
    if (tid.x == 127) groupCounts[gid.x] = prefix[127];
}
[numthreads(1024, 1, 1)]
void scan(uint3 tid : SV_GroupThreadID) {
    uint first = tid.x, second = tid.x + 1024;
    uint count0 = first < groupCount ? groupCounts[first] : 0;
    uint count1 = second < groupCount ? groupCounts[second] : 0;
    prefix[first] = count0; prefix[second] = count1;
    GroupMemoryBarrierWithGroupSync();
    [unroll] for (uint step = 1; step < 2048; step <<= 1) {
        uint add0 = first >= step ? prefix[(first - step) & 2047] : 0;
        uint add1 = second >= step ? prefix[(second - step) & 2047] : 0;
        GroupMemoryBarrierWithGroupSync();
        prefix[first] += add0; prefix[second] += add1;
        GroupMemoryBarrierWithGroupSync();
    }
    if (first < groupCount) groupCounts[first] = prefix[first] - count0;
    if (second < groupCount) groupCounts[second] = prefix[second] - count1;
    if (tid.x == 1023) {
        uint vertexCount = prefix[2047] * 3;
        argumentsOutput.Store4(argsByteOffset, uint4(vertexCount, 1, destinationFirstVertex, 0));
        argumentsOutput.Store4(argsByteOffset + 16, uint4((vertexCount + 255) / 256, 1, 1, 0));
    }
}
[numthreads(128, 1, 1)]
void compact(uint3 gid : SV_GroupID, uint3 tid : SV_GroupThreadID) {
    Group g = groups[gid.x];
    if (tid.x >= g.triangleCount) return;
    Scratch s = scratch[g.candidateFirstTriangle + tid.x];
    if (!s.accepted) return;
    uint destination = destinationFirstVertex + (groupCounts[gid.x] + s.rank) * 3;
    uint first = tid.x * 3;
    ProjectedVertex v; v.sourceIndex = source_address(g, first); v.uv = s.uv0; generatedOutput[destination] = v;
    v.sourceIndex = source_address(g, first + 1); v.uv = s.uv1; generatedOutput[destination + 1] = v;
    v.sourceIndex = source_address(g, first + 2); v.uv = s.uv2; generatedOutput[destination + 2] = v;
}
[numthreads(256, 1, 1)]
void skin(uint3 tid : SV_DispatchThreadID) {
    uint count = arguments.Load(argsByteOffset);
    if (tid.x >= count) return;
    uint vertex = destinationFirstVertex + tid.x;
    ProjectedVertex projected = generated[vertex];
    InputVertex v = load_vertex(projected.sourceIndex);
    v.uv = projected.uv;
    precise float3 p = transform_point(world, skin_point(v, false, 0));
    uint address = vertex * 44;
    verticesOutput.Store3(address, asuint(p));
    verticesOutput.Store(address + 12, color);
    verticesOutput.Store2(address + 16, asuint(v.uv));
    verticesOutput.Store3(address + 24, asuint(v.p));
    verticesOutput.Store2(address+36,asuint(v.ownership.x==0xffffffffu?float2(-1,0):float2(v.ownership)));
}

[numthreads(1024, 1, 1)]
void scan_render(uint3 tid : SV_GroupThreadID) {
    [unroll] for (uint part = 0; part < 4; ++part) {
        uint i = tid.x + part * 1024;
        renderPrefix[i] = i < groupCount ? (arguments.Load(renderJobs[i].argsByteOffset) + 255) / 256 : 0;
    }
    GroupMemoryBarrierWithGroupSync();
    [unroll] for (uint step = 1; step < 4096; step <<= 1) {
        uint adds[4];
        [unroll] for (uint part = 0; part < 4; ++part) {
            uint i = tid.x + part * 1024;
            adds[part] = i >= step ? renderPrefix[(i - step) & 4095] : 0;
        }
        GroupMemoryBarrierWithGroupSync();
        [unroll] for (uint writePart = 0; writePart < 4; ++writePart) renderPrefix[tid.x + writePart * 1024] += adds[writePart];
        GroupMemoryBarrierWithGroupSync();
    }
    [unroll] for (uint outputPart = 0; outputPart < 4; ++outputPart) {
        uint i = tid.x + outputPart * 1024;
        if (i < groupCount) renderGroupEnds[i] = renderPrefix[i];
    }
    if (tid.x == 1023) renderArguments.Store4(0, uint4(renderPrefix[4095], 1, 1, 0));
}
[numthreads(256, 1, 1)]
void skin_batch(uint3 gid : SV_GroupID, uint3 tid : SV_GroupThreadID) {
    uint lo = 0, hi = groupCount;
    while (lo < hi) {
        uint middle = lo + (hi - lo) / 2;
        if (renderGroupEnds[middle] <= gid.x) lo = middle + 1; else hi = middle;
    }
    if (lo >= groupCount) return;
    RenderJob job = renderJobs[lo];
    uint firstGroup = lo ? renderGroupEnds[lo - 1] : 0;
    uint index = (gid.x - firstGroup) * 256 + tid.x;
    if (index >= arguments.Load(job.argsByteOffset)) return;
    uint vertex = job.firstVertex + index;
    ProjectedVertex projected = generated[vertex];
    InputVertex v = load_vertex(projected.sourceIndex);
    dent_n = job.dents >> 20; dent_base = job.dents & 0xFFFFFu; if (dent_n) v = dented(v); // melee (value 3): on the body's dents
    precise float3 p = transform_point(job.world, skin_point(v, false, job.paletteOffset));
    uint address = vertex * 44;
    verticesOutput.Store3(address, asuint(p));
    verticesOutput.Store(address + 12, job.color);
    verticesOutput.Store2(address + 16, asuint(projected.uv));
    verticesOutput.Store3(address + 24, asuint(v.p));
    verticesOutput.Store2(address+36,asuint(v.ownership.x==0xffffffffu?float2(-1,0):float2(v.ownership)));
}
[numthreads(256, 1, 1)]
void relocate(uint3 tid : SV_DispatchThreadID) {
    // groupCount is vertex count; color is new first vertex in this copy pass.
    // Source and destination ranges are disjoint and destination is earlier.
    if (tid.x >= groupCount) return;
    uint from = destinationFirstVertex + tid.x, to = color + tid.x;
    generatedOutput[to] = generatedOutput[from];
    if (radius != 0.0f) {
        uint4 a = verticesOutput.Load4(from * 44);
        uint2 b = verticesOutput.Load2(from * 44 + 16);
        uint3 rest = verticesOutput.Load3(from * 44 + 24);
        uint2 owner = verticesOutput.Load2(from * 44 + 36);
        verticesOutput.Store4(to * 44, a);
        verticesOutput.Store2(to * 44 + 16, b);
        verticesOutput.Store3(to * 44 + 24, rest);
        verticesOutput.Store2(to * 44+36,owner);
    }
    if (tid.x == 0) argumentsOutput.Store(argsByteOffset + 8, color);
}
)BLOOD_PROJECT";

template <class T> void Release(T*& value) noexcept
{
    if (value) { value->Release(); value = nullptr; }
}
struct Buffer
{
    ID3D11Buffer* buffer = nullptr;
    ID3D11ShaderResourceView* srv = nullptr;
    ID3D11UnorderedAccessView* uav = nullptr;
    ~Buffer() { Release(srv); Release(uav); Release(buffer); }
};
struct Span { std::uint32_t first, count; };
struct Slot
{
    std::uint32_t first = 0, count = 0, generation = 0, paletteNeeded = 0, references = 0, capacity = 0;
    bool active = false, rendered = false, sourceOwned = false, trimmed = false;
};
template <std::size_t N, std::uint32_t Capacity> struct Atlas
{
    std::array<Slot, N> slots = {};
    std::array<Span, N + 1> free = {};
    std::uint32_t freeCount = 1;
    Atlas() noexcept { free[0] = {0, Capacity}; }
    bool Find(std::uint32_t count, std::uint32_t& slot, std::uint32_t& span) const noexcept
    {
        if (!count || count > Capacity) return false;
        slot = static_cast<std::uint32_t>(N); span = freeCount;
        for (std::uint32_t i = 0; i < N; ++i)
            if (!slots[i].active && slots[i].generation != (std::numeric_limits<std::uint32_t>::max)()) { slot = i; break; }
        for (std::uint32_t i = 0; i < freeCount; ++i)
            if (free[i].count >= count) { span = i; break; }
        return slot < N && span < freeCount;
    }
    void Allocate(std::uint32_t slot, std::uint32_t span, std::uint32_t count, std::uint32_t palette) noexcept
    {
        Slot& s = slots[slot];
        s.first = free[span].first; s.count = s.capacity = count; s.paletteNeeded = palette; s.trimmed = false;
        s.active = true; s.rendered = false; ++s.generation;
        free[span].first += count; free[span].count -= count;
        if (!free[span].count)
        {
            for (std::uint32_t i = span + 1; i < freeCount; ++i) free[i - 1] = free[i];
            --freeCount;
        }
    }
    void Return(std::uint32_t first, std::uint32_t count) noexcept
    {
        if (!count) return;
        std::uint32_t at = 0;
        while (at < freeCount && free[at].first < first) ++at;
        const bool left = at && free[at - 1].first + free[at - 1].count == first;
        const bool right = at < freeCount && first + count == free[at].first;
        // Coalesce BEFORE insertion. A fully fragmented atlas may already have
        // N+1 free intervals; release then bridges neighbors without a transient
        // N+2nd interval or out-of-bounds array write.
        if (left)
        {
            free[at - 1].count += count;
            if (right)
            {
                free[at - 1].count += free[at].count;
                for (std::uint32_t i = at + 1; i < freeCount; ++i) free[i - 1] = free[i];
                --freeCount;
            }
        }
        else if (right) { free[at].first = first; free[at].count += count; }
        else
        {
            // A new isolated gap consumes/splits an occupied span; the number
            // of disjoint gaps can therefore never exceed the N+1 bound.
            for (std::uint32_t i = freeCount; i > at; --i) free[i] = free[i - 1];
            free[at] = {first, count}; ++freeCount;
        }
    }
    void Free(std::uint32_t slot) noexcept
    {
        Slot& s = slots[slot]; Return(s.first, s.count);
        s.active = s.rendered = s.sourceOwned = false;
    }
    void MoveEarlier(std::uint32_t slot, std::uint32_t span) noexcept
    {
        Slot& s = slots[slot]; const std::uint32_t oldFirst = s.first;
        s.first = free[span].first; free[span].first += s.count; free[span].count -= s.count;
        if (!free[span].count)
        {
            for (std::uint32_t i = span + 1; i < freeCount; ++i) free[i - 1] = free[i];
            --freeCount;
        }
        Return(oldFirst, s.count);
    }
    void Shrink(std::uint32_t slot, std::uint32_t count) noexcept
    {
        Slot& s = slots[slot];
        Return(s.first + count, s.count - count); s.count = count; s.trimmed = true;
    }
};
bool Finite(const Rows& m) noexcept
{
    for (unsigned i = 0; i < 4; ++i)
        if (!std::isfinite(m.x[i]) || !std::isfinite(m.y[i]) || !std::isfinite(m.z[i])) return false;
    return true;
}
bool SourceBytes(std::uint32_t vertexCount, std::uint32_t indexCount, bool indexed, std::uint32_t& bytes) noexcept
{
    if (!vertexCount || vertexCount > Backend::MaxSourceVertices) return false;
    if (indexed)
    {
        if (!indexCount || indexCount > Backend::MaxSourceIndices || indexCount % 3) return false;
    }
    else if (indexCount || vertexCount % 3) return false;
    const std::uint64_t total = std::uint64_t(vertexCount) * sizeof(InputVertex) +
        std::uint64_t(indexCount) * sizeof(std::uint32_t);
    if (total > Backend::MaxSourceBytes) return false;
    bytes = static_cast<std::uint32_t>(total);
    return true;
}
bool FinitePalette(const Rows* palette, std::uint32_t count) noexcept
{
    if (!palette || !count || count > Backend::MaxPaletteBones) return false;
    for (std::uint32_t i = 0; i < count; ++i) if (!Finite(palette[i])) return false;
    return true;
}
void Upload(ID3D11DeviceContext* ctx, ID3D11Buffer* buffer, UINT first, UINT bytes, const void* data) noexcept
{
    D3D11_BOX box = {first, 0, 0, first + bytes, 1, 1};
    ctx->UpdateSubresource(buffer, 0, &box, data, 0, 0);
}

// Preserve actual bindings and leave RCache's mirrors unchanged. Get* obtains
// COM references, not device completion or GPU data. The resources used here
// can alias IA slot zero from the immediately preceding wallmark draw.
class SavedComputeState
{
public:
    explicit SavedComputeState(ID3D11DeviceContext* context) noexcept : ctx(context)
    {
        instanceCount = static_cast<UINT>(instances.size());
        ctx->CSGetShader(&shader, instances.data(), &instanceCount);
        ctx->CSGetShaderResources(0, 7, views.data()); // melee: t0-t6
        ctx->CSGetUnorderedAccessViews(0, 7, outputs.data());
        ctx->CSGetConstantBuffers(0, 1, &constants);
        ctx->IAGetVertexBuffers(0, 1, &vertices, &stride, &offset);
        ID3D11Buffer* none = nullptr; UINT zero = 0;
        ctx->IASetVertexBuffers(0, 1, &none, &zero, &zero);
        Unbind();
    }
    void Unbind() noexcept
    {
        ID3D11ShaderResourceView* noViews[7] = {};
        ID3D11UnorderedAccessView* noOutputs[7] = {};
        ctx->CSSetShaderResources(0, 7, noViews);
        ctx->CSSetUnorderedAccessViews(0, 7, noOutputs, nullptr);
    }
    ~SavedComputeState()
    {
        Unbind();
        ctx->CSSetShader(shader, instances.data(), instanceCount);
        ctx->CSSetConstantBuffers(0, 1, &constants);
        ctx->CSSetShaderResources(0, 7, views.data());
        const UINT preserve[7] = {UINT(-1), UINT(-1), UINT(-1), UINT(-1), UINT(-1), UINT(-1), UINT(-1)};
        ctx->CSSetUnorderedAccessViews(0, 7, outputs.data(), preserve);
        ctx->IASetVertexBuffers(0, 1, &vertices, &stride, &offset);
        Release(shader); Release(constants); Release(vertices);
        for (auto*& view : views) Release(view);
        for (auto*& output : outputs) Release(output);
        for (UINT i = 0; i < instanceCount; ++i) Release(instances[i]);
    }
private:
    ID3D11DeviceContext* ctx;
    ID3D11ComputeShader* shader = nullptr;
    std::array<ID3D11ClassInstance*, D3D11_SHADER_MAX_INTERFACES> instances = {};
    std::array<ID3D11ShaderResourceView*, 7> views = {};
    std::array<ID3D11UnorderedAccessView*, 7> outputs = {};
    UINT instanceCount = 0, stride = 0, offset = 0;
    ID3D11Buffer *constants = nullptr, *vertices = nullptr;
};
}

struct Backend::State
{
    ID3D11Device* device = nullptr;
    std::array<ID3D11ComputeShader*, 7> shaders = {};
    // sourceIndex is an absolute byte address of the immutable unique vertex.
    // Only generated spans move; source allocations stay pinned until release.
    struct ProjectedVertex { std::uint32_t sourceIndex; float uv[2]; };
    static_assert(sizeof(ProjectedVertex) == 12, "GPU compact projection ABI");
    Buffer source, bones, groups, scratch, groupCounts, generated, arguments, vertices, constants;
    Buffer renderJobs, renderGroupEnds, renderArguments, allocationCopy;
    Buffer dents; DentBatch renderDents; // melee (value 3): the render batch's dents (t6)
    Atlas<MaxSources, MaxSourceBytes> sourceAtlas;
    struct SourceInfo
    {
        std::uint32_t vertexCount = 0, indexCount = 0;
        std::uint32_t LogicalCount() const noexcept { return indexCount ? indexCount : vertexCount; }
    };
    std::array<SourceInfo, MaxSources> sourceInfo = {};
    Atlas<MaxMarks, MaxGeneratedVertices> markAtlas;
    std::array<Group, MaxGroups> cpuGroups = {};
    std::uint64_t bytes = 0;
    std::array<Rows, MaxBatchBones> renderBones;
    std::array<RenderJob, MaxBatchJobs> cpuRenderJobs;
    std::array<MarkHandle, MaxBatchJobs> renderMarks;
    std::array<bool, MaxMarks> renderQueued = {};
    std::uint32_t renderBoneCount = 0, renderJobCount = 0;
    std::array<std::uint32_t, MaxMarks> allocationGenerations = {};
    bool allocationPending = false;
    std::uint32_t untrimmedMarks = 0;
    std::array<std::uint32_t*, MaxMarks> markSources = {};
    std::array<std::uint32_t, MaxMarks> markSourceCounts = {};

    ~State()
    {
        for (auto*& shader : shaders) Release(shader);
        for (auto* sources : markSources) std::free(sources);
    }
    void DropSource(std::uint32_t index, Counters& counters) noexcept
    {
        Slot& slot = sourceAtlas.slots[index];
        if (--slot.references == 0)
        {
            --counters.live_sources;
            counters.live_source_vertices -= sourceInfo[index].vertexCount;
            counters.live_source_indices -= sourceInfo[index].indexCount;
            counters.live_source_bytes -= slot.count;
            sourceAtlas.Free(index);
            sourceInfo[index] = {};
        }
    }
    HRESULT CreateBuffer(Buffer& b, UINT byteCount, UINT bind, UINT misc, UINT stride = 0) noexcept
    {
        D3D11_BUFFER_DESC desc = {};
        desc.ByteWidth = byteCount; desc.Usage = D3D11_USAGE_DEFAULT;
        desc.BindFlags = bind; desc.MiscFlags = misc; desc.StructureByteStride = stride;
        HRESULT hr = device->CreateBuffer(&desc, nullptr, &b.buffer);
        if (FAILED(hr)) return hr;
        bytes += byteCount;
        const bool raw = (misc & D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS) != 0;
        if (bind & D3D11_BIND_SHADER_RESOURCE)
        {
            D3D11_SHADER_RESOURCE_VIEW_DESC desc = {};
            if (raw)
            {
                desc.Format = DXGI_FORMAT_R32_TYPELESS; desc.ViewDimension = D3D11_SRV_DIMENSION_BUFFEREX;
                desc.BufferEx.NumElements = byteCount / 4; desc.BufferEx.Flags = D3D11_BUFFEREX_SRV_FLAG_RAW;
            }
            else
            {
                desc.Format = DXGI_FORMAT_UNKNOWN; desc.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
                desc.Buffer.NumElements = byteCount / stride;
            }
            hr = device->CreateShaderResourceView(b.buffer, &desc, &b.srv);
            if (FAILED(hr)) return hr;
        }
        if (bind & D3D11_BIND_UNORDERED_ACCESS)
        {
            D3D11_UNORDERED_ACCESS_VIEW_DESC desc = {};
            desc.Format = raw ? DXGI_FORMAT_R32_TYPELESS : DXGI_FORMAT_UNKNOWN;
            desc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
            desc.Buffer.NumElements = byteCount / (raw ? 4 : stride);
            desc.Buffer.Flags = raw ? D3D11_BUFFER_UAV_FLAG_RAW : 0;
            hr = device->CreateUnorderedAccessView(b.buffer, &desc, &b.uav);
        }
        return hr;
    }
    HRESULT Initialize(ID3D11Device* newDevice) noexcept
    {
        device = newDevice;
        const char* entries[] = {"project", "scan", "compact", "skin", "scan_render", "skin_batch", "relocate"};
        for (unsigned i = 0; i < shaders.size(); ++i)
        {
            ID3DBlob *bytecode = nullptr, *errors = nullptr;
            HRESULT hr = D3DCompile(ShaderSource, sizeof(ShaderSource) - 1, "blood_gpu_projection", nullptr, nullptr,
                entries[i], "cs_5_0", D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_IEEE_STRICTNESS |
                D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &bytecode, &errors);
            if (FAILED(hr) && errors) Msg("! Blood projection %s shader: %.240s", entries[i], static_cast<const char*>(errors->GetBufferPointer()));
            Release(errors);
            if (SUCCEEDED(hr)) hr = device->CreateComputeShader(bytecode->GetBufferPointer(), bytecode->GetBufferSize(), nullptr, &shaders[i]);
            Release(bytecode);
            if (FAILED(hr)) return hr;
        }
        constexpr UINT sr = D3D11_BIND_SHADER_RESOURCE, ua = D3D11_BIND_UNORDERED_ACCESS;
        constexpr UINT structured = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED, raw = D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
        HRESULT hr = CreateBuffer(source, MaxSourceBytes, sr, raw);
        if (FAILED(hr)) return hr;
        hr = CreateBuffer(bones, MaxBatchBones * sizeof(Rows), sr, structured, sizeof(Rows));
        if (FAILED(hr)) return hr;
        hr = CreateBuffer(groups, MaxGroups * sizeof(Group), sr, structured, sizeof(Group));
        if (FAILED(hr)) return hr;
        hr = CreateBuffer(scratch, (MaxCandidateVertices / 3) * sizeof(Scratch), ua, structured, sizeof(Scratch));
        if (FAILED(hr)) return hr;
        hr = CreateBuffer(groupCounts, MaxGroups * sizeof(std::uint32_t), ua, structured, sizeof(std::uint32_t));
        if (FAILED(hr)) return hr;
        hr = CreateBuffer(generated, MaxGeneratedVertices * sizeof(ProjectedVertex), sr | ua, structured, sizeof(ProjectedVertex));
        if (FAILED(hr)) return hr;
        hr = CreateBuffer(arguments, MaxMarks * 32, sr | ua, raw | D3D11_RESOURCE_MISC_DRAWINDIRECT_ARGS);
        if (FAILED(hr)) return hr;
        hr = CreateBuffer(vertices, MaxGeneratedVertices * Output::vertex_stride, D3D11_BIND_VERTEX_BUFFER | ua, raw);
        if (FAILED(hr)) return hr;
        hr = CreateBuffer(constants, sizeof(Constants), D3D11_BIND_CONSTANT_BUFFER, 0);
        if (FAILED(hr)) return hr;
        hr = CreateBuffer(renderJobs, MaxBatchJobs * sizeof(RenderJob), sr, structured, sizeof(RenderJob));
        if (FAILED(hr)) return hr;
        hr = CreateBuffer(dents, MaxDentTexels * sizeof(DentTexel), sr, structured, sizeof(DentTexel)); // melee
        if (FAILED(hr)) return hr;
        hr = CreateBuffer(renderGroupEnds, MaxBatchJobs * sizeof(std::uint32_t), ua, structured, sizeof(std::uint32_t));
        if (FAILED(hr)) return hr;
        hr = CreateBuffer(renderArguments, 16, ua, raw | D3D11_RESOURCE_MISC_DRAWINDIRECT_ARGS);
        if (FAILED(hr)) return hr;
        D3D11_BUFFER_DESC staging = {}; staging.ByteWidth = MaxMarks * 32;
        staging.Usage = D3D11_USAGE_STAGING; staging.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
        hr = device->CreateBuffer(&staging, nullptr, &allocationCopy.buffer);
        if (SUCCEEDED(hr)) bytes += staging.ByteWidth;
        return hr;
    }
    void CompactAllocations(Counters& counters) noexcept
    {
        if (renderJobCount) return; // queued jobs already captured firstVertex
        struct Move { std::uint32_t oldFirst, newFirst, count, argsOffset; bool rendered; };
        std::array<Move, MaxMovesPerPoll> moves = {};
        std::uint32_t moveCount = 0, payloadBytes = 0, probes = 0;
        // One slot scan and at most 8192 interval probes, never quadratic work.
        for (std::uint32_t i = 0; i < MaxMarks && moveCount < MaxMovesPerPoll && probes < MaxMarks * 2; ++i)
        {
            Slot& slot = markAtlas.slots[i];
            if (!slot.active || !slot.trimmed || !slot.count) continue;
            const std::uint32_t stride = slot.rendered ? 56 : 12;
            if (slot.count > (MaxMoveBytesPerPoll - payloadBytes) / stride) continue;
            for (std::uint32_t span = 0; span < markAtlas.freeCount && probes < MaxMarks * 2; ++span)
            {
                ++probes;
                const Span& hole = markAtlas.free[span];
                if (hole.first >= slot.first) break;
                if (hole.count < slot.count || hole.first + slot.count > slot.first) continue;
                moves[moveCount++] = {slot.first, hole.first, slot.count, i * 32, slot.rendered};
                payloadBytes += slot.count * stride;
                markAtlas.MoveEarlier(i, span);
                break;
            }
        }
        if (!moveCount) return;
        SavedComputeState saved(HW.pContext);
        ID3D11UnorderedAccessView* outputs[7] = {nullptr, nullptr, generated.uav, arguments.uav, vertices.uav, nullptr, nullptr};
        HW.pContext->CSSetUnorderedAccessViews(0, 7, outputs, nullptr);
        HW.pContext->CSSetConstantBuffers(0, 1, &constants.buffer);
        HW.pContext->CSSetShader(shaders[6], nullptr, 0);
        for (std::uint32_t i = 0; i < moveCount; ++i)
        {
            const Move& move = moves[i];
            Constants c = {}; c.groupCount = move.count; c.firstVertex = move.oldFirst;
            c.color = move.newFirst; c.argsByteOffset = move.argsOffset; c.projection.size = move.rendered ? 1.0f : 0.0f;
            HW.pContext->UpdateSubresource(constants.buffer, 0, nullptr, &c, 0, 0);
            HW.pContext->Dispatch((move.count + 255) / 256, 1, 1);
            counters.moved_vertices += move.count;
        }
        // Moves may depend on earlier moves freeing their old ranges. Separate
        // ordered dispatches preserve that dependency; no parallel copy races.
        counters.allocation_moves += moveCount; counters.moved_payload_bytes += payloadBytes;
    }
    bool Healthy() const noexcept
    {
        return device == HW.pDevice && HW.pDevice && HW.pContext && SUCCEEDED(HW.pDevice->GetDeviceRemovedReason());
    }
};

Backend::Backend() noexcept : m_State(nullptr), m_Epoch(1), m_Counters{}, m_InitializationFailed(false) {}
Backend::~Backend() { Reset(); }
void Backend::Reset() noexcept
{
    if (m_State) { m_State->~State(); std::free(m_State); m_State = nullptr; }
    ++m_Epoch; if (!m_Epoch) ++m_Epoch;
    m_InitializationFailed = false;
    ++m_Counters.resets;
    m_Counters.live_sources = m_Counters.live_source_vertices = 0;
    m_Counters.live_source_indices = m_Counters.live_source_bytes = 0;
    m_Counters.live_marks = m_Counters.reserved_mark_vertices = 0;
    m_Counters.gpu_allocated_bytes = 0;
}
bool Backend::EnsureReady() noexcept
{
    if (m_State && m_State->device != HW.pDevice) Reset();
    if (m_State) return m_State->Healthy();
    if (m_InitializationFailed) return false;
    HRESULT hr = E_FAIL;
    if (HW.pDevice && HW.pContext && HW.FeatureLevel >= D3D_FEATURE_LEVEL_11_0)
    {
        void* memory = std::malloc(sizeof(State));
        if (memory)
        {
            m_State = new (memory) State;
            hr = m_State->Initialize(HW.pDevice);
            if (SUCCEEDED(hr)) { m_Counters.gpu_allocated_bytes = m_State->bytes; return true; }
            m_State->~State(); std::free(m_State); m_State = nullptr;
        }
        else hr = E_OUTOFMEMORY;
    }
    m_InitializationFailed = true;
    ++m_Counters.resource_failures;
    Msg("! Blood GPU projection unavailable (0x%08x); immutable hit snapshots use CPU", static_cast<unsigned>(hr));
    return false;
}
bool Backend::IsValid(const SourceHandle& h) const noexcept
{
    if (!m_State || m_State->device != HW.pDevice || h.epoch != m_Epoch || h.slot >= MaxSources) return false;
    const Slot& s = m_State->sourceAtlas.slots[h.slot];
    return s.active && s.sourceOwned && s.generation == h.generation &&
        m_State->sourceInfo[h.slot].LogicalCount() == h.vertex_count;
}
bool Backend::IsValid(const MarkHandle& h) const noexcept
{
    if (!m_State || m_State->device != HW.pDevice || h.epoch != m_Epoch || h.slot >= MaxMarks) return false;
    const Slot& s = m_State->markAtlas.slots[h.slot];
    return s.active && s.generation == h.generation && s.capacity == h.capacity_vertices;
}
bool Backend::CanRegisterSource(std::uint32_t count) const noexcept
{
    std::uint32_t bytes, slot, span;
    return m_State && m_State->device == HW.pDevice && SourceBytes(count, 0, false, bytes) &&
        m_State->sourceAtlas.Find(bytes, slot, span);
}
bool Backend::RegisterSource(const InputVertex* vertices, std::uint32_t count, SourceHandle& handle) noexcept
{
    return RegisterSourceInternal(vertices, count, nullptr, 0, false, handle);
}
bool Backend::CanRegisterSource(std::uint32_t vertexCount, std::uint32_t indexCount) const noexcept
{
    std::uint32_t bytes, slot, span;
    return m_State && m_State->device == HW.pDevice && SourceBytes(vertexCount, indexCount, true, bytes) &&
        m_State->sourceAtlas.Find(bytes, slot, span);
}
bool Backend::RegisterSource(const InputVertex* vertices, std::uint32_t vertexCount,
    const std::uint32_t* indices, std::uint32_t indexCount, SourceHandle& handle) noexcept
{
    return RegisterSourceInternal(vertices, vertexCount, indices, indexCount, true, handle);
}
bool Backend::RegisterSourceInternal(const InputVertex* vertices, std::uint32_t count,
    const std::uint32_t* indices, std::uint32_t indexCount, bool indexed, SourceHandle& handle) noexcept
{
    std::uint32_t bytes;
    if (!vertices || (indexed && !indices) || !SourceBytes(count, indexCount, indexed, bytes) ||
        !EnsureReady() || IsValid(handle))
    { ++m_Counters.fallbacks; return false; }
    State& state = *m_State;
    std::uint32_t slot, span;
    if (!state.sourceAtlas.Find(bytes, slot, span)) { ++m_Counters.fallbacks; return false; }
    std::uint32_t paletteNeeded = 0;
    for (std::uint32_t i = 0; i < count; ++i)
    {
        const InputVertex& v = vertices[i];
        if (v.job < 1 || v.job > 4) { ++m_Counters.fallbacks; return false; }
        for (unsigned axis = 0; axis < 3; ++axis)
            if (!std::isfinite(v.p[axis])) { ++m_Counters.fallbacks; return false; }
        for (std::uint32_t link = 0; link < v.job; ++link)
        {
            if (v.ids[link] >= MaxPaletteBones || (link + 1 < v.job && !std::isfinite(v.weights[link])))
            { ++m_Counters.fallbacks; return false; }
            paletteNeeded = (std::max)(paletteNeeded, v.ids[link] + 1);
        }
        // Canonical repeated-ID tail is required by the legacy rendering rule.
        for (std::uint32_t link = v.job; link < 4; ++link)
            if (v.ids[link] != v.ids[v.job - 1]) { ++m_Counters.fallbacks; return false; }
    }
    for (std::uint32_t i = 0; i < indexCount; ++i)
        if (indices[i] >= count) { ++m_Counters.fallbacks; return false; }
    const std::uint32_t first = state.sourceAtlas.free[span].first;
    // A single combined span, committed only after validation. There is no
    // index expansion, second allocation or per-hit full-mesh upload. D3D11
    // UpdateSubresource records these immutable bytes without an explicit wait.
    Upload(HW.pContext, state.source.buffer, first, count * sizeof(InputVertex), vertices);
    if (indexed) Upload(HW.pContext, state.source.buffer, first + count * sizeof(InputVertex),
        indexCount * sizeof(std::uint32_t), indices);
    state.sourceAtlas.Allocate(slot, span, bytes, paletteNeeded);
    state.sourceInfo[slot] = {count, indexCount};
    state.sourceAtlas.slots[slot].references = 1;
    state.sourceAtlas.slots[slot].sourceOwned = true;
    handle = {m_Epoch, slot, state.sourceAtlas.slots[slot].generation, indexed ? indexCount : count};
    ++m_Counters.source_uploads; ++m_Counters.live_sources;
    m_Counters.source_vertices_uploaded += count; m_Counters.live_source_vertices += count;
    m_Counters.source_indices_uploaded += indexCount; m_Counters.live_source_indices += indexCount;
    m_Counters.source_bytes_uploaded += bytes; m_Counters.live_source_bytes += bytes;
    return true;
}
void Backend::ReleaseSource(SourceHandle& h) noexcept
{
    if (IsValid(h))
    {
        m_State->sourceAtlas.slots[h.slot].sourceOwned = false;
        m_State->DropSource(h.slot, m_Counters);
    }
    h = {};
}
bool Backend::CanCreateMark(const SourceRange* ranges, std::uint32_t rangeCount, std::uint32_t paletteCount) const noexcept
{
    if (!m_State || !ranges || !rangeCount || rangeCount > MaxGroups || !paletteCount || paletteCount > MaxPaletteBones) return false;
    std::uint32_t vertices = 0, groups = 0;
    for (std::uint32_t i = 0; i < rangeCount; ++i)
    {
        const SourceRange& r = ranges[i];
        if (!IsValid(r.source) || !r.vertex_count || r.vertex_count % 3 || r.first_vertex % 3 ||
            r.first_vertex > r.source.vertex_count || r.vertex_count > r.source.vertex_count - r.first_vertex ||
            r.vertex_count > MaxCandidateVertices - vertices || paletteCount < m_State->sourceAtlas.slots[r.source.slot].paletteNeeded) return false;
        vertices += r.vertex_count;
        groups += (r.vertex_count / 3 + TrianglesPerGroup - 1) / TrianglesPerGroup;
        if (groups > MaxGroups) return false;
    }
    std::uint32_t slot, span;
    return m_State->markAtlas.Find(vertices, slot, span);
}
bool Backend::CreateMark(const SourceRange* ranges, std::uint32_t rangeCount,
    const Rows* hitPalette, std::uint32_t paletteCount, const Projection& projection, MarkHandle& handle) noexcept
{
    if (!EnsureReady() || IsValid(handle) || !CanCreateMark(ranges, rangeCount, paletteCount) ||
        !FinitePalette(hitPalette, paletteCount) || !Finite(projection.view) || !std::isfinite(projection.size) || projection.size <= 0)
    { ++m_Counters.fallbacks; return false; }
    for (unsigned axis = 0; axis < 3; ++axis)
        if (!std::isfinite(projection.contact[axis]) || !std::isfinite(projection.normal[axis]))
        { ++m_Counters.fallbacks; return false; }
    State& state = *m_State;
    std::uint32_t vertices = 0, groupCount = 0, paletteNeeded = 0;
    for (std::uint32_t i = 0; i < rangeCount; ++i)
    {
        const SourceRange& r = ranges[i];
        const Slot& source = state.sourceAtlas.slots[r.source.slot];
        const auto& info = state.sourceInfo[r.source.slot];
        paletteNeeded = (std::max)(paletteNeeded, source.paletteNeeded);
        for (std::uint32_t first = 0; first < r.vertex_count; first += TrianglesPerGroup * 3)
        {
            const std::uint32_t take = (std::min)(TrianglesPerGroup * 3, r.vertex_count - first);
            const std::uint32_t logicalFirst = r.first_vertex + first;
            const std::uint32_t firstByte = info.indexCount ?
                source.first + info.vertexCount * sizeof(InputVertex) + logicalFirst * sizeof(std::uint32_t) :
                source.first + logicalFirst * sizeof(InputVertex);
            state.cpuGroups[groupCount++] = {firstByte, vertices / 3, take / 3,
                info.indexCount ? source.first | IndexedSourceBit : 0};
            vertices += take;
        }
    }
    std::uint32_t slot, span;
    if (!state.markAtlas.Find(vertices, slot, span)) { ++m_Counters.fallbacks; return false; }
    // Retain source indices independently of caller cache lifetime. Allocate
    // before recording work so failure still leaves the CPU snapshot untouched.
    auto* sourceOwners = static_cast<std::uint32_t*>(std::malloc(rangeCount * sizeof(std::uint32_t)));
    if (!sourceOwners) { ++m_Counters.fallbacks; ++m_Counters.resource_failures; return false; }
    for (std::uint32_t i = 0; i < rangeCount; ++i) sourceOwners[i] = ranges[i].source.slot;
    const std::uint32_t first = state.markAtlas.free[span].first;
    Constants c = {}; c.projection = projection; c.groupCount = groupCount; c.firstVertex = first; c.argsByteOffset = slot * 32;
    SavedComputeState saved(HW.pContext);
    Upload(HW.pContext, state.bones.buffer, 0, paletteCount * sizeof(Rows), hitPalette);
    Upload(HW.pContext, state.groups.buffer, 0, groupCount * sizeof(Group), state.cpuGroups.data());
    HW.pContext->UpdateSubresource(state.constants.buffer, 0, nullptr, &c, 0, 0);
    ID3D11ShaderResourceView* views[5] = {state.source.srv, state.bones.srv, state.groups.srv, nullptr, nullptr};
    ID3D11UnorderedAccessView* outputs[5] = {state.scratch.uav, state.groupCounts.uav, state.generated.uav, state.arguments.uav, nullptr};
    HW.pContext->CSSetShaderResources(0, 5, views);
    HW.pContext->CSSetUnorderedAccessViews(0, 5, outputs, nullptr);
    HW.pContext->CSSetConstantBuffers(0, 1, &state.constants.buffer);
    HW.pContext->CSSetShader(state.shaders[0], nullptr, 0);
    HW.pContext->Dispatch(groupCount, 1, 1);
    HW.pContext->CSSetShader(state.shaders[1], nullptr, 0);
    HW.pContext->Dispatch(1, 1, 1);
    HW.pContext->CSSetShader(state.shaders[2], nullptr, 0);
    HW.pContext->Dispatch(groupCount, 1, 1);
    // All fallible validation/allocation precedes recording these commands.
    // Source slot references keep compacted indices valid until ReleaseMark.
    state.markAtlas.Allocate(slot, span, vertices, paletteNeeded);
    state.markSources[slot] = sourceOwners;
    state.markSourceCounts[slot] = rangeCount;
    for (std::uint32_t i = 0; i < rangeCount; ++i) ++state.sourceAtlas.slots[sourceOwners[i]].references;
    handle = {m_Epoch, slot, state.markAtlas.slots[slot].generation, vertices};
    ++m_Counters.marks_created; ++m_Counters.live_marks; ++state.untrimmedMarks;
    m_Counters.candidate_vertices += vertices; m_Counters.reserved_mark_vertices += vertices;
    m_Counters.projection_dispatches += 3;
    return true;
}
void Backend::ReleaseMark(MarkHandle& h) noexcept
{
    if (IsValid(h))
    {
        ++m_Counters.marks_released; --m_Counters.live_marks;
        m_Counters.reserved_mark_vertices -= m_State->markAtlas.slots[h.slot].count;
        if (!m_State->markAtlas.slots[h.slot].trimmed) --m_State->untrimmedMarks;
        for (std::uint32_t i = 0; i < m_State->markSourceCounts[h.slot]; ++i)
            m_State->DropSource(m_State->markSources[h.slot][i], m_Counters);
        std::free(m_State->markSources[h.slot]);
        m_State->markSources[h.slot] = nullptr;
        m_State->markSourceCounts[h.slot] = 0;
        m_State->markAtlas.Free(h.slot);
    }
    h = {};
}
bool Backend::GetOutput(const MarkHandle& h, Output& output) const noexcept
{
    output = {};
    if (!IsValid(h) || !m_State->markAtlas.slots[h.slot].rendered) return false;
    output = {m_State->vertices.buffer, m_State->arguments.buffer, h.slot * 32, h.capacity_vertices};
    return true;
}
bool Backend::RenderMark(const MarkHandle& h, const Rows* palette, std::uint32_t paletteCount,
    const Rows& world, std::uint32_t color, Output& output) noexcept
{
    output = {};
    if (!IsValid(h) || !m_State->Healthy() || !FinitePalette(palette, paletteCount) || !Finite(world) ||
        paletteCount < m_State->markAtlas.slots[h.slot].paletteNeeded)
    { ++m_Counters.fallbacks; return false; }
    State& state = *m_State;
    Slot& slot = state.markAtlas.slots[h.slot];
    Constants c = {}; c.world = world; c.firstVertex = slot.first; c.argsByteOffset = h.slot * 32; c.color = color;
    SavedComputeState saved(HW.pContext);
    Upload(HW.pContext, state.bones.buffer, 0, paletteCount * sizeof(Rows), palette);
    HW.pContext->UpdateSubresource(state.constants.buffer, 0, nullptr, &c, 0, 0);
    ID3D11ShaderResourceView* views[5] = {state.source.srv, state.bones.srv, nullptr, state.generated.srv, state.arguments.srv};
    ID3D11UnorderedAccessView* outputs[5] = {nullptr, nullptr, nullptr, nullptr, state.vertices.uav};
    HW.pContext->CSSetShaderResources(0, 5, views);
    HW.pContext->CSSetUnorderedAccessViews(0, 5, outputs, nullptr);
    HW.pContext->CSSetConstantBuffers(0, 1, &state.constants.buffer);
    HW.pContext->CSSetShader(state.shaders[3], nullptr, 0);
    HW.pContext->DispatchIndirect(state.arguments.buffer, c.argsByteOffset + 16);
    slot.rendered = true;
    ++m_Counters.render_dispatches;
    return GetOutput(h, output);
}
bool Backend::BeginRenderBatch() noexcept
{
    if (!EnsureReady()) { ++m_Counters.fallbacks; return false; }
    m_State->renderJobCount = m_State->renderBoneCount = 0; m_State->renderDents.Clear();
    m_State->renderQueued.fill(false);
    return true;
}
bool Backend::CanAppendRender(std::uint32_t paletteCount) const noexcept
{
    return m_State && paletteCount && paletteCount <= MaxPaletteBones &&
        m_State->renderJobCount < MaxBatchJobs && paletteCount <= MaxBatchBones - m_State->renderBoneCount;
}
bool Backend::AppendRender(const MarkHandle& h, const Rows* palette, std::uint32_t paletteCount,
    const Rows& world, std::uint32_t color,
    const float* dents, std::uint32_t dentCount) noexcept
{
    if (!IsValid(h) || !CanAppendRender(paletteCount) || m_State->renderQueued[h.slot] ||
        !FinitePalette(palette, paletteCount) || !Finite(world) || paletteCount < m_State->markAtlas.slots[h.slot].paletteNeeded)
    { ++m_Counters.fallbacks; return false; }
    State& s = *m_State;
    std::uint32_t dentCode = 0; // melee (value 3): the body's dents for this mark (no room: refused, the caller falls back to the CPU)
    if (dentCount && !(dentCode = s.renderDents.Append(dents, dentCount))) { ++m_Counters.fallbacks; return false; }
    std::memcpy(s.renderBones.data() + s.renderBoneCount, palette, paletteCount * sizeof(Rows));
    s.cpuRenderJobs[s.renderJobCount] = {world, s.renderBoneCount, s.markAtlas.slots[h.slot].first, h.slot * 32, color, dentCode, {0, 0, 0}};
    s.renderMarks[s.renderJobCount] = h;
    s.renderQueued[h.slot] = true;
    s.renderBoneCount += paletteCount; ++s.renderJobCount;
    return true;
}
bool Backend::DispatchRenderBatch() noexcept
{
    if (!m_State || !m_State->Healthy()) { ++m_Counters.fallbacks; return false; }
    State& s = *m_State;
    if (!s.renderJobCount) return true;
    for (std::uint32_t i = 0; i < s.renderJobCount; ++i)
        if (!IsValid(s.renderMarks[i])) { ++m_Counters.fallbacks; return false; }
    Constants c = {}; c.groupCount = s.renderJobCount;
    SavedComputeState saved(HW.pContext);
    Upload(HW.pContext, s.bones.buffer, 0, s.renderBoneCount * sizeof(Rows), s.renderBones.data());
    Upload(HW.pContext, s.renderJobs.buffer, 0, s.renderJobCount * sizeof(RenderJob), s.cpuRenderJobs.data());
    if (s.renderDents.count) Upload(HW.pContext, s.dents.buffer, 0, s.renderDents.count * sizeof(DentTexel), s.renderDents.texels.data()); // melee
    HW.pContext->UpdateSubresource(s.constants.buffer, 0, nullptr, &c, 0, 0);
    ID3D11ShaderResourceView* views[7] = {s.source.srv, s.bones.srv, nullptr, s.generated.srv, s.arguments.srv, s.renderJobs.srv, s.dents.srv}; // melee: t6
    ID3D11UnorderedAccessView* outputs[7] = {nullptr, nullptr, nullptr, nullptr, s.vertices.uav, s.renderGroupEnds.uav, s.renderArguments.uav};
    HW.pContext->CSSetShaderResources(0, 7, views);
    HW.pContext->CSSetUnorderedAccessViews(0, 7, outputs, nullptr);
    HW.pContext->CSSetConstantBuffers(0, 1, &s.constants.buffer);
    HW.pContext->CSSetShader(s.shaders[4], nullptr, 0);
    HW.pContext->Dispatch(1, 1, 1);
    ID3D11UnorderedAccessView* noOutput = nullptr;
    HW.pContext->CSSetUnorderedAccessViews(6, 1, &noOutput, nullptr);
    HW.pContext->CSSetShader(s.shaders[5], nullptr, 0);
    HW.pContext->DispatchIndirect(s.renderArguments.buffer, 0);
    for (std::uint32_t i = 0; i < s.renderJobCount; ++i) s.markAtlas.slots[s.renderMarks[i].slot].rendered = true;
    m_Counters.render_dispatches += 2; ++m_Counters.render_batches;
    s.renderJobCount = s.renderBoneCount = 0; s.renderDents.Clear();
    s.renderQueued.fill(false);
    return true;
}
void Backend::PollCompletedAllocations() noexcept
{
    if (!m_State || !m_State->Healthy()) return;
    State& s = *m_State;
    ++m_Counters.allocation_polls;
    if (s.allocationPending)
    {
        D3D11_MAPPED_SUBRESOURCE mapped = {};
        const HRESULT hr = HW.pContext->Map(s.allocationCopy.buffer, 0, D3D11_MAP_READ, D3D11_MAP_FLAG_DO_NOT_WAIT, &mapped);
        if (hr == DXGI_ERROR_WAS_STILL_DRAWING) { ++m_Counters.allocation_busy; return; }
        if (FAILED(hr)) { ++m_Counters.resource_failures; return; }
        const auto* arguments = static_cast<const std::uint32_t*>(mapped.pData);
        for (std::uint32_t i = 0; i < MaxMarks; ++i)
        {
            Slot& slot = s.markAtlas.slots[i];
            if (!s.allocationGenerations[i] || !slot.active || slot.trimmed || slot.generation != s.allocationGenerations[i]) continue;
            const std::uint32_t count = arguments[i * 8];
            if (count > slot.count || count % 3 || arguments[i * 8 + 1] != 1 || arguments[i * 8 + 2] != slot.first) continue;
            const std::uint32_t reclaimed = slot.count - count;
            s.markAtlas.Shrink(i, count);
            --s.untrimmedMarks;
            m_Counters.reserved_mark_vertices -= reclaimed;
            m_Counters.reclaimed_vertices += reclaimed;
        }
        HW.pContext->Unmap(s.allocationCopy.buffer, 0);
        s.allocationPending = false;
        ++m_Counters.allocation_reads;
    }
    s.CompactAllocations(m_Counters);
    if (!s.untrimmedMarks) return;
    for (std::uint32_t i = 0; i < MaxMarks; ++i)
    {
        const Slot& slot = s.markAtlas.slots[i];
        s.allocationGenerations[i] = slot.active && !slot.trimmed ? slot.generation : 0;
    }
    // The ordered copy follows all queued creation writes. Its CPU metadata is
    // bound to this State/epoch and captured slot generations. Newly reused slots
    // cannot consume an old count, even if release/reallocation precedes Map.
    HW.pContext->CopyResource(s.allocationCopy.buffer, s.arguments.buffer);
    s.allocationPending = true;
    ++m_Counters.allocation_copies;
}

}

#else
namespace BloodGpuProjection
{
struct Backend::State {};
Backend::Backend() noexcept : m_State(nullptr), m_Epoch(1), m_Counters{}, m_InitializationFailed(false) {}
Backend::~Backend() {}
bool Backend::EnsureReady() noexcept { return false; }
bool Backend::CanRegisterSource(std::uint32_t) const noexcept { return false; }
bool Backend::CanRegisterSource(std::uint32_t, std::uint32_t) const noexcept { return false; }
bool Backend::RegisterSource(const InputVertex*, std::uint32_t, SourceHandle&) noexcept { ++m_Counters.fallbacks; return false; }
bool Backend::RegisterSource(const InputVertex*, std::uint32_t, const std::uint32_t*, std::uint32_t, SourceHandle&) noexcept { ++m_Counters.fallbacks; return false; }
bool Backend::IsValid(const SourceHandle&) const noexcept { return false; }
void Backend::ReleaseSource(SourceHandle& h) noexcept { h = {}; }
bool Backend::CanCreateMark(const SourceRange*, std::uint32_t, std::uint32_t) const noexcept { return false; }
bool Backend::CreateMark(const SourceRange*, std::uint32_t, const Rows*, std::uint32_t, const Projection&, MarkHandle&) noexcept { ++m_Counters.fallbacks; return false; }
bool Backend::IsValid(const MarkHandle&) const noexcept { return false; }
void Backend::ReleaseMark(MarkHandle& h) noexcept { h = {}; }
bool Backend::RenderMark(const MarkHandle&, const Rows*, std::uint32_t, const Rows&, std::uint32_t, Output& out) noexcept { out = {}; ++m_Counters.fallbacks; return false; }
bool Backend::BeginRenderBatch() noexcept { return false; }
bool Backend::CanAppendRender(std::uint32_t) const noexcept { return false; }
bool Backend::AppendRender(const MarkHandle&, const Rows*, std::uint32_t, const Rows&, std::uint32_t, const float*, std::uint32_t) noexcept { ++m_Counters.fallbacks; return false; }
bool Backend::DispatchRenderBatch() noexcept { ++m_Counters.fallbacks; return false; }
void Backend::PollCompletedAllocations() noexcept {}
bool Backend::GetOutput(const MarkHandle&, Output& out) const noexcept { out = {}; return false; }
void Backend::Reset() noexcept { ++m_Counters.resets; }
}
#endif
