#pragma once
// R69: std::max / std::min of a short list of values. Without a NaN the library's initializer_list overloads return the
// leftmost largest (smallest) element: largest=first, then for each next x `if(largest<x)largest=x` (min: `if(x<smallest)
// smallest=x`), which is what these left-to-right pairwise calls compute - the same value, bit for bit (+-0, infinities and
// denormals included); MSVC reaches that result through its vectorized array kernels, whose dispatch dominated short lists in
// the separation's owner searches. A list holding a NaN (unspecified by the standard; MSVC's kernels do not follow the
// sequential definition there) is handed to the library call itself, so every result is the library's.
#include <algorithm>
#include <initializer_list>
#include <type_traits>
namespace painter_short_minmax {
template<class T,class...R>inline T maxOf(T a,R...rest){
 static_assert((std::is_same_v<T,R>&&...),"maxOf: one element type, as in an initializer_list");
 if constexpr(std::is_floating_point_v<T>){if(a!=a||((rest!=rest)||...))return std::max({a,rest...});}
 T m=a;((m=std::max(m,rest)),...);return m;}
template<class T,class...R>inline T minOf(T a,R...rest){
 static_assert((std::is_same_v<T,R>&&...),"minOf: one element type, as in an initializer_list");
 if constexpr(std::is_floating_point_v<T>){if(a!=a||((rest!=rest)||...))return std::min({a,rest...});}
 T m=a;((m=std::min(m,rest)),...);return m;}
}
