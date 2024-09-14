#ifndef VECTORIZATION_HPP
#define VECTORIZATION_HPP

#include <cstdint>
#include <cstddef>  // Add this line for size_t
#include <type_traits>
#include <array>

// Check for available SIMD instruction sets
#if defined(__AVX512F__)
#include <immintrin.h>
#define SIMD_AVX512 1
#elif defined(__AVX__)
#include <immintrin.h>
#define SIMD_AVX 1
#elif defined(__SSE4_1__)
#include <smmintrin.h>
#define SIMD_SSE 1
#endif

namespace Vectorization {

// SIMD vector type
template<typename T, size_t N>
struct alignas(sizeof(T) * N) SimdVector {
    std::array<T, N> data;

    SimdVector() = default;
    SimdVector(T scalar) { data.fill(scalar); }
    SimdVector(const std::array<T, N>& arr) : data(arr) {}

    T& operator[](size_t i) { return data[i]; }
    const T& operator[](size_t i) const { return data[i]; }
};

// Trait to determine the appropriate SIMD vector type
template<typename T, size_t N>
struct SimdTrait {
    // Default case: no SIMD type available
    using Type = void;
};

// SSE specializations
#if defined(SIMD_SSE)
template<> struct SimdTrait<float, 4> { using Type = __m128; };
template<> struct SimdTrait<double, 2> { using Type = __m128d; };
template<> struct SimdTrait<int32_t, 4> { using Type = __m128i; };
#endif

// AVX specializations
#if defined(SIMD_AVX)
template<> struct SimdTrait<float, 8> { using Type = __m256; };
template<> struct SimdTrait<double, 4> { using Type = __m256d; };
template<> struct SimdTrait<int32_t, 8> { using Type = __m256i; };
#endif

// AVX-512 specializations
#if defined(SIMD_AVX512)
template<> struct SimdTrait<float, 16> { using Type = __m512; };
template<> struct SimdTrait<double, 8> { using Type = __m512d; };
template<> struct SimdTrait<int32_t, 16> { using Type = __m512i; };
#endif

// Load operation
template<typename T, size_t N>
inline auto load(const SimdVector<T, N>& v);

// Store operation
template<typename T, size_t N>
inline void store(SimdVector<T, N>& v, const typename SimdTrait<T, N>::Type& x);

// Add operation
template<typename T, size_t N>
inline SimdVector<T, N> add(const SimdVector<T, N>& a, const SimdVector<T, N>& b);

// Multiply operation
template<typename T, size_t N>
inline SimdVector<T, N> multiply(const SimdVector<T, N>& a, const SimdVector<T, N>& b);

// Add more SIMD operations as needed (subtract, divide, max, min, etc.)

// Helper for static_assert
template<typename T>
inline constexpr bool always_false = false;

} // namespace Vectorization

#include "vectorization.inl"

#endif // VECTORIZATION_HPP