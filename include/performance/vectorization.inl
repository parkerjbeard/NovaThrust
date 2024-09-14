#ifndef VECTORIZATION_INL
#define VECTORIZATION_INL

#include <cstddef>

namespace Vectorization {

// Load operation
template<typename T, size_t N>
inline auto load(const SimdVector<T, N>& v) {
    if constexpr (std::is_same_v<T, float> && N == 4 && defined(SIMD_SSE))
        return _mm_load_ps(v.data.data());
    else if constexpr (std::is_same_v<T, double> && N == 2 && defined(SIMD_SSE))
        return _mm_load_pd(v.data.data());
    else if constexpr (std::is_same_v<T, int32_t> && N == 4 && defined(SIMD_SSE))
        return _mm_load_si128(reinterpret_cast<const __m128i*>(v.data.data()));
    else if constexpr (std::is_same_v<T, float> && N == 8 && defined(SIMD_AVX))
        return _mm256_load_ps(v.data.data());
    else if constexpr (std::is_same_v<T, double> && N == 4 && defined(SIMD_AVX))
        return _mm256_load_pd(v.data.data());
    else if constexpr (std::is_same_v<T, int32_t> && N == 8 && defined(SIMD_AVX))
        return _mm256_load_si256(reinterpret_cast<const __m256i*>(v.data.data()));
    else if constexpr (std::is_same_v<T, float> && N == 16 && defined(SIMD_AVX512))
        return _mm512_load_ps(v.data.data());
    else if constexpr (std::is_same_v<T, double> && N == 8 && defined(SIMD_AVX512))
        return _mm512_load_pd(v.data.data());
    else if constexpr (std::is_same_v<T, int32_t> && N == 16 && defined(SIMD_AVX512))
        return _mm512_load_si512(v.data.data());
    else
        static_assert(always_false<T>, "Unsupported SIMD vector type");
}

// Store operation
template<typename T, size_t N>
inline void store(SimdVector<T, N>& v, const typename SimdTrait<T, N>::Type& x) {
    if constexpr (std::is_same_v<T, float> && N == 4 && defined(SIMD_SSE))
        _mm_store_ps(v.data.data(), x);
    else if constexpr (std::is_same_v<T, double> && N == 2 && defined(SIMD_SSE))
        _mm_store_pd(v.data.data(), x);
    else if constexpr (std::is_same_v<T, int32_t> && N == 4 && defined(SIMD_SSE))
        _mm_store_si128(reinterpret_cast<__m128i*>(v.data.data()), x);
    else if constexpr (std::is_same_v<T, float> && N == 8 && defined(SIMD_AVX))
        _mm256_store_ps(v.data.data(), x);
    else if constexpr (std::is_same_v<T, double> && N == 4 && defined(SIMD_AVX))
        _mm256_store_pd(v.data.data(), x);
    else if constexpr (std::is_same_v<T, int32_t> && N == 8 && defined(SIMD_AVX))
        _mm256_store_si256(reinterpret_cast<__m256i*>(v.data.data()), x);
    else if constexpr (std::is_same_v<T, float> && N == 16 && defined(SIMD_AVX512))
        _mm512_store_ps(v.data.data(), x);
    else if constexpr (std::is_same_v<T, double> && N == 8 && defined(SIMD_AVX512))
        _mm512_store_pd(v.data.data(), x);
    else if constexpr (std::is_same_v<T, int32_t> && N == 16 && defined(SIMD_AVX512))
        _mm512_store_si512(v.data.data(), x);
    else
        static_assert(always_false<T>, "Unsupported SIMD vector type");
}

// Add operation
template<typename T, size_t N>
inline SimdVector<T, N> add(const SimdVector<T, N>& a, const SimdVector<T, N>& b) {
    SimdVector<T, N> result;
    auto va = load(a);
    auto vb = load(b);

    if constexpr (std::is_same_v<T, float> && N == 4 && defined(SIMD_SSE))
        store(result, _mm_add_ps(va, vb));
    else if constexpr (std::is_same_v<T, double> && N == 2 && defined(SIMD_SSE))
        store(result, _mm_add_pd(va, vb));
    else if constexpr (std::is_same_v<T, int32_t> && N == 4 && defined(SIMD_SSE))
        store(result, _mm_add_epi32(va, vb));
    else if constexpr (std::is_same_v<T, float> && N == 8 && defined(SIMD_AVX))
        store(result, _mm256_add_ps(va, vb));
    else if constexpr (std::is_same_v<T, double> && N == 4 && defined(SIMD_AVX))
        store(result, _mm256_add_pd(va, vb));
    else if constexpr (std::is_same_v<T, int32_t> && N == 8 && defined(SIMD_AVX))
        store(result, _mm256_add_epi32(va, vb));
    else if constexpr (std::is_same_v<T, float> && N == 16 && defined(SIMD_AVX512))
        store(result, _mm512_add_ps(va, vb));
    else if constexpr (std::is_same_v<T, double> && N == 8 && defined(SIMD_AVX512))
        store(result, _mm512_add_pd(va, vb));
    else if constexpr (std::is_same_v<T, int32_t> && N == 16 && defined(SIMD_AVX512))
        store(result, _mm512_add_epi32(va, vb));
    else {
        for (size_t i = 0; i < N; ++i)
            result[i] = a[i] + b[i];
    }

    return result;
}

// Multiply operation
template<typename T, size_t N>
inline SimdVector<T, N> multiply(const SimdVector<T, N>& a, const SimdVector<T, N>& b) {
    SimdVector<T, N> result;
    auto va = load(a);
    auto vb = load(b);

    if constexpr (std::is_same_v<T, float> && N == 4 && defined(SIMD_SSE))
        store(result, _mm_mul_ps(va, vb));
    else if constexpr (std::is_same_v<T, double> && N == 2 && defined(SIMD_SSE))
        store(result, _mm_mul_pd(va, vb));
    else if constexpr (std::is_same_v<T, int32_t> && N == 4 && defined(SIMD_SSE))
        store(result, _mm_mullo_epi32(va, vb));
    else if constexpr (std::is_same_v<T, float> && N == 8 && defined(SIMD_AVX))
        store(result, _mm256_mul_ps(va, vb));
    else if constexpr (std::is_same_v<T, double> && N == 4 && defined(SIMD_AVX))
        store(result, _mm256_mul_pd(va, vb));
    else if constexpr (std::is_same_v<T, int32_t> && N == 8 && defined(SIMD_AVX))
        store(result, _mm256_mullo_epi32(va, vb));
    else if constexpr (std::is_same_v<T, float> && N == 16 && defined(SIMD_AVX512))
        store(result, _mm512_mul_ps(va, vb));
    else if constexpr (std::is_same_v<T, double> && N == 8 && defined(SIMD_AVX512))
        store(result, _mm512_mul_pd(va, vb));
    else if constexpr (std::is_same_v<T, int32_t> && N == 16 && defined(SIMD_AVX512))
        store(result, _mm512_mullo_epi32(va, vb));
    else {
        for (size_t i = 0; i < N; ++i)
            result[i] = a[i] * b[i];
    }

    return result;
}

} // namespace Vectorization

#endif // VECTORIZATION_INL