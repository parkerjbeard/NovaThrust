#ifndef LINEAR_ALGEBRA_HPP
#define LINEAR_ALGEBRA_HPP

#include <vector>
#include <complex>
#include <stdexcept>

namespace LinearAlgebra {

template<typename T>
class Matrix {
public:
    Matrix(size_t rows, size_t cols);
    Matrix(size_t rows, size_t cols, const std::vector<T>& data);

    T& operator()(size_t i, size_t j);
    const T& operator()(size_t i, size_t j) const;

    size_t rows() const { return m_rows; }
    size_t cols() const { return m_cols; }
    const std::vector<T>& data() const { return m_data; }
    std::vector<T>& data() { return m_data; }

private:
    size_t m_rows, m_cols;
    std::vector<T> m_data;
};

// Matrix operations
template<typename T>
Matrix<T> operator*(const Matrix<T>& A, const Matrix<T>& B);

template<typename T>
std::vector<T> operator*(const Matrix<T>& A, const std::vector<T>& x);

// Linear system solving
template<typename T>
std::vector<T> solveLinearSystem(const Matrix<T>& A, const std::vector<T>& b);

template<typename T>
void gaussianElimination(Matrix<T>& A, std::vector<T>& b);

// BLAS and LAPACK interfaces
#if defined(HAVE_BLAS) && defined(HAVE_LAPACK)
namespace BLAS {
    void gemm(const Matrix<double>& A, const Matrix<double>& B, Matrix<double>& C);
    void gemv(const Matrix<double>& A, const std::vector<double>& x, std::vector<double>& y);
}

namespace LAPACK {
    std::vector<double> gesv(const Matrix<double>& A, const std::vector<double>& b);
}
#endif

} // namespace LinearAlgebra

#include "linear_algebra.inl"

#endif // LINEAR_ALGEBRA_HPP