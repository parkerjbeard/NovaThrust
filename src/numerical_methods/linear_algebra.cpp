#include "numerical_methods/linear_algebra.hpp"
#include <algorithm>
#include <cmath>

#if defined(HAVE_BLAS) && defined(HAVE_LAPACK)
extern "C" {
    #include <cblas.h>
    #include <lapacke.h>
}
#endif

namespace LinearAlgebra {

// Implement non-template functions here...

#if defined(HAVE_BLAS) && defined(HAVE_LAPACK)
namespace BLAS {

void gemm(const Matrix<double>& A, const Matrix<double>& B, Matrix<double>& C) {
    cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
                A.rows(), B.cols(), A.cols(),
                1.0, A.data().data(), A.cols(),
                B.data().data(), B.cols(),
                0.0, const_cast<double*>(C.data().data()), C.cols());
}

void gemv(const Matrix<double>& A, const std::vector<double>& x, std::vector<double>& y) {
    cblas_dgemv(CblasRowMajor, CblasNoTrans,
                A.rows(), A.cols(),
                1.0, A.data().data(), A.cols(),
                x.data(), 1,
                0.0, y.data(), 1);
}

} // namespace BLAS

namespace LAPACK {

std::vector<double> gesv(const Matrix<double>& A, const std::vector<double>& b) {
    Matrix<double> A_copy = A;
    std::vector<double> x = b;
    std::vector<lapack_int> ipiv(A.rows());

    lapack_int n = A.rows();
    lapack_int nrhs = 1;
    lapack_int lda = A.cols();
    lapack_int ldb = 1;

    lapack_int info = LAPACKE_dgesv(LAPACK_ROW_MAJOR, n, nrhs,
                                    A_copy.data().data(), lda, ipiv.data(),
                                    x.data(), ldb);

    if (info != 0) {
        throw std::runtime_error("LAPACKE_dgesv failed");
    }

    return x;
}

} // namespace LAPACK
#endif

} // namespace LinearAlgebra