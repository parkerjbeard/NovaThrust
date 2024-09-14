#ifndef LINEAR_ALGEBRA_INL
#define LINEAR_ALGEBRA_INL

namespace LinearAlgebra {

template<typename T>
Matrix<T>::Matrix(size_t rows, size_t cols) : m_rows(rows), m_cols(cols), m_data(rows * cols) {}

template<typename T>
Matrix<T>::Matrix(size_t rows, size_t cols, const std::vector<T>& data)
    : m_rows(rows), m_cols(cols), m_data(data) {
    if (data.size() != rows * cols) {
        throw std::invalid_argument("Data size does not match matrix dimensions");
    }
}

template<typename T>
T& Matrix<T>::operator()(size_t i, size_t j) {
    return m_data[i * m_cols + j];
}

template<typename T>
const T& Matrix<T>::operator()(size_t i, size_t j) const {
    return m_data[i * m_cols + j];
}

// Implement other template functions here...

} // namespace LinearAlgebra

#endif // LINEAR_ALGEBRA_INL