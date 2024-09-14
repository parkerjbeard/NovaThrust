#ifndef PARALLELIZATION_HPP
#define PARALLELIZATION_HPP

#include <thread>
#include <future>
#include <vector>
#include <functional>
#include <algorithm>
#include <numeric>
#include <coroutine>
#include <optional>

#ifdef USE_MPI
#include <mpi.h>
#endif

namespace Parallelization {

// Multi-threading

template<typename Iterator, typename T, typename BinaryOp>
T parallel_reduce(Iterator first, Iterator last, T init, BinaryOp op, size_t num_threads = std::thread::hardware_concurrency()) {
    const auto length = std::distance(first, last);
    if (length < 1000) {
        return std::accumulate(first, last, init, op);
    }

    const auto chunk_size = length / num_threads;
    std::vector<std::future<T>> futures(num_threads - 1);

    for (size_t i = 0; i < num_threads - 1; ++i) {
        futures[i] = std::async(std::launch::async, [=, &op] {
            auto chunk_start = std::next(first, i * chunk_size);
            auto chunk_end = std::next(chunk_start, chunk_size);
            return std::accumulate(chunk_start, chunk_end, T{}, op);
        });
    }

    T result = std::accumulate(std::next(first, (num_threads - 1) * chunk_size), last, init, op);

    for (auto& f : futures) {
        result = op(result, f.get());
    }

    return result;
}

template<typename Iterator, typename UnaryOp>
void parallel_for_each(Iterator first, Iterator last, UnaryOp op, size_t num_threads = std::thread::hardware_concurrency()) {
    const auto length = std::distance(first, last);
    if (length < 1000) {
        std::for_each(first, last, op);
        return;
    }

    const auto chunk_size = length / num_threads;
    std::vector<std::future<void>> futures(num_threads - 1);

    for (size_t i = 0; i < num_threads - 1; ++i) {
        futures[i] = std::async(std::launch::async, [=, &op] {
            auto chunk_start = std::next(first, i * chunk_size);
            auto chunk_end = std::next(chunk_start, chunk_size);
            std::for_each(chunk_start, chunk_end, op);
        });
    }

    std::for_each(std::next(first, (num_threads - 1) * chunk_size), last, op);

    for (auto& f : futures) {
        f.wait();
    }
}

// C++20 Coroutines

template<typename T>
struct Generator {
    struct promise_type {
        T current_value;
        std::suspend_always yield_value(T value) {
            current_value = value;
            return {};
        }
        std::suspend_always initial_suspend() { return {}; }
        std::suspend_always final_suspend() noexcept { return {}; }
        Generator get_return_object() { return Generator{this}; }
        void unhandled_exception() { std::terminate(); }
        void return_void() {}
    };

    struct iterator {
        std::coroutine_handle<promise_type> h;
        iterator(std::coroutine_handle<promise_type> handle) : h(handle) {}
        iterator(const iterator&) = delete;
        iterator& operator=(const iterator&) = delete;
        iterator& operator++() {
            h.resume();
            return *this;
        }
        bool operator!=(const iterator& other) const { return h != other.h; }
        const T& operator*() const { return h.promise().current_value; }
    };

    iterator begin() {
        if (h) {
            h.resume();
        }
        return {h};
    }
    iterator end() { return {nullptr}; }

    Generator(const Generator&) = delete;
    Generator& operator=(const Generator&) = delete;
    Generator(Generator&& other) : h(other.h) { other.h = nullptr; }
    Generator& operator=(Generator&& other) {
        if (this != &other) {
            if (h) {
                h.destroy();
            }
            h = other.h;
            other.h = nullptr;
        }
        return *this;
    }
    ~Generator() {
        if (h) {
            h.destroy();
        }
    }

private:
    explicit Generator(promise_type* p)
        : h(std::coroutine_handle<promise_type>::from_promise(*p)) {}
    std::coroutine_handle<promise_type> h;
};

// MPI Wrapper

#ifdef USE_MPI
class MPIWrapper {
public:
    MPIWrapper(int* argc, char*** argv) {
        MPI_Init(argc, argv);
        MPI_Comm_rank(MPI_COMM_WORLD, &rank);
        MPI_Comm_size(MPI_COMM_WORLD, &size);
    }

    ~MPIWrapper() {
        MPI_Finalize();
    }

    int getRank() const { return rank; }
    int getSize() const { return size; }

    template<typename T>
    void broadcast(T& data, int root = 0) {
        MPI_Bcast(&data, sizeof(T), MPI_BYTE, root, MPI_COMM_WORLD);
    }

    template<typename T>
    void send(const T& data, int dest, int tag = 0) {
        MPI_Send(&data, sizeof(T), MPI_BYTE, dest, tag, MPI_COMM_WORLD);
    }

    template<typename T>
    T receive(int source, int tag = 0) {
        T data;
        MPI_Recv(&data, sizeof(T), MPI_BYTE, source, tag, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
        return data;
    }

    template<typename T>
    T reduce(const T& local_data, MPI_Op op, int root = 0) {
        T result;
        MPI_Reduce(&local_data, &result, 1, mpi_type<T>(), op, root, MPI_COMM_WORLD);
        return result;
    }

private:
    int rank, size;

    template<typename T>
    MPI_Datatype mpi_type() {
        if constexpr (std::is_same_v<T, int>) return MPI_INT;
        else if constexpr (std::is_same_v<T, float>) return MPI_FLOAT;
        else if constexpr (std::is_same_v<T, double>) return MPI_DOUBLE;
        else return MPI_BYTE;
    }
};
#endif // USE_MPI

} // namespace Parallelization

#endif // PARALLELIZATION_HPP