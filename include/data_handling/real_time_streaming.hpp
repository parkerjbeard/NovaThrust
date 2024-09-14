#ifndef REAL_TIME_STREAMING_HPP
#define REAL_TIME_STREAMING_HPP

#include <vector>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <functional>
#include <stdexcept>
#include <algorithm>
#include <type_traits>

namespace RealTimeStreaming {

template<typename T>
class CircularBuffer {
public:
    explicit CircularBuffer(size_t capacity)
        : buffer_(capacity), head_(0), tail_(0), size_(0), capacity_(capacity) {}

    void push(const T& item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (size_ == capacity_) {
            throw std::runtime_error("Buffer is full");
        }
        buffer_[tail_] = item;
        tail_ = (tail_ + 1) % capacity_;
        ++size_;
    }

    T pop() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (size_ == 0) {
            throw std::runtime_error("Buffer is empty");
        }
        T item = buffer_[head_];
        head_ = (head_ + 1) % capacity_;
        --size_;
        return item;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return size_ == 0;
    }

    bool full() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return size_ == capacity_;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return size_;
    }

    size_t capacity() const {
        return capacity_;
    }

private:
    std::vector<T> buffer_;
    size_t head_;
    size_t tail_;
    size_t size_;
    const size_t capacity_;
    mutable std::mutex mutex_;
};

template<typename T>
class LockFreeCircularBuffer {
public:
    explicit LockFreeCircularBuffer(size_t capacity)
        : buffer_(capacity), head_(0), tail_(0), capacity_(capacity) {}

    bool push(const T& item) {
        size_t current_tail = tail_.load(std::memory_order_relaxed);
        size_t next_tail = (current_tail + 1) % capacity_;
        if (next_tail == head_.load(std::memory_order_acquire)) {
            return false; // Buffer is full
        }
        buffer_[current_tail] = item;
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }

    bool pop(T& item) {
        size_t current_head = head_.load(std::memory_order_relaxed);
        if (current_head == tail_.load(std::memory_order_acquire)) {
            return false; // Buffer is empty
        }
        item = buffer_[current_head];
        head_.store((current_head + 1) % capacity_, std::memory_order_release);
        return true;
    }

    bool empty() const {
        return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
    }

    size_t capacity() const {
        return capacity_;
    }

private:
    std::vector<T> buffer_;
    std::atomic<size_t> head_;
    std::atomic<size_t> tail_;
    const size_t capacity_;
};

template<typename T>
class StreamProcessor {
public:
    using ProcessFunction = std::function<void(const T&)>;

    StreamProcessor(size_t buffer_size, ProcessFunction process_func)
        : buffer_(buffer_size), process_func_(std::move(process_func)), running_(false) {}

    ~StreamProcessor() {
        stop();
    }

    void start() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) {
            running_ = true;
            worker_thread_ = std::thread(&StreamProcessor::processLoop, this);
        }
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (running_) {
                running_ = false;
            }
        }
        cv_.notify_one();
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

    void pushData(const T& data) {
        buffer_.push(data);
        cv_.notify_one();
    }

private:
    void processLoop() {
        while (true) {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return !buffer_.empty() || !running_; });

            if (!running_ && buffer_.empty()) {
                break;
            }

            while (!buffer_.empty()) {
                T data = buffer_.pop();
                lock.unlock();
                process_func_(data);
                lock.lock();
            }
        }
    }

    CircularBuffer<T> buffer_;
    ProcessFunction process_func_;
    std::atomic<bool> running_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::thread worker_thread_;
};

template<typename T, typename Aggregator>
class StreamAggregator {
public:
    StreamAggregator(size_t window_size, Aggregator aggregator)
        : window_size_(window_size), aggregator_(std::move(aggregator)) {}

    void addSample(const T& sample) {
        buffer_.push_back(sample);
        if (buffer_.size() > window_size_) {
            buffer_.erase(buffer_.begin());
        }
    }

    T getAggregate() const {
        if (buffer_.empty()) {
            throw std::runtime_error("No samples to aggregate");
        }
        return aggregator_(buffer_.begin(), buffer_.end());
    }

private:
    std::vector<T> buffer_;
    size_t window_size_;
    Aggregator aggregator_;
};

} // namespace RealTimeStreaming

#endif // REAL_TIME_STREAMING_HPP