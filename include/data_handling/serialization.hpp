#ifndef SERIALIZATION_HPP
#define SERIALIZATION_HPP

#include <cstring>
#include <vector>
#include <string>
#include <type_traits>
#include <stdexcept>
#include <memory>
#include <cstdint>

namespace Serialization {

class Buffer {
public:
    Buffer() : data_(nullptr), size_(0), capacity_(0) {}
    explicit Buffer(size_t capacity) : data_(new char[capacity]), size_(0), capacity_(capacity) {}
    ~Buffer() { delete[] data_; }

    Buffer(const Buffer&) = delete;
    Buffer& operator=(const Buffer&) = delete;

    Buffer(Buffer&& other) noexcept
        : data_(other.data_), size_(other.size_), capacity_(other.capacity_) {
        other.data_ = nullptr;
        other.size_ = other.capacity_ = 0;
    }

    Buffer& operator=(Buffer&& other) noexcept {
        if (this != &other) {
            delete[] data_;
            data_ = other.data_;
            size_ = other.size_;
            capacity_ = other.capacity_;
            other.data_ = nullptr;
            other.size_ = other.capacity_ = 0;
        }
        return *this;
    }

    void resize(size_t new_size) {
        if (new_size > capacity_) {
            size_t new_capacity = std::max(new_size, capacity_ * 2);
            char* new_data = new char[new_capacity];
            std::memcpy(new_data, data_, size_);
            delete[] data_;
            data_ = new_data;
            capacity_ = new_capacity;
        }
        size_ = new_size;
    }

    char* data() { return data_; }
    const char* data() const { return data_; }
    size_t size() const { return size_; }
    size_t capacity() const { return capacity_; }

private:
    char* data_;
    size_t size_;
    size_t capacity_;
};

class Serializer {
public:
    explicit Serializer(Buffer& buffer) : buffer_(buffer), position_(0) {}

    template<typename T>
    void write(const T& value) {
        static_assert(std::is_trivially_copyable_v<T>, "Type must be trivially copyable");
        ensureCapacity(sizeof(T));
        std::memcpy(buffer_.data() + position_, &value, sizeof(T));
        position_ += sizeof(T);
    }

    void write(const std::string& str) {
        write(static_cast<uint32_t>(str.size()));
        ensureCapacity(str.size());
        std::memcpy(buffer_.data() + position_, str.data(), str.size());
        position_ += str.size();
    }

    template<typename T>
    void write(const std::vector<T>& vec) {
        write(static_cast<uint32_t>(vec.size()));
        for (const auto& item : vec) {
            write(item);
        }
    }

    // Zero-copy write for POD types
    template<typename T>
    void write_zero_copy(const T& value) {
        static_assert(std::is_trivially_copyable_v<T>, "Type must be trivially copyable");
        ensureCapacity(sizeof(T));
        *reinterpret_cast<T*>(buffer_.data() + position_) = value;
        position_ += sizeof(T);
    }

    size_t getPosition() const { return position_; }

private:
    Buffer& buffer_;
    size_t position_;

    void ensureCapacity(size_t additional_size) {
        if (position_ + additional_size > buffer_.capacity()) {
            buffer_.resize(position_ + additional_size);
        }
    }
};

class Deserializer {
public:
    Deserializer(const char* data, size_t size) : data_(data), size_(size), position_(0) {}

    template<typename T>
    T read() {
        static_assert(std::is_trivially_copyable_v<T>, "Type must be trivially copyable");
        if (position_ + sizeof(T) > size_) {
            throw std::out_of_range("Buffer overflow");
        }
        T value;
        std::memcpy(&value, data_ + position_, sizeof(T));
        position_ += sizeof(T);
        return value;
    }

    std::string readString() {
        uint32_t length = read<uint32_t>();
        if (position_ + length > size_) {
            throw std::out_of_range("Buffer overflow");
        }
        std::string str(data_ + position_, length);
        position_ += length;
        return str;
    }

    template<typename T>
    std::vector<T> readVector() {
        uint32_t size = read<uint32_t>();
        std::vector<T> vec;
        vec.reserve(size);
        for (uint32_t i = 0; i < size; ++i) {
            vec.push_back(read<T>());
        }
        return vec;
    }

    // Zero-copy read for POD types
    template<typename T>
    const T& read_zero_copy() {
        static_assert(std::is_trivially_copyable_v<T>, "Type must be trivially copyable");
        if (position_ + sizeof(T) > size_) {
            throw std::out_of_range("Buffer overflow");
        }
        const T& value = *reinterpret_cast<const T*>(data_ + position_);
        position_ += sizeof(T);
        return value;
    }

    size_t getPosition() const { return position_; }

private:
    const char* data_;
    size_t size_;
    size_t position_;
};

// Helper function for serialization
template<typename T>
Buffer serialize(const T& value) {
    Buffer buffer;
    Serializer serializer(buffer);
    serializer.write(value);
    return buffer;
}

// Helper function for deserialization
template<typename T>
T deserialize(const Buffer& buffer) {
    Deserializer deserializer(buffer.data(), buffer.size());
    return deserializer.read<T>();
}

} // namespace Serialization

#endif // SERIALIZATION_HPP