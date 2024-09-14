#ifndef MEMORY_OPTIMIZATION_HPP
#define MEMORY_OPTIMIZATION_HPP

#include <cstddef>
#include <cstdlib>
#include <new>
#include <limits>
#include <vector>
#include <memory>
#include <stdexcept>

namespace MemoryOptimization {

// Aligned allocation functions
void* aligned_alloc(size_t alignment, size_t size) {
#if defined(_MSC_VER)
    return _aligned_malloc(size, alignment);
#else
    void* ptr = nullptr;
    if (posix_memalign(&ptr, alignment, size) != 0) {
        throw std::bad_alloc();
    }
    return ptr;
#endif
}

void aligned_free(void* ptr) {
#if defined(_MSC_VER)
    _aligned_free(ptr);
#else
    free(ptr);
#endif
}

// Aligned allocator
template <typename T, size_t Alignment>
class AlignedAllocator {
public:
    using value_type = T;
    using pointer = T*;
    using const_pointer = const T*;
    using reference = T&;
    using const_reference = const T&;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;

    template <typename U>
    struct rebind {
        using other = AlignedAllocator<U, Alignment>;
    };

    AlignedAllocator() noexcept = default;
    template <typename U>
    AlignedAllocator(const AlignedAllocator<U, Alignment>&) noexcept {}

    pointer allocate(size_type n) {
        if (n > std::numeric_limits<size_type>::max() / sizeof(T)) {
            throw std::bad_alloc();
        }
        return static_cast<pointer>(aligned_alloc(Alignment, n * sizeof(T)));
    }

    void deallocate(pointer p, size_type) noexcept {
        aligned_free(p);
    }
};

template <typename T1, typename T2, size_t Alignment>
bool operator==(const AlignedAllocator<T1, Alignment>&, const AlignedAllocator<T2, Alignment>&) noexcept {
    return true;
}

template <typename T1, typename T2, size_t Alignment>
bool operator!=(const AlignedAllocator<T1, Alignment>&, const AlignedAllocator<T2, Alignment>&) noexcept {
    return false;
}

// Memory pool
template <typename T, size_t BlockSize = 4096>
class MemoryPool {
public:
    using value_type = T;
    using pointer = T*;
    using const_pointer = const T*;
    using reference = T&;
    using const_reference = const T&;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;

    template <typename U>
    struct rebind {
        using other = MemoryPool<U, BlockSize>;
    };

    MemoryPool() noexcept : current_block_(nullptr), current_slot_(nullptr), last_slot_(nullptr), free_slots_(nullptr) {}
    ~MemoryPool() noexcept {
        slot_pointer_ curr = current_block_;
        while (curr != nullptr) {
            slot_pointer_ prev = curr->next;
            operator delete(reinterpret_cast<void*>(curr));
            curr = prev;
        }
    }

    pointer allocate(size_type n = 1) {
        if (n > 1) {
            return static_cast<pointer>(::operator new(n * sizeof(T)));
        }

        if (free_slots_ != nullptr) {
            pointer result = reinterpret_cast<pointer>(free_slots_);
            free_slots_ = free_slots_->next;
            return result;
        }

        if (current_slot_ >= last_slot_) {
            data_pointer_ new_block = reinterpret_cast<data_pointer_>(operator new(BlockSize));
            reinterpret_cast<slot_pointer_>(new_block)->next = current_block_;
            current_block_ = reinterpret_cast<slot_pointer_>(new_block);
            current_slot_ = new_block;
            last_slot_ = reinterpret_cast<slot_pointer_>(new_block + BlockSize);
        }

        return reinterpret_cast<pointer>(current_slot_++);
    }

    void deallocate(pointer p, size_type n = 1) noexcept {
        if (n > 1) {
            ::operator delete(p);
            return;
        }

        reinterpret_cast<slot_pointer_>(p)->next = free_slots_;
        free_slots_ = reinterpret_cast<slot_pointer_>(p);
    }

    template <typename U, typename... Args>
    void construct(U* p, Args&&... args) {
        new (p) U(std::forward<Args>(args)...);
    }

    template <typename U>
    void destroy(U* p) {
        p->~U();
    }

private:
    union Slot_ {
        value_type element;
        Slot_* next;
    };

    using data_pointer_ = char*;
    using slot_pointer_ = Slot_*;

    slot_pointer_ current_block_;
    slot_pointer_ current_slot_;
    slot_pointer_ last_slot_;
    slot_pointer_ free_slots_;

    static_assert(BlockSize >= 2 * sizeof(slot_pointer_), "BlockSize too small.");
};

// Custom deleter for unique_ptr with aligned allocation
template <typename T>
struct AlignedDeleter {
    void operator()(T* ptr) const {
        if (ptr) {
            ptr->~T();
            aligned_free(ptr);
        }
    }
};

// Helper function to create a unique_ptr with aligned allocation
template <typename T, size_t Alignment, typename... Args>
std::unique_ptr<T, AlignedDeleter<T>> make_aligned_unique(Args&&... args) {
    void* ptr = aligned_alloc(Alignment, sizeof(T));
    if (!ptr) {
        throw std::bad_alloc();
    }
    try {
        T* obj = new (ptr) T(std::forward<Args>(args)...);
        return std::unique_ptr<T, AlignedDeleter<T>>(obj);
    } catch (...) {
        aligned_free(ptr);
        throw;
    }
}

} // namespace MemoryOptimization

#endif // MEMORY_OPTIMIZATION_HPP