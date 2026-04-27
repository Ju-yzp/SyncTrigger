#ifndef STORAGE_POLICY_H_
#define STORAGE_POLICY_H_

#include <memory>

namespace {
struct ValueStorage {
    template <typename T>
    using Handle = T;
};

struct SharedPtrStorage {
    template <typename T>
    using Handle = std::shared_ptr<T>;
};

struct UniquePtrStorage {
    template <typename T>
    using Handle = std::unique_ptr<T>;
};

template <typename T>
struct is_shared_ptr : std::false_type {};

template <typename T>
struct is_shared_ptr<std::shared_ptr<T>> : std::true_type {};

template <typename T>
struct is_unique_ptr : std::false_type {};

template <typename T, typename D>
struct is_unique_ptr<std::unique_ptr<T, D>> : std::true_type {};

enum class BufferStrategy : uint8_t { Dynamic = 0, FixedSize = 1, Discard = 2 };
}  // namespace

#endif
