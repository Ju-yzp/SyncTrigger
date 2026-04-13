#ifndef STORAGE_POLICY_H_
#define STORAGE_POLICY_H_

#include <memory>

namespace data {

// custom storage policies
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

enum class BufferStrategy : uint8_t {
    Dynamic = 0,
    FixedSize = 1,  // default policy
    Discard = 2
};
}  // namespace data

#endif
