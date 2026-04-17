#ifndef SPIN_LOCK_H_
#define SPIN_LOCK_H_

#include <atomic>
#include <memory>
#include <stdexcept>

namespace ts {

/**
 * @brief Factory for hardware-optimized synchronization primitives.
 * * This factory ensures that each atomic flag is physically isolated in memory
 * to prevent performance degradation caused by "False Sharing."
 */
class AtomicFlagFactory {
public:
    struct alignas(64) AlignedFlag {
        std::atomic<bool> value{false};
    };

    static std::shared_ptr<std::atomic<bool>> create() {
        auto pool = std::make_shared<AlignedFlag>();
        return std::shared_ptr<std::atomic<bool>>(pool, &pool->value);
    }
};

class SharedSpinLock {
public:
    explicit SharedSpinLock(std::shared_ptr<std::atomic<bool>> flag) : flag_(flag) {
        if (!flag_) {
            throw std::runtime_error(
                "[ts::SharedSpinLock] Initialization failed: flag pointer is null. "
                "Ensure the AtomicFlagFactory was called correctly before creating the lock.");
        }

        while (flag_->exchange(true, std::memory_order_acquire)) {
            __builtin_ia32_pause();
        }
    }

    SharedSpinLock(const SharedSpinLock&) = delete;

    SharedSpinLock& operator=(const SharedSpinLock&) = delete;

    ~SharedSpinLock() { flag_->store(false, std::memory_order_release); }

private:
    std::shared_ptr<std::atomic<bool>> flag_;
};
}  // namespace ts

#endif
