#ifndef BUFFER_BASE_H_
#define BUFFER_BASE_H_

// cpp
#include <sys/eventfd.h>
#include <unistd.h>
#include <cstddef>
#include <stdexcept>

namespace ts {
class BufferBase {
public:
    virtual ~BufferBase() = default;

    virtual size_t size() = 0;

    virtual bool empty() = 0;

    virtual void clear() = 0;

    virtual void pop_front() = 0;

    virtual void pop_back() = 0;

    virtual bool isPresent(const double ts, const double tolerance) = 0;

    virtual bool isSliceAvailable(const double ts_begin, const double ts_end) = 0;

    virtual double get_oldest_timestamp() = 0;

    virtual double get_newest_timestamp() = 0;

    void register_as_trigger() {
        if (efd_ >= 0) {
            close(efd_);
            efd_ = -1;
        }

        efd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);

        if (efd_ < 0) {
            throw std::runtime_error("Failed to create eventfd");
            trigger_flag_ = false;
            return;
        }

        trigger_flag_ = true;
    }

    int get_eventfd() const { return efd_; }

protected:
    bool trigger_flag_ = false;
    int efd_ = -1;
};
}  // namespace ts

#endif
