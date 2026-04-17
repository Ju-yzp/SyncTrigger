#ifndef BUFFER_H_
#define BUFFER_H_

// cpp
#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "buffer_base.h"
#include "spinLock.h"
#include "storagePolicy.h"

namespace ts {

template <typename Message>
concept ValidMessage = requires(Message msg, double ts) {
    { msg.get_timestamp() } -> std::convertible_to<double>;
    { msg.set_timestamp(ts) } -> std::same_as<void>;
};

template <
    ValidMessage Message, typename StoragePolicy,
    BufferStrategy Strategy = BufferStrategy::FixedSize>

class Buffer : public BufferBase {
public:
    using Handle = typename StoragePolicy::template Handle<Message>;

    using Interpolator = std::function<Handle(const Handle&, const Handle&, const double)>;

    explicit Buffer(const size_t capacity) : capacity_(capacity) {
        flag_ = AtomicFlagFactory::create();
        buffer_.reserve(capacity_);
    }

    Buffer(Buffer& other) {
        SharedSpinLock lock(other.flag_);
        this->flag_ = AtomicFlagFactory::create();
        if constexpr (is_unique_ptr<Handle>::value) {
            buffer_ = std::move(other.buffer_);
        } else {
            buffer_ = other.buffer_;
        }
        capacity_ = other.capacity_;
    }

    void set_interpolator(Interpolator interpolator) { interpolator_ = interpolator; }

    size_t size() override {
        SharedSpinLock lock(flag_);
        return buffer_.size();
    }

    bool empty() override {
        SharedSpinLock lock(flag_);
        return buffer_.empty();
    }

    void clear() override {
        SharedSpinLock lock(flag_);
        buffer_.clear();
    }

    void pop_front() override {
        SharedSpinLock lock(flag_);
        if (!buffer_.empty()) {
            buffer_.erase(buffer_.begin());
        }
    }

    void pop_back() override {
        SharedSpinLock lock(flag_);
        if (!buffer_.empty()) {
            buffer_.pop_back();
        }
    }

    bool isPresent(const double ts, const double tolerance) override {
        SharedSpinLock lock(flag_);
        if (buffer_.empty()) {
            return false;
        }

        return (get_timestamp(buffer_.back()) <= (ts + tolerance) &&
                get_timestamp(buffer_.back()) >= (ts - tolerance)) ||
               (get_timestamp(buffer_.front()) <= (ts + tolerance) &&
                get_timestamp(buffer_.front()) >= (ts - tolerance));
    }

    bool isSliceAvailable(const double ts_begin, const double ts_end) override {
        SharedSpinLock lock(flag_);
        if (buffer_.empty()) {
            return false;
        }

        bool flag = false;
        if (ts_begin < 0) {
            flag = get_timestamp(buffer_.back()) >= ts_end;
        } else {
            flag =
                (get_timestamp(buffer_.front()) >= ts_begin &&
                 get_timestamp(buffer_.back()) >= ts_end);
        }

        return flag;
    }

    double get_oldest_timestamp() override {
        SharedSpinLock lock(flag_);
        if (!buffer_.empty()) {
            return get_timestamp(buffer_.front());
        }
        return -1.0;
    }

    double get_newest_timestamp() override {
        SharedSpinLock lock(flag_);
        if (!buffer_.empty()) {
            return get_timestamp(buffer_.back());
        }
        return -1.0;
    }

    std::vector<Handle> fetchSlice(const double ts_begin, const double ts_end) {
        if (ts_begin >= ts_end) {
            throw std::runtime_error("[ts::Buffer] selectByRange: begin_ts >= end_ts");
        }

        if (isSliceAvailable(ts_begin, ts_end)) {
            auto it_begin = std::lower_bound(
                buffer_.begin(), buffer_.end(), ts_begin,
                [this](const Handle& h, double ts) { return get_timestamp(h) < ts; });
            auto it_end = std::upper_bound(
                buffer_.begin(), buffer_.end(), ts_end,
                [this](double ts, const Handle& h) { return ts < get_timestamp(h); });

            std::vector<Handle> result;
            result.reserve(std::distance(it_begin, it_end) + 1);

            if (interpolator_ && it_end != buffer_.begin() && it_end != buffer_.end()) {
                if (it_begin < it_end) {
                    if constexpr (is_unique_ptr<Handle>::value) {
                        result.insert(
                            result.end(), std::make_move_iterator(it_begin),
                            std::make_move_iterator(it_end));
                    } else {
                        result.insert(result.end(), it_begin, it_end);
                    }
                }

                Handle interpolated_node = interpolator_(*(it_end - 1), *it_end, ts_end);

                if constexpr (is_unique_ptr<Handle>::value) {
                    interpolated_node->set_timestamp(ts_end);
                } else {
                    interpolated_node.set_timestamp(ts_end);
                }
                result.push_back(interpolated_node);
                buffer_.erase(buffer_.begin(), it_end);
                buffer_.insert(buffer_.begin(), std::move(interpolated_node));

            } else {
                if (it_begin < it_end) {
                    if constexpr (is_unique_ptr<Handle>::value) {
                        result.insert(
                            result.end(), std::make_move_iterator(it_begin),
                            std::make_move_iterator(it_end));
                    } else {
                        result.insert(result.end(), it_begin, it_end);
                    }
                }
                buffer_.erase(buffer_.begin(), it_end);
            }

            return result;
        }
        return std::vector<Handle>();
    }

    std::optional<Handle> fetchNearest(const double ts, const double tolerance) {
        SharedSpinLock lock(flag_);
        if (buffer_.empty()) {
            return std::nullopt;
        }

        auto it_begin = std::lower_bound(
            buffer_.begin(), buffer_.end(), ts - tolerance,
            [](const Handle& h, double ts) { return get_timestamp(h) <= ts; });
        auto it_end = std::upper_bound(
            buffer_.begin(), buffer_.end(), ts + tolerance,
            [](double ts, const Handle& h) { return ts <= get_timestamp(h); });
        if (it_begin == it_end) {
            return std::nullopt;
        }

        auto best_it = it_begin;
        double min_diff = std::abs(get_timestamp(*it_begin) - ts);

        auto last_it = std::prev(it_end);
        if (last_it != it_begin) {
            double last_diff = std::abs(get_timestamp(*last_it) - ts);
            if (last_diff < min_diff) {
                best_it = last_it;
            }
        }

        Handle result;
        if constexpr (is_unique_ptr<Handle>::value) {
            result = std::move(*best_it);
        } else {
            result = *best_it;
        }

        buffer_.erase(buffer_.begin(), std::next(best_it));
        return result;
    }

    void push_back(Handle value) {
        SharedSpinLock lock(flag_);

        if (!buffer_.empty() && get_timestamp(value) <= get_timestamp(buffer_.back())) {
            return;
        }
        if (Strategy != BufferStrategy::Dynamic) {
            if (buffer_.size() >= capacity_) {
                buffer_.erase(buffer_.begin());
            }
        }

        if constexpr (is_unique_ptr<Handle>::value) {
            buffer_.emplace_back(std::move(value));
        } else {
            buffer_.emplace_back(value);
        }

        if (trigger_flag_) {
            uint64_t signal = 1;
            write(get_eventfd(), &signal, sizeof(signal));
        }
    }

private:
    Buffer() = delete;

    inline static double get_timestamp(const Handle& handle) {
        if constexpr (is_unique_ptr<Handle>::value || is_shared_ptr<Handle>::value) {
            return handle->get_timestamp();
        } else {
            return handle.get_timestamp();
        }
    }

    size_t capacity_;

    std::vector<Handle> buffer_;

    Interpolator interpolator_;

    std::shared_ptr<std::atomic<bool>> flag_;
};
}  // namespace ts

#endif
