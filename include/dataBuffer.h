#ifndef DATA_BUFFER_H_
#define DATA_BUFFER_H_

// cpp
#include <algorithm>
#include <atomic>
#include <concepts>
#include <cstddef>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include "dataBuffer_base.h"
#include "storagePolicy.h"

namespace data {

template <typename Message>
concept ValidMessage = requires(Message msg) {
    { msg.get_timestamp() } -> std::convertible_to<double>;
};

template <typename T>
struct is_shared_ptr : std::false_type {};

template <typename T>
struct is_shared_ptr<std::shared_ptr<T>> : std::true_type {};

template <typename T>
struct is_unique_ptr : std::false_type {};

template <typename T, typename D>
struct is_unique_ptr<std::unique_ptr<T, D>> : std::true_type {};

// TODO: Implement automatic resource management to replace manual release.
class SpinLock {
public:
    SpinLock() : flag_(false) {}
    void wait() {
        while (flag_.exchange(true, std::memory_order_acquire)) {
            __builtin_ia32_pause();
        }
    }
    void release() { flag_.store(false, std::memory_order_release); }

private:
    alignas(64) std::atomic<bool> flag_;
};

template <
    ValidMessage Message, SensorType type, typename StoragePolicy,
    BufferStrategy Strategy = BufferStrategy::FixedSize>
class DataBuffer : public DataBufferBase {
public:
    using Handle = typename StoragePolicy::template Handle<Message>;

    DataBuffer(size_t capacity) : capacity_(capacity) {}

    DataBuffer(DataBuffer& other) {
        other.spin_lock_.wait();
        if constexpr (is_unique_ptr<Handle>::value) {
            buffer_ = std::move(other.buffer_);
        } else {
            buffer_ = other.buffer_;
        }
        capacity_ = other.capacity_;
        other.spin_lock_.release();
    }

    void push_back(Handle value) {
        spin_lock_.wait();

        if (!buffer_.empty() && get_timestamp(value) <= get_timestamp(buffer_.back())) {
            spin_lock_.release();
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

        spin_lock_.release();
        if (trigger_flag_) {
            uint64_t signal = 1;
            write(get_eventfd(), &signal, sizeof(signal));
        }
    }

    void clear() override {
        spin_lock_.wait();
        buffer_.clear();
        spin_lock_.release();
    }

    std::size_t size() override {
        spin_lock_.wait();
        std::size_t size = buffer_.size();
        spin_lock_.release();
        return size;
    }

    bool empty() override {
        spin_lock_.wait();
        bool is_empty = buffer_.empty();
        spin_lock_.release();
        return is_empty;
    }

    bool query_by_time_range(double ts_begin, double ts_end) override {
        spin_lock_.wait();
        if (buffer_.empty()) {
            spin_lock_.release();
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
        spin_lock_.release();
        return flag;
    }

    bool query_by_time_window(double ts, double tolerance) override {
        spin_lock_.wait();
        if (buffer_.empty()) {
            spin_lock_.release();
            return false;
        }

        bool flag = (get_timestamp(buffer_.back()) <= (ts + tolerance) &&
                     get_timestamp(buffer_.back()) >= (ts - tolerance)) ||
                    (get_timestamp(buffer_.front()) <= (ts + tolerance) &&
                     get_timestamp(buffer_.front()) >= (ts - tolerance));
        spin_lock_.release();
        return flag;
    }

    void get_nearest_data(Callback cb, double ts, double tolerance) override {
        auto result = get_nearest(ts, tolerance);
        if (result.has_value()) {
            cb(static_cast<void*>(&result.value()));
        }
    }

    void get_data_set(Callback cb, double ts_begin, double ts_end) override {
        std::vector<Handle> result = selectByRange(ts_begin, ts_end);
        for (auto& handle : result) {
            cb(static_cast<void*>(&handle));
        }
    }

    double get_oldest_timestamp() override {
        spin_lock_.wait();
        if (!buffer_.empty()) {
            spin_lock_.release();
            return get_timestamp(buffer_.front());
        }
        spin_lock_.release();
        return -1.0;
    }

    void pop_front() override {
        spin_lock_.wait();
        if (!buffer_.empty()) {
            buffer_.erase(buffer_.begin());
        }
        spin_lock_.release();
    }

private:
    inline static double get_timestamp(const Handle& handle) {
        if constexpr (is_unique_ptr<Handle>::value || is_shared_ptr<Handle>::value) {
            return handle->get_timestamp();
        } else {
            return handle.get_timestamp();
        }
    }

    std::optional<Handle> get_nearest(const double ts, const double tolerance) {
        spin_lock_.wait();
        if (buffer_.empty()) {
            spin_lock_.release();
            return std::nullopt;
        }

        auto it_begin = std::lower_bound(
            buffer_.begin(), buffer_.end(), ts - tolerance,
            [](const Handle& h, double ts) { return get_timestamp(h) <= ts; });
        auto it_end = std::upper_bound(
            buffer_.begin(), buffer_.end(), ts + tolerance,
            [](double ts, const Handle& h) { return ts <= get_timestamp(h); });
        if (it_begin == it_end) {
            spin_lock_.release();
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
        spin_lock_.release();
        return result;
    }

    std::vector<Handle> selectByRange(const double begin_ts, const double end_ts) {
        if (begin_ts >= end_ts) {
            throw std::runtime_error("begin_ts must be less than end_ts");
        }

        spin_lock_.wait();
        if (buffer_.empty()) {
            spin_lock_.release();
            return {};
        }

        auto it_begin = std::lower_bound(
            buffer_.begin(), buffer_.end(), begin_ts,
            [this](const Handle& h, double ts) { return this->get_timestamp(h) < ts; });
        auto it_end = std::upper_bound(
            buffer_.begin(), buffer_.end(), end_ts,
            [this](double ts, const Handle& h) { return ts < this->get_timestamp(h); });

        std::vector<Handle> result;
        auto get_ptr = [](auto& handle) -> void* {
            if constexpr (is_unique_ptr<Handle>::value || is_shared_ptr<Handle>::value) {
                return static_cast<void*>(handle.get());
            } else {
                return static_cast<void*>(&handle);
            }
        };

        if (interpolate_f_ && it_end != buffer_.begin() && it_end != buffer_.end()) {
            auto& lhs = *(it_end - 1);
            auto& rhs = *it_end;

            Handle res_node;
            Handle buf_node;

            if constexpr (is_unique_ptr<Handle>::value) {
                using T = typename Handle::element_type;
                res_node = std::make_unique<T>(*lhs);
                buf_node = std::make_unique<T>(*lhs);
            } else if constexpr (is_shared_ptr<Handle>::value) {
                using T = typename Handle::element_type;
                res_node = std::make_shared<T>(*lhs);
                buf_node = std::make_shared<T>(*lhs);
            } else {
                res_node = lhs;
                buf_node = lhs;
            }
            if constexpr (is_unique_ptr<Handle>::value || is_shared_ptr<Handle>::value) {
                res_node->timestamp = end_ts;
            } else {
                res_node.timestamp = end_ts;
            }

            interpolate_f_(get_ptr(lhs), get_ptr(rhs), get_ptr(res_node));

            if constexpr (is_unique_ptr<Handle>::value || is_shared_ptr<Handle>::value) {
                *buf_node = *res_node;
            } else {
                buf_node = res_node;
            }

            if constexpr (is_unique_ptr<Handle>::value) {
                result.assign(std::make_move_iterator(it_begin), std::make_move_iterator(it_end));
            } else {
                result.assign(it_begin, it_end);
            }
            result.push_back(std::move(res_node));
            buffer_.erase(buffer_.begin(), it_end);
            buffer_.insert(buffer_.begin(), std::move(buf_node));
        } else {
            if constexpr (is_unique_ptr<Handle>::value) {
                result.assign(std::make_move_iterator(it_begin), std::make_move_iterator(it_end));
            } else {
                result.assign(it_begin, it_end);
            }
            buffer_.erase(buffer_.begin(), it_end);
        }

        spin_lock_.release();
        return result;
    }

    SpinLock spin_lock_;
    size_t capacity_;
    std::vector<Handle> buffer_;
};
}  // namespace data

#endif  // DATA_BUFFER_H_
