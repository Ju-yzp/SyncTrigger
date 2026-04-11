#ifndef DATA_BUFFER_H_
#define DATA_BUFFER_H_

#include <dataBuffer_base.h>
#include <storagePolicy.h>
#include <algorithm>
#include <concepts>
#include <memory>
#include <mutex>
#include <optional>
#include <type_traits>
#include <vector>

namespace data {

// message constains a timestamp field (either a member or a function)
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

template <
    ValidMessage Message, typename StoragePolicy, BufferStrategy Strategy = BufferStrategy::Dynamic,
    SensorType type = SensorType::Camera>
class DataBuffer : public DataBufferBase {
public:
    using Handle = typename StoragePolicy::template Handle<Message>;

    DataBuffer(size_t capacity) : capacity_(capacity) {}

    DataBuffer(DataBuffer& other) {
        std::lock_guard<std::mutex> lock(other.mtx_);
        if constexpr (is_unique_ptr<Handle>::value) {
            buffer_ = std::move(other.buffer_);
        } else {
            buffer_ = other.buffer_;
        }
        capacity_ = other.capacity_;
    }

    bool empty() override {
        std::lock_guard<std::mutex> lock(mtx_);
        return buffer_.empty();
    }

    void clear() override {
        std::lock_guard<std::mutex> lock(mtx_);
        buffer_.clear();
    }

    double get_lastest_timestamp() override {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!buffer_.empty()) {
            return get_timestamp(*buffer_.begin());
        }

        return -1.0;
    }

    void push_back(Handle value) {
        std::lock_guard<std::mutex> lock(mtx_);
        if(!buffer_.empty()&&get_timestamp(value) <= get_timestamp(*buffer_.end())){
            return;
        }
        if (Strategy != BufferStrategy::Dynamic) {
            if (buffer_.size() > capacity_) {
                buffer_.erase(buffer_.begin());
            }
        }

        if constexpr (is_unique_ptr<Handle>::value) {
            buffer_.emplace_back(std::move(value));
        } else {
            buffer_.emplace_back(value);
        }
        if (notify_cv_) {
            notify_cv_->notify_all();
        }
    }

    std::size_t size() override {
        std::lock_guard<std::mutex> lock(mtx_);
        return buffer_.size();
    }

    void get_nearest_data(ItemCallback cb, double ts, double tolerance) override {
        auto result = get_nearest(ts, tolerance);
        if (result.has_value()) {
            cb(static_cast<void*>(&result.value()));
        }
    }

    void get_data_by_range(ItemCallback cb, double ts_begin, double ts_end) override {
        std::vector<Handle> result = selectByRange(ts_begin, ts_end);
        for (auto& handle : result) {
            cb(static_cast<void*>(&handle));
        }
    }

private:
    inline static double get_timestamp(const Handle& handle) {
        if constexpr (is_unique_ptr<Handle>::value || is_shared_ptr<Handle>::value) {
            return handle->get_timestamp();
        } else {
            return handle.get_timestamp();
        }
    }
    // select messages with timestamp range [begin_ts, end_ts),messages that earlier than begin_ts
    // will be dropped
    std::vector<Handle> selectByRange(const double begin_ts, const double end_ts) {
        if (begin_ts >= end_ts) {
            std::throw_with_nested(std::runtime_error("begin_ts must be less than end_ts"));
        }

        std::lock_guard<std::mutex> lock_(mtx_);
        if (buffer_.empty()) return {};

        auto it_begin = std::lower_bound(
            buffer_.begin(), buffer_.end(), begin_ts,
            [this](const Handle& h, double ts) { return get_timestamp(h) < ts; });
        auto it_end = std::upper_bound(
            buffer_.begin(), buffer_.end(), end_ts,
            [this](double ts, const Handle& h) { return ts < get_timestamp(h); });
        std::vector<Handle> result;
        if constexpr (is_unique_ptr<Handle>::value) {
            result.assign(std::make_move_iterator(it_begin), std::make_move_iterator(it_end));
        } else {
            result.assign(it_begin, it_end);
        }
        buffer_.erase(buffer_.begin(), it_end);
        return result;
    }

    std::optional<Handle> get_nearest(const double ts, const double tolerance) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (buffer_.empty()) {
            return std::nullopt;
        }

        auto it_begin = std::lower_bound(
            buffer_.begin(), buffer_.end(), ts - tolerance,
            [](const Handle& h, double ts) { return get_timestamp(h) < ts; });
        auto it_end = std::upper_bound(
            buffer_.begin(), buffer_.end(), ts + tolerance,
            [](double ts, const Handle& h) { return ts < get_timestamp(h); });
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

    mutable std::mutex mtx_;
    size_t capacity_;
    std::vector<Handle> buffer_;
};
}  // namespace data

#endif  // DATA_BUFFER_H_
