#ifndef BUFFER_H_
#define BUFFER_H_

// cpp
#include <sys/eventfd.h>
#include <unistd.h>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

#include "sensorType.h"
#include "spinLock.h"
#include "statistical_analyzer.h"
#include "storagePolicy.h"

namespace ts {

template <typename Message>
concept ValidMessage = requires(Message msg, double ts) {
    { msg.get_timestamp() } -> std::convertible_to<double>;
    { msg.set_timestamp(ts) } -> std::same_as<void>;
#ifdef ENABLE_ANALYZER
    { msg.get_host_timestamp() } -> std::convertible_to<double>;
#endif
};

template <
    ValidMessage Message, SensorType Type, typename StoragePolicy,
    BufferStrategy Strategy = BufferStrategy::FixedSize>

class Buffer {
public:
    using Handle = typename StoragePolicy::template Handle<Message>;

    using Interpolator = std::function<Handle(const Handle&, const Handle&, const double)>;

    using SharedPtr = std::shared_ptr<Buffer<Message, Type, StoragePolicy, Strategy>>;

    using UniquePtr = std::unique_ptr<Buffer<Message, Type, StoragePolicy, Strategy>>;

    static UniquePtr createUnique(size_t capacity) {
        return std::unique_ptr<Buffer>(new Buffer(capacity));
    }

    static SharedPtr createShared(size_t capacity) {
        return std::shared_ptr<Buffer>(new Buffer(capacity));
    }

    void set_interpolator(Interpolator interpolator) { interpolator_ = interpolator; }

    size_t size() {
        SharedSpinLock lock(flag_);
        return buffer_.size();
    }

    bool empty() {
        SharedSpinLock lock(flag_);
        return buffer_.empty();
    }

    void clear() {
        SharedSpinLock lock(flag_);
        buffer_.clear();
    }

    void pop_front() {
        SharedSpinLock lock(flag_);
        if (!buffer_.empty()) {
            buffer_.erase(buffer_.begin());
        }
    }

    void pop_back() {
        SharedSpinLock lock(flag_);
        if (!buffer_.empty()) {
            buffer_.pop_back();
        }
    }

    bool isPresent(const double ts) {
        SharedSpinLock lock(flag_);
        if (buffer_.empty()) {
            return false;
        }

        auto it_begin = std::lower_bound(
            buffer_.begin(), buffer_.end(), ts - dt_tolerance_,
            [](const Handle& h, double ts) { return get_timestamp(h) < ts; });
        auto it_end = std::upper_bound(
            buffer_.begin(), buffer_.end(), ts + dt_tolerance_,
            [](double ts, const Handle& h) { return ts < get_timestamp(h); });

        return it_begin != it_end;
    }

    bool isSliceAvailable(const double ts_begin, const double ts_end) {
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

    double get_oldest_timestamp() {
        SharedSpinLock lock(flag_);
        if (!buffer_.empty()) {
            return get_timestamp(buffer_.front());
        }
        return -1.0;
    }

    double get_newest_timestamp() {
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

    std::optional<Handle> fetchNearest(const double ts) {
        SharedSpinLock lock(flag_);
        if (buffer_.empty()) {
            return std::nullopt;
        }

        auto it_begin = std::lower_bound(
            buffer_.begin(), buffer_.end(), ts - dt_tolerance_,
            [](const Handle& h, double ts) { return get_timestamp(h) < ts; });
        auto it_end = std::upper_bound(
            buffer_.begin(), buffer_.end(), ts + dt_tolerance_,
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

    void push_back(Handle value) {
        SharedSpinLock lock(flag_);

        if constexpr (is_unique_ptr<Handle>::value || is_shared_ptr<Handle>::value) {
            if (!value) {
                return;
            }
        }

        if (!buffer_.empty() && get_timestamp(value) <= get_timestamp(buffer_.back())) {
            return;
        }
#ifdef ENABLE_ANALYZER

        if (analyzer_) {
            analyzer_->add_timestamp(id, get_host_timestamp(value));
        }
#endif

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

    void set_tolerance(const double tolerance) {
        SharedSpinLock lock(flag_);
        dt_tolerance_ = tolerance;
    }

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
        std::cout << "Successfully to register as trigger" << std::endl;
    }

    int get_eventfd() const { return efd_; }

    void set_sensor_name(const std::string& name) { sensor_name_ = name; }

    std::string get_sensor_name() const { return sensor_name_; }

    void set_sensor_id(size_t id) { this->id = id; }

    size_t get_sensor_id() const { return id; }

private:
    Buffer() = delete;

    Buffer(const size_t capacity)
        : capacity_(capacity), dt_tolerance_(0.005), trigger_flag_(false) {
        flag_ = AtomicFlagFactory::create();
        buffer_.reserve(capacity_);
    }

    Buffer(Buffer&) = delete;

    Buffer(const Buffer&) = delete;

    inline static double get_timestamp(const Handle& handle) {
        if constexpr (is_unique_ptr<Handle>::value || is_shared_ptr<Handle>::value) {
            return handle->get_timestamp();
        } else {
            return handle.get_timestamp();
        }
    }

#ifdef ENABLE_ANALYZER
    inline static double get_host_timestamp(const Handle& handle) {
        if constexpr (is_unique_ptr<Handle>::value || is_shared_ptr<Handle>::value) {
            return handle->get_host_timestamp();
        } else {
            return handle.get_host_timestamp();
        }
    }
#endif
    size_t capacity_;

    std::vector<Handle> buffer_;

    Interpolator interpolator_;

    std::shared_ptr<std::atomic<bool>> flag_;

    double dt_tolerance_;

    bool trigger_flag_;

    int efd_ = -1;

    std::string sensor_name_;

    StatisticalAnalyzer* analyzer_;

    size_t id;
};
}  // namespace ts

#endif
