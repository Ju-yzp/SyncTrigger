#ifndef SYNCHRONIZER_H_
#define SYNCHRONIZER_H_

// cpp
#include <poll.h>
#include <unistd.h>
#include <deque>
#include <stdexcept>
#include <thread>

#include "dataBuffer.h"
#include "dataBuffer_base.h"
#include "storagePolicy.h"

namespace data {
template <typename T>
struct buffer_traits;

template <typename Msg, data::SensorType ST, typename SP, data::BufferStrategy BS>
struct buffer_traits<data::DataBuffer<Msg, ST, SP, BS>> {
    using HandleType = typename SP::template Handle<Msg>;
    static constexpr data::SensorType sensor_type = ST;

    using StorageType =
        std::conditional_t<ST == data::SensorType::IMU, std::vector<HandleType>, HandleType>;
};

template <typename... BufferTypes>
struct DataPacket {
    using TupleType = std::tuple<typename buffer_traits<BufferTypes>::StorageType...>;
    TupleType data;

    template <std::size_t I>
    auto& get() {
        return std::get<I>(data);
    }
};

template <typename T>
struct is_data_buffer : std::false_type {};

template <
    typename Message, data::SensorType Type, typename StoragePolicy, data::BufferStrategy Strategy>
struct is_data_buffer<data::DataBuffer<Message, Type, StoragePolicy, Strategy>> : std::true_type {};

template <typename Trigger, typename... Buffers>
class Synchronizer {
public:
    static_assert(is_data_buffer<Trigger>::value, "Trigger must be a specialization of DataBuffer");

    static_assert(
        (is_data_buffer<Buffers>::value && ...),
        "All Buffers must be specializations of DataBuffer");

    using DataPackage = DataPacket<Trigger, Buffers...>;

    Synchronizer(Trigger* trigger, Buffers*... buffers)
        : all_buffers_(std::make_tuple(trigger, buffers...)),
          trigger_(trigger),
          terminated_(false),
          prev_timestamp_(-1.0) {
        if (!trigger_) throw std::invalid_argument("Trigger cannot be null");

        trigger_->register_as_trigger();
        align_thread_ = std::thread(&Synchronizer::align_loop, this);
    }

    ~Synchronizer() {
        terminated_ = true;

        if (trigger_) {
            uint64_t signal = 1;
            write(trigger_->get_eventfd(), &signal, sizeof(signal));
        }

        if (align_thread_.joinable()) {
            align_thread_.join();
        }
    }

    void set_dt_tolerance(double dt_tolerance) { dt_tolerance_ = dt_tolerance; }

    bool pop_package(DataPackage& out) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (data_queue_.empty()) {
            return false;
        }

        out = std::move(data_queue_.front());
        data_queue_.pop_front();
        return true;
    }

private:
    std::tuple<Trigger*, Buffers*...> all_buffers_;

    Trigger* trigger_ = nullptr;

    std::thread align_thread_;

    std::atomic<bool> terminated_;

    std::deque<DataPackage> data_queue_;

    int pending_num = 0;

    std::mutex queue_mutex_;

    double prev_timestamp_ = -1.0;

    double dt_tolerance_ = 0.005;

    void align_loop() {
        int efd = trigger_->get_eventfd();
        struct pollfd pfd = {.fd = efd, .events = POLLIN};

        while (!terminated_) {
            int ret = poll(&pfd, 1, 5);

            if (ret >= 0) {
                if (ret > 0 && (pfd.revents & POLLIN)) {
                    uint64_t count;
                    read(efd, &count, sizeof(count));
                }

                while (!trigger_->empty()) {
                    double current_ts = trigger_->get_oldest_timestamp();
                    if (current_ts < 0.0) break;

                    bool is_all_ready = std::apply(
                        [&](auto*... bufs) {
                            return (
                                this->check_data_ready(bufs, current_ts, prev_timestamp_) && ...);
                        },
                        all_buffers_);

                    if (is_all_ready) {
                        auto synced_results = std::apply(
                            [&](auto*... bufs) {
                                return std::make_tuple(
                                    this->extract_data(bufs, current_ts, prev_timestamp_)...);
                            },
                            all_buffers_);

                        DataPackage new_package;
                        bool move_success = true;
                        std::apply(
                            [&](auto&... opts) {
                                if ((opts.has_value() && ...)) {
                                    new_package.data = std::make_tuple(std::move(opts.value())...);
                                } else {
                                    move_success = false;
                                }
                            },
                            synced_results);

                        if (move_success) {
                            {
                                std::lock_guard<std::mutex> lock(queue_mutex_);
                                data_queue_.push_back(std::move(new_package));
                            }
                        }

                        trigger_->pop_front();
                        prev_timestamp_ = current_ts;
                        pending_num = 0;
                    } else {
                        if (pending_num < 2) {
                            pending_num++;
                            break;
                        } else {
                            trigger_->pop_front();
                        }
                    }
                }
            }
        }
    }

    template <typename BufferPtr>
    auto extract_data(BufferPtr buf, double current_ts, double prev_ts) {
        using BufType = typename std::remove_pointer_t<BufferPtr>;
        constexpr auto s_type = buffer_traits<BufType>::sensor_type;
        using StorageT = typename buffer_traits<BufType>::StorageType;

        std::optional<StorageT> result = std::nullopt;

        if constexpr (s_type == data::SensorType::IMU) {
            StorageT imu_data;
            buf->get_data_set(
                [&](void* item) {
                    imu_data.push_back(std::move(
                        *static_cast<typename buffer_traits<BufType>::HandleType*>(item)));
                },
                prev_ts, current_ts);

            if (!imu_data.empty()) {
                result = std::move(imu_data);
            }
        } else {
            buf->get_nearest_data(
                [&](void* item) {
                    result =
                        std::move(*static_cast<typename buffer_traits<BufType>::HandleType*>(item));
                },
                current_ts, dt_tolerance_);
        }

        return result;
    }

    template <typename BufferPtr>
    bool check_data_ready(BufferPtr buf, double current_ts, double prev_ts) {
        using BufType = typename std::remove_pointer_t<BufferPtr>;
        constexpr auto s_type = buffer_traits<BufType>::sensor_type;

        if constexpr (s_type == data::SensorType::IMU) {
            return buf->query_by_time_range(prev_ts, current_ts);
        } else {
            return buf->query_by_time_window(current_ts, dt_tolerance_);
        }
    }
};
};  // namespace data

#endif
