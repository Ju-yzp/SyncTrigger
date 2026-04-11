#ifndef SYNCHRONIZER_H_
#define SYNCHRONIZER_H_

#include <dataBuffer.h>
#include <dataBuffer_base.h>
#include <storagePolicy.h>
#include <condition_variable>
#include <deque>
#include <stdexcept>
#include <thread>

namespace data {
template <typename T>
struct buffer_traits;

template <typename Msg, typename SP, data::BufferStrategy BS, data::SensorType ST>
struct buffer_traits<data::DataBuffer<Msg, SP, BS, ST>> {
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
    typename Message, typename StoragePolicy, data::BufferStrategy Strategy, data::SensorType Type>
struct is_data_buffer<data::DataBuffer<Message, StoragePolicy, Strategy, Type>> : std::true_type {};

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

        trigger_->as_trigger(cv_);
        align_thread_ = std::thread(&Synchronizer::align_loop, this);
    }

    ~Synchronizer() {
        terminated_ = true;
        cv_.notify_all();
        if (align_thread_.joinable()) {
            align_thread_.join();
        }
    }

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

    std::condition_variable cv_;

    std::deque<DataPackage> data_queue_;

    std::mutex queue_mutex_;

    double prev_timestamp_ = -1.0;

    // void align_loop() {
    //     while (!terminated_) {
    //         std::unique_lock<std::mutex> lock(queue_mutex_);
    //         cv_.wait(lock, [this] { return terminated_ || !trigger_->empty(); });

    //         if (terminated_) break;

    //         double current_ts = trigger_->get_lastest_timestamp();
    //         if (current_ts == -1.0) continue;

    //         auto optional_results = std::apply(
    //             [&](auto*... bufs) {
    //                 return std::make_tuple(extract_data(bufs, current_ts, prev_timestamp_)...);
    //             },
    //             all_buffers_);

    //         bool all_synced = std::apply(
    //             [](auto&... opts) { return (opts.has_value() && ...); }, optional_results);

    //         if (all_synced) {
    //             DataPackage new_package;

    //             std::apply(
    //                 [&](auto&... opts) {
    //                     new_package.data = std::make_tuple(std::move(opts.value())...);
    //                 },
    //                 optional_results);

    //             data_queue_.push_back(std::move(new_package));

    //             prev_timestamp_ = current_ts;
    //         }
    //     }
    // }
    // 
    void align_loop() {
        while (!terminated_) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            cv_.wait(lock, [this] { return terminated_ || !trigger_->empty(); });
    
            if (terminated_) break;
    
            double current_ts = trigger_->get_lastest_timestamp();
            if (current_ts == -1.0) continue;
    
            auto synced_results = std::apply(
                [&](auto*... bufs) {
                    return std::make_tuple(this->extract_data(bufs, current_ts, prev_timestamp_)...);
                },
                all_buffers_);
    
            auto is_all_synced = [](const auto& results) {
                return std::apply([](auto&... opts) { return (opts.has_value() && ...); }, results);
            };
    
            if (!is_all_synced(synced_results)) {
                lock.unlock(); 
                std::this_thread::sleep_for(std::chrono::milliseconds(3));
                lock.lock();
    
                constexpr auto TupleSize = std::tuple_size_v<decltype(all_buffers_)>;
                
                [&]<std::size_t... Is>(std::index_sequence<Is...>) {
                    (..., [&]{
                        auto& opt = std::get<Is>(synced_results);
                        if (!opt.has_value()) {
                            opt = this->extract_data(std::get<Is>(all_buffers_), current_ts, prev_timestamp_);
                        }
                    }());
                }(std::make_index_sequence<TupleSize>{});
            }
    
            if (is_all_synced(synced_results)) {
                DataPackage new_package;
                std::apply([&](auto&... opts) {
                    new_package.data = std::make_tuple(std::move(opts.value())...);
                }, synced_results);
    
                data_queue_.push_back(std::move(new_package));
                prev_timestamp_ = current_ts;
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
            buf->get_data_by_range(
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
                current_ts, 0.05);
        }

        return result;
    }
};
};  // namespace data

#endif
