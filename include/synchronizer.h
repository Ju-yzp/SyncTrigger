#ifndef SYNCHRONIZER_H_
#define SYNCHRONIZER_H_

// cpp
#include <poll.h>
#include <unistd.h>
#include <cstddef>
#include <deque>
#include <stdexcept>
#include <thread>

#include "bufferManager.h"
#include "buffer_traits.h"

namespace ts {

template <typename... Buffers>
class Synchronizer {
public:
    static_assert(
        (is_data_buffer<Buffers>::value && ...), "All types must be DataBuffer specializations");

    using DataPackage = DataPacket<Buffers...>;

    explicit Synchronizer(size_t trigger_id, Buffers*... buffers)
        : trigger_id_(trigger_id), terminated_(false), prev_ts_(-1.0), btm_(buffers...) {
        if (trigger_id_ >= btm_.size()) {
            throw std::invalid_argument("Trigger ID out of range");
        }

        btm_.execute(trigger_id_, [](auto* buf) { buf->register_as_trigger(); });

        sync_thread_ = std::thread(&Synchronizer::sync_loop, this);
    }

    ~Synchronizer() {
        terminated_ = true;
        uint64_t signal = 1;
        write(btm_.get_fd(trigger_id_), &signal, sizeof(signal));

        if (sync_thread_.joinable()) sync_thread_.join();
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
    void sync_loop() {
        int efd = btm_.get_fd(trigger_id_);
        struct pollfd pfd = {.fd = efd, .events = POLLIN};

        while (!terminated_) {
            int ret = poll(&pfd, 1, 10);
            if (ret > 0 && (pfd.revents & POLLIN)) {
                uint64_t count;
                read(efd, &count, sizeof(count));
            }

            while (!btm_.is_buffer_empty(trigger_id_)) {
                double current_ts = btm_.get_oldest_ts(trigger_id_);
                if (current_ts < 0.0) break;

                bool all_ready = true;
                for (size_t i = 0; i < btm_.size(); ++i) {
                    if (!btm_.check_ready(i, current_ts, prev_ts_)) {
                        all_ready = false;
                        break;
                    }
                }

                if (all_ready) {
                    std::cout << "Fill package successful: Data not aligned." << std::endl;
                    DataPackage package;
                    if (fill_package(package, current_ts)) {
                        std::lock_guard<std::mutex> lock(queue_mutex_);
                        data_queue_.push_back(std::move(package));
                    }

                    btm_.pop_front(trigger_id_);
                    prev_ts_ = current_ts;
                    pending_num_ = 0;
                } else {
                    std::cout << "Fill package failed: Data not aligned." << std::endl;
                    if (pending_num_ < 2) {
                        pending_num_++;
                        break;
                    } else {
                        btm_.pop_front(trigger_id_);
                        pending_num_ = 0;
                    }
                }
            }
        }
    }

    template <std::size_t... Is>
    bool fill_helper(DataPackage& pkg, double current_ts, std::index_sequence<Is...>) {
        return ([&]() -> bool {
            std::any raw = btm_.fetch_data(Is, current_ts, prev_ts_);
            if (!raw.has_value()) return false;

            using TargetType =
                typename std::tuple_element<Is, typename DataPackage::TupleType>::type;

            try {
                std::get<Is>(pkg.data) = std::move(std::any_cast<TargetType&>(raw));
            } catch (const std::bad_any_cast&) {
                return false;
            }
            return true;
        }() && ...);
    }

    bool fill_package(DataPackage& pkg, double current_ts) {
        return fill_helper(pkg, current_ts, std::index_sequence_for<Buffers...>{});
    }

    size_t trigger_id_;
    BufferManager<Buffers...> btm_;
    std::thread sync_thread_;
    std::atomic<bool> terminated_;
    double prev_ts_;
    int pending_num_ = 0;
    std::deque<DataPackage> data_queue_;
    std::mutex queue_mutex_;
};
}  // namespace ts

#endif
