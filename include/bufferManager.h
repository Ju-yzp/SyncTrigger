#ifndef BUFFER_MANAGER_H_
#define BUFFER_MANAGER_H_

#include <any>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "buffer_traits.h"

namespace ts {

template <typename... BufferTypes>
class BufferManager {
public:
    BufferManager(BufferTypes*... bufs) { instances = {std::any(bufs)...}; }

    size_t size() const { return instances.size(); }
    bool empty() const { return instances.empty(); }

    template <typename F>
    void execute(size_t id, F&& func) {
        if (id >= instances.size()) throw std::out_of_range("Buffer ID out of range");
        execute_helper(id, std::forward<F>(func), std::index_sequence_for<BufferTypes...>{});
    }

    bool is_buffer_empty(size_t id) {
        bool result = true;
        execute(id, [&](auto* buf) { result = buf->empty(); });
        return result;
    }

    int get_fd(size_t id) {
        int fd = -1;
        execute(id, [&](auto* buf) { fd = buf->get_eventfd(); });
        return fd;
    }

    double get_oldest_ts(size_t id) {
        double ts = -1.0;
        execute(id, [&](auto* buf) { ts = buf->get_oldest_timestamp(); });
        return ts;
    }

    double get_newest_ts(size_t id) {
        double ts = -1.0;
        execute(id, [&](auto* buf) { ts = buf->get_newest_timestamp(); });
        return ts;
    }

    bool check_ready(size_t id, double current_ts, double prev_ts) {
        bool ready = false;
        execute(id, [&](auto* buf) {
            using BufT = std::remove_pointer_t<decltype(buf)>;
            if constexpr (buffer_traits<BufT>::sensor_type == SensorType::Slice) {
                ready = buf->isSliceAvailable(prev_ts, current_ts);
            } else {
                ready = buf->isPresent(current_ts);
            }
        });
        return ready;
    }

    std::any fetch_data(size_t id, double current_ts, double prev_ts) {
        std::any result;
        execute(id, [&](auto* buf) {
            using BufType = std::remove_pointer_t<decltype(buf)>;
            constexpr auto s_type = buffer_traits<BufType>::sensor_type;

            if constexpr (s_type == SensorType::Slice) {
                result = std::make_any<typename buffer_traits<BufType>::StorageType>(
                    buf->fetchSlice(prev_ts, current_ts));
            } else {
                auto opt_handle = buf->fetchNearest(current_ts);
                if (opt_handle) result = std::any(std::move(*opt_handle));
            }
        });
        return result;
    }

    void pop_front(size_t id) {
        execute(id, [](auto* buf) { buf->pop_front(); });
    }

private:
    template <typename F, std::size_t... Is>
    void execute_helper(size_t id, F&& func, std::index_sequence<Is...>) {
        ((id == Is ? func(std::any_cast<
                          typename std::tuple_element<Is, std::tuple<BufferTypes*...>>::type>(
                         instances[Is]))
                   : (void)0),
         ...);
    }

    BufferManager() = delete;
    std::vector<std::any> instances;
};

}  // namespace ts

#endif
