#ifndef BUFFER_TRAITS_H_
#define BUFFER_TRAITS_H_
#include <unistd.h>
#include <cstddef>

#include "buffer.h"
#include "sensorType.h"

namespace ts {
template <typename T>
struct buffer_traits;

template <typename Msg, SensorType ST, typename SP, BufferStrategy BS>
struct buffer_traits<Buffer<Msg, ST, SP, BS>> {
    using HandleType = typename SP::template Handle<Msg>;
    static constexpr SensorType sensor_type = ST;

    using StorageType =
        std::conditional_t<ST == SensorType::Slice, std::vector<HandleType>, HandleType>;
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

template <typename Message, SensorType Type, typename StoragePolicy, BufferStrategy Strategy>
struct is_data_buffer<Buffer<Message, Type, StoragePolicy, Strategy>> : std::true_type {};

}  // namespace ts
#endif
