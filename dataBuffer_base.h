#ifndef DATA_BUFFER_BASE_H_
#define DATA_BUFFER_BASE_H_

#include <condition_variable>
#include <cstdint>
#include <functional>

namespace data {
// Only supports these sensor types
enum class SensorType : uint8_t { Camera = 0, LiDAR = 1, IMU = 2, Wheel_Speed_Sensor = 3 };

using ItemCallback = std::function<void(void*)>;

class DataBufferBase {
public:
    virtual ~DataBufferBase() = default;

    virtual bool empty() = 0;

    virtual std::size_t size() = 0;

    virtual void clear() = 0;

    virtual void get_nearest_data(ItemCallback cb, double ts, double tolerance) = 0;

    virtual void get_data_by_range(ItemCallback cb, double ts_begin, double ts_end) = 0;

    virtual double get_lastest_timestamp() = 0;

    void as_trigger(std::condition_variable& cv) { notify_cv_ = &cv; }

protected:
    std::condition_variable* notify_cv_ = nullptr;
};
}  // namespace data

#endif
