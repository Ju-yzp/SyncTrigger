#ifndef DATA_BUFFER_BASE_H_
#define DATA_BUFFER_BASE_H_

#include <sys/eventfd.h>
#include <unistd.h>
#include <cstdint>
#include <functional>
#include <stdexcept>

namespace data {
// Only supports these sensor types
enum class SensorType : uint8_t { Camera = 0, LiDAR = 1, IMU = 2, Wheel_Speed_Sensor = 3 };

using Callback = std::function<void(void*)>;

using OnInterpolate = std::function<void(void*, void*, void*)>;

class DataBufferBase {
public:
    ~DataBufferBase() {
        if (efd_ >= 0) {
            close(efd_);
        }
    }

    virtual bool empty() = 0;

    virtual std::size_t size() = 0;

    virtual void clear() = 0;

    virtual bool query_by_time_range(double ts_begin, double ts_end) = 0;

    virtual bool query_by_time_window(double ts, double tolerance) = 0;

    virtual void get_nearest_data(Callback cb, double ts, double tolerance) = 0;

    virtual void get_data_set(Callback cb, double ts_begin, double ts_end) = 0;

    virtual double get_oldest_timestamp() = 0;

    virtual void pop_front() = 0;

    void register_as_trigger() {
        if (efd_ >= 0) {
            close(efd_);
            efd_ = -1;
        }

        efd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);

        if (efd_ < 0) {
            throw  std::runtime_error("Failed to create eventfd");
            trigger_flag_ = false;
            return;
        }

        trigger_flag_ = true;
    }

    int get_eventfd() const { return efd_; }

    void set_interpolate(OnInterpolate oi) { interpolate_f_ = oi; }

protected:
    bool trigger_flag_ = false;
    int efd_ = -1;
    OnInterpolate interpolate_f_;
};
}  // namespace data

#endif
