#ifndef SENSOR_TYPE_H_
#define SENSOR_TYPE_H_

#include <cstdint>

namespace ts {
enum class SensorType : uint8_t { Nearest = 0, Slice = 1 };
}

#endif
