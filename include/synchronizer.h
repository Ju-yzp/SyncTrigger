#ifndef SYNCHRONIZER_H_
#define SYNCHRONIZER_H_

#include <memory>
#include <vector>
#include "buffer_base.h"

namespace ts {
class Synchronizer {
public:
    using UniqueBufferBase = std::unique_ptr<BufferBase>;
    using SharedBufferBase = std::shared_ptr<BufferBase>;

private:
    // std::vector<>
};
}  // namespace ts

#endif
