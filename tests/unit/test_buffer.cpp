#include <gtest/gtest.h>

#include "buffer.h"

using namespace ts;

struct MockMessage {
    double timestamp;
    double get_timestamp() const { return timestamp; }
    void set_timestamp(double ts) { timestamp = ts; }
};

TEST(BufferTest, BasicPushAndCapacity) {
    auto buffer = Buffer<
        MockMessage, SensorType::Nearest, ValueStorage, BufferStrategy::FixedSize>::createUnique(2);

    EXPECT_TRUE(buffer->empty());

    buffer->push_back({1.0});
    buffer->push_back({2.0});
    EXPECT_EQ(buffer->size(), 2);

    buffer->push_back({3.0});
    EXPECT_EQ(buffer->size(), 2);
    EXPECT_DOUBLE_EQ(buffer->get_oldest_timestamp(), 2.0);
}

TEST(BufferTest, MonotonicityProtection) {
    auto buffer = Buffer<MockMessage, SensorType::Nearest, ValueStorage>::createUnique(10);

    buffer->push_back({10.0});
    buffer->push_back({9.0});

    EXPECT_EQ(buffer->size(), 1);
    EXPECT_DOUBLE_EQ(buffer->get_newest_timestamp(), 10.0);
}

TEST(BufferTest, FetchSliceWithInterpolation) {
    auto buffer = Buffer<MockMessage, SensorType::Slice, ValueStorage>::createUnique(10);

    buffer->set_interpolator([](const MockMessage& lhs, const MockMessage& rhs, double ts) {
        MockMessage msg;
        msg.timestamp = ts;
        return msg;
    });

    buffer->push_back({1.0});
    buffer->push_back({2.0});
    buffer->push_back({3.0});

    auto slice = buffer->fetchSlice(1.0, 2.5);

    ASSERT_EQ(slice.size(), 3);
    EXPECT_DOUBLE_EQ(slice.back().get_timestamp(), 2.5);

    EXPECT_EQ(buffer->size(), 2);
}

#include <atomic>
#include <thread>

TEST(BufferTest, ConcurrencyStress) {
    auto buffer = Buffer<MockMessage, SensorType::Nearest, ValueStorage>::createShared(1000);
    std::atomic<bool> stop{false};

    std::thread writer([&]() {
        double ts = 0.0;
        while (!stop) {
            buffer->push_back({ts});
            ts += 0.01;
            std::this_thread::yield();
        }
    });

    std::thread reader([&]() {
        while (!stop) {
            buffer->size();
            buffer->get_oldest_timestamp();
            std::this_thread::yield();
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    stop = true;
    writer.join();
    reader.join();

    SUCCEED();
}
