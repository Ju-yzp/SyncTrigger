#include <dataBuffer.h>
#include <cassert>
#include <iostream>
struct Image {
    double get_timestamp() const { return timestamp; }
    double timestamp;
    int id;
};

using namespace data;

void test_shared_policy() {
    std::cout << "--- Testing SharedStorage Policy ---" << std::endl;

    DataBuffer<Image, SharedPtrStorage, BufferStrategy::Dynamic, SensorType::Camera> buffer(100);

    for (int i = 0; i < 10; ++i) {
        auto img = std::make_shared<Image>(static_cast<double>(i), i);
        buffer.push_back(img);
    }
    assert(buffer.size() == 10);
    std::vector<std::shared_ptr<Image>> results;
    auto cb = [&results](void* item) {
        auto img = *static_cast<std::shared_ptr<Image>*>(item);
        results.push_back(img);
    };
    buffer.get_data_by_range(cb, 3.0, 6.0);

    std::cout << "Selected " << results.size() << " images." << std::endl;
    assert(results.size() == 3);
    assert(results[0]->id == 3);
    assert(results[2]->id == 5);

    assert(buffer.size() == 4);
    std::cout << "SharedStorage Test Passed!" << std::endl;
}

void test_unique_policy() {
    std::cout << "\n--- Testing UniqueStorage Policy ---" << std::endl;

    DataBuffer<Image, UniquePtrStorage, BufferStrategy::Dynamic, SensorType::Camera> buffer(5);
    for (int i = 0; i < 10; ++i) {
        buffer.push_back(std::make_unique<Image>(static_cast<double>(i), i));
    }
    assert(buffer.size() == 5);
    std::vector<std::unique_ptr<Image>> results;
    auto cb = [&results](void* item) {
        results.push_back(std::move(*static_cast<std::unique_ptr<Image>*>(item)));
    };
    buffer.get_data_by_range(cb, 7.0, 9.0);

    assert(results.size() == 2);
    assert(results[0]->id == 7);
    assert(results[0] != nullptr);
    assert(buffer.size() == 1);

    std::cout << "UniqueStorage Test Passed!" << std::endl;
}

int main() {
    try {
        test_shared_policy();
        test_unique_policy();
        std::cout << "\nAll tests passed successfully!" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
