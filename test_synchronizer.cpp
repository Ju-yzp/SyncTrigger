#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <memory>
#include <thread>
#include <tuple>

#include <dataBuffer.h>
#include <storagePolicy.h>
#include <synchronizer.h>

#include <depthai/depthai.hpp>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

using namespace data;

struct CameraFrame {
    double timestamp;
    uint32_t seq;
    cv::Mat image;
    double exposure_time;
    double get_timestamp() const { return timestamp; }
};

struct IMUMeasurement {
    double timestamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    double get_timestamp() const { return timestamp; }
};

using ImageBuf = DataBuffer<CameraFrame, SharedPtrStorage, BufferStrategy::FixedSize, SensorType::Camera>;
using IMUBuf   = DataBuffer<IMUMeasurement, ValueStorage, BufferStrategy::FixedSize, SensorType::IMU>;

int main() {
    auto left_buffer  = new ImageBuf(10);
    auto right_buffer = new ImageBuf(10);
    auto imu_buffer   = new IMUBuf(500);

    using SyncType = Synchronizer<ImageBuf, ImageBuf, IMUBuf>;
    SyncType sync(left_buffer, right_buffer, imu_buffer);

    dai::Pipeline pipeline;

    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    monoLeft->setCamera("left");
    monoRight->setCamera("right");
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_480_P);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_480_P);
    monoLeft->setFps(30);
    monoRight->setFps(30);

    auto imu = pipeline.create<dai::node::IMU>();
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 400);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(1);

    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    auto xoutImu = pipeline.create<dai::node::XLinkOut>();

    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    xoutImu->setStreamName("imu");

    monoLeft->out.link(xoutLeft->input);
    monoRight->out.link(xoutRight->input);
    imu->out.link(xoutImu->input);

    dai::Device device(pipeline);
    auto qLeft = device.getOutputQueue("left", 8, false);
    auto qRight = device.getOutputQueue("right", 8, false);
    auto qImu = device.getOutputQueue("imu", 50, false);

    auto process_img = [](std::shared_ptr<dai::ImgFrame> daiFrame, ImageBuf* buf) {
        auto frame = std::make_shared<CameraFrame>();
        frame->timestamp = daiFrame->getTimestamp().time_since_epoch().count() / 1e9;
        frame->seq = daiFrame->getSequenceNum();

        frame->image = cv::Mat(daiFrame->getHeight(), daiFrame->getWidth(), 
                               CV_8UC1, daiFrame->getData().data()).clone();

        frame->exposure_time = static_cast<double>(daiFrame->getExposureTime().count());
        buf->push_back(frame);
    };

    qLeft->addCallback([&](std::shared_ptr<dai::ADatatype> data) { 
        process_img(std::static_pointer_cast<dai::ImgFrame>(data), left_buffer); 
    });
    qRight->addCallback([&](std::shared_ptr<dai::ADatatype> data) { 
        process_img(std::static_pointer_cast<dai::ImgFrame>(data), right_buffer); 
    });

    qImu->addCallback([&](std::shared_ptr<dai::ADatatype> data) {
        auto imuData = std::static_pointer_cast<dai::IMUData>(data);
        for (auto& packet : imuData->packets) {
            IMUMeasurement m;
            m.timestamp = packet.acceleroMeter.timestamp.get().time_since_epoch().count() / 1e9;
            m.acc << packet.acceleroMeter.x, packet.acceleroMeter.y, packet.acceleroMeter.z;
            m.gyro << packet.gyroscope.x, packet.gyroscope.y, packet.gyroscope.z;
            imu_buffer->push_back(m);
        }
    });

    std::cout << "Stereo-IMU Synchronizer Running... Press 'q' to exit." << std::endl;
    
    while (true) {
        SyncType::DataPackage package;
        if (sync.pop_package(package)) {
            auto& left_ptr  = package.get<0>();
            auto& right_ptr = package.get<1>();
            auto& imu_vec   = package.get<2>();

            cv::Mat combined;
            cv::hconcat(left_ptr->image, right_ptr->image, combined);
            
            std::string info = "TS: " + std::to_string(left_ptr->timestamp) + 
                               " IMUs: " + std::to_string(imu_vec.size());
            cv::putText(combined, info, cv::Point(20, 30), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255), 2);

            cv::imshow("Aligned Stereo (Left & Right)", combined);

            double dt_ms = (right_ptr->timestamp - left_ptr->timestamp) * 1000.0;
            std::cout << "\r[Synced] Left TS: " << std::fixed << std::setprecision(4) << left_ptr->timestamp 
                      << " | Stereo Drift: " << std::setw(6) << dt_ms << " ms"
                      << " | IMU Pkgs: " << imu_vec.size() << std::flush;
        }

        if (cv::waitKey(1) == 'q') break;
    }

    delete left_buffer;
    delete right_buffer;
    delete imu_buffer;

    return 0;
}