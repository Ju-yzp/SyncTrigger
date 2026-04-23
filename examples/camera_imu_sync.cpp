#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <opencv2/core/types.hpp>
#include <vector>

#include <Eigen/Core>
#include <depthai/depthai.hpp>
#include <opencv2/opencv.hpp>

#include "buffer.h"
#include "synchronizer.h"
#ifdef ENABLE_ANALYZE
#include "statistical_analyzer.h"
#endif

using namespace ts;

struct CameraFrame {
    double timestamp;
    double host_timestamp;
    uint32_t seq;
    cv::Mat image;
    double exposure_time;
    double get_timestamp() const { return timestamp; }
    void set_timestamp(double ts) { timestamp = ts; }
    double get_device_timestamp() const { return timestamp; }
    double get_host_timestamp() const { return host_timestamp; }
};

struct IMUMeasurement {
    double timestamp;
    double host_timestamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    double get_timestamp() const { return timestamp; }
    void set_timestamp(double ts) { timestamp = ts; }
    double get_device_timestamp() const { return timestamp; }
    double get_host_timestamp() const { return host_timestamp; }
};

using ImageHandle = std::shared_ptr<CameraFrame>;
using IMUHandle = IMUMeasurement;

IMUHandle IMUInterpolator(const IMUHandle& p0, const IMUHandle& p1, double t_target) {
    IMUHandle out;
    out.timestamp = t_target;
    double alpha = (t_target - p0.timestamp) / (p1.timestamp - p0.timestamp);
    out.acc = p0.acc + alpha * (p1.acc - p0.acc);
    out.gyro = p0.gyro + alpha * (p1.gyro - p0.gyro);
    out.host_timestamp = p0.host_timestamp + alpha * (p1.host_timestamp - p0.host_timestamp);
    return out;
}

using RGBBuf = Buffer<CameraFrame, SensorType::Nearest, SharedPtrStorage>;
using LeftBuf = Buffer<CameraFrame, SensorType::Nearest, SharedPtrStorage>;
using RightBuf = Buffer<CameraFrame, SensorType::Nearest, SharedPtrStorage>;
using IMUBuf = Buffer<IMUMeasurement, SensorType::Slice, ValueStorage>;

int main() {
    auto rgb_buffer = RGBBuf::createShared(10);
    auto left_buffer = LeftBuf::createShared(10);
    auto right_buffer = RightBuf::createShared(10);
    auto imu_buffer = IMUBuf::createShared(500);

    imu_buffer->set_interpolator(IMUInterpolator);

#ifdef ENABLE_ANALYZE
    auto analyzer = std::make_unique<ts::StatisticalAnalyzer>(500, 900, 50);
    analyzer->add_sensor(0, "RGB Camera", cv::Scalar(0, 255, 0));
    analyzer->add_sensor(1, "Left Camera", cv::Scalar(0, 0, 255));
    analyzer->add_sensor(2, "Right Camera", cv::Scalar(255, 0, 0));
    analyzer->add_sensor(3, "IMU", cv::Scalar(0, 255, 255));
#endif

    using SyncType = Synchronizer<RGBBuf, LeftBuf, RightBuf, IMUBuf>;
    SyncType sync(2, rgb_buffer.get(), left_buffer.get(), right_buffer.get(), imu_buffer.get());

    dai::Pipeline pipeline;

    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();

    monoLeft->setCamera("left");
    monoRight->setCamera("right");
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_480_P);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_480_P);
    monoLeft->setFps(80);
    monoRight->setFps(50);

    colorCam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setFps(30);
    colorCam->setInterleaved(false);

    auto imu = pipeline.create<dai::node::IMU>();
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 400);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(1);

    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    auto xoutImu = pipeline.create<dai::node::XLinkOut>();

    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    xoutImu->setStreamName("imu");
    xoutRgb->setStreamName("rgb");

    colorCam->isp.link(xoutRgb->input);
    monoLeft->out.link(xoutLeft->input);
    monoRight->out.link(xoutRight->input);
    imu->out.link(xoutImu->input);

    dai::Device device(pipeline);

    auto qLeft = device.getOutputQueue("left", 8, false);
    auto qRight = device.getOutputQueue("right", 8, false);
    auto qImu = device.getOutputQueue("imu", 60, false);
    auto qRgb = device.getOutputQueue("rgb", 8, false);

    auto process_img = [&](std::shared_ptr<dai::ImgFrame> daiFrame, auto& buf, bool isRgb,
                           size_t sensor_id) {
        if (!daiFrame) return;

        auto frame = std::make_shared<CameraFrame>();
        frame->timestamp = daiFrame->getTimestamp().time_since_epoch().count() / 1e9;
        frame->host_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::steady_clock::now().time_since_epoch())
                                    .count() /
                                1e9;
        frame->seq = daiFrame->getSequenceNum();
        frame->exposure_time = static_cast<double>(daiFrame->getExposureTime().count());

#ifdef ENABLE_ANALYZE
        analyzer->add_timestamp(sensor_id, frame->host_timestamp);
#endif

        if (isRgb) {
            int w = daiFrame->getWidth();
            int h = daiFrame->getHeight();
            std::vector<uint8_t> buffer = daiFrame->getData();
            cv::Mat yuv420p(h + h / 2, w, CV_8UC1, buffer.data());
            cv::cvtColor(yuv420p, frame->image, cv::COLOR_YUV2BGR_I420);
        } else {
            cv::Mat mono(
                daiFrame->getHeight(), daiFrame->getWidth(), CV_8UC1, daiFrame->getData().data());
            frame->image = mono.clone();
        }
        buf->push_back(frame);
    };

    qRgb->addCallback([&](std::shared_ptr<dai::ADatatype> data) {
        process_img(std::static_pointer_cast<dai::ImgFrame>(data), rgb_buffer, true, 0);
    });

    qLeft->addCallback([&](std::shared_ptr<dai::ADatatype> data) {
        process_img(std::static_pointer_cast<dai::ImgFrame>(data), left_buffer, false, 1);
    });

    qRight->addCallback([&](std::shared_ptr<dai::ADatatype> data) {
        process_img(std::static_pointer_cast<dai::ImgFrame>(data), right_buffer, false, 2);
    });

    qImu->addCallback([&](std::shared_ptr<dai::ADatatype> data) {
        auto imuData = std::static_pointer_cast<dai::IMUData>(data);
        auto host_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::steady_clock::now().time_since_epoch())
                             .count() /
                         1e9;
        for (auto& packet : imuData->packets) {
            IMUMeasurement m;
            m.timestamp = packet.acceleroMeter.timestamp.get().time_since_epoch().count() / 1e9;
            m.host_timestamp = host_time;
            m.acc << packet.acceleroMeter.x, packet.acceleroMeter.y, packet.acceleroMeter.z;
            m.gyro << packet.gyroscope.x, packet.gyroscope.y, packet.gyroscope.z;
            imu_buffer->push_back(m);

#ifdef ENABLE_ANALYZE
            // 向统计器添加主机时间戳
            analyzer->add_timestamp(3, m.host_timestamp);
#endif
        }
    });

    std::cout << "Stereo-IMU Synchronizer Running (Left Trigger)... Press 'q' to exit."
              << std::endl;

    while (true) {
        SyncType::DataPackage package;
        if (sync.pop_package(package)) {
            auto& rgb_ptr = package.get<0>();
            auto& left_ptr = package.get<1>();
            auto& right_ptr = package.get<2>();
            auto& imu_vec = package.get<3>();

            cv::Mat combined;
            cv::hconcat(left_ptr->image, right_ptr->image, combined);

            std::string info = "TS: " + std::to_string(left_ptr->timestamp) +
                               " IMUs: " + std::to_string(imu_vec.size());
            cv::putText(
                combined, info, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255),
                2);

            cv::Mat resize_rgb;
            double aspect_ratio = (double)rgb_ptr->image.cols / rgb_ptr->image.rows;
            int target_h = 480;
            int target_w = static_cast<int>(target_h * aspect_ratio);
            cv::resize(
                rgb_ptr->image, resize_rgb, cv::Size(target_w, target_h), 0, 0, cv::INTER_LINEAR);

            cv::imshow("Aligned Stereo (Left & Right)", combined);
            cv::imshow("RGB Camera", resize_rgb);

            double dt_ms = (right_ptr->timestamp - left_ptr->timestamp) * 1000.0;
            std::cout << "\r[Synced] Left TS: " << std::fixed << std::setprecision(4)
                      << left_ptr->timestamp << " | Stereo Drift: " << std::setw(6) << dt_ms
                      << " ms | IMU Pkgs: " << imu_vec.size() << std::flush;
        }

        if (cv::waitKey(1) == 'q') break;
    }

    return 0;
}
