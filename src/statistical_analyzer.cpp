#include "statistical_analyzer.h"
#include <unistd.h>
#include <cstddef>
#include "plot_drawer.h"

namespace ts {
StatisticalAnalyzer::StatisticalAnalyzer(size_t height, size_t width, size_t interval)
    : plot_drawer_(height, width, interval), terminated_(false) {
    render_thread_ = std::thread(&StatisticalAnalyzer::render, this);
}

StatisticalAnalyzer::~StatisticalAnalyzer() {
    terminated_.store(true);
    if (render_thread_.joinable()) {
        render_thread_.join();
    }
}
void StatisticalAnalyzer::render() {
    while (!terminated_.load()) {
        update_all_frame_ratio();
        cv::Mat plot_img = plot_drawer_.draw();
        cv::imshow("StatisticalAnalyzer", plot_img);
        cv::waitKey(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void StatisticalAnalyzer::add_sensor(size_t sensor_id, std::string sensor_name, cv::Scalar color) {
    sensor_map_.insert({sensor_id, sensor_name});
    ts_vec_map_.insert({sensor_id, std::vector<double>()});
    plot_drawer_.set_color(sensor_name, color);
}

void StatisticalAnalyzer::add_timestamp(size_t sensor_id, double timestamp) {
    ts_vec_map_[sensor_id].push_back(timestamp);
}

void StatisticalAnalyzer::update_all_frame_ratio() {
    constexpr double time_thres = 1.2;
    for (auto& [id, ts_vec] : ts_vec_map_) {
        if (!ts_vec.empty()) {
            double ts_diff = ts_vec.back() - ts_vec.front();
            if (ts_diff > time_thres && ts_vec.size() > 4) {
                double fps = (ts_vec.size() - 1) / ts_diff;
                plot_drawer_.add_point(sensor_map_[id], fps);
                ts_vec.clear();
            }
        }
    }
}
}  // namespace ts
