#ifndef STATISTICAL_ANALYZER_H_
#define STATISTICAL_ANALYZER_H_

#include <atomic>
#include <cstddef>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "plot_drawer.h"

namespace ts {
class StatisticalAnalyzer {
public:
    StatisticalAnalyzer(size_t height, size_t width, size_t interval);

    ~StatisticalAnalyzer();

    void add_sensor(size_t sensor_id, std::string sensor_name, cv::Scalar color);

    void add_timestamp(size_t sensor_id, double ts);

private:
    void render();

    void update_all_frame_ratio();

    double frame_ratio(std::vector<double> ts_vec);

    std::map<size_t, std::string> sensor_map_;
    std::map<size_t, std::vector<double>> ts_vec_map_;

    std::thread render_thread_;

    std::atomic<bool> terminated_;

    PlotDrawer plot_drawer_;
};
}  // namespace ts

#endif /* STATISTICAL_ANALYZER_H_ */
