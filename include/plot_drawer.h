#ifndef PLOT_DRAWER_H_
#define PLOT_DRAWER_H_

#include <cassert>
#include <cstddef>
#include <map>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace ts {
class PlotDrawer {
public:
    PlotDrawer(size_t height, size_t width, size_t interval = 30)
        : height_(height), width_(width), interval_(interval), cache_size_(width_ / interval_) {
        assert(height_ > 0 && width_ > 0 && cache_size_ > 0 && interval_ > 0);
        img_ = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(60, 60, 60));
    };

    cv::Mat draw() {
        img_.setTo(cv::Scalar(60, 60, 60));
        float lable_y = 30;
        float lable_dy = 30;
        for (auto& [name, pts] : points_map_) {
            int text_width = name.length() * 10;
            cv::putText(
                img_, name, cv::Point2f(width_ - text_width - 10, lable_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, colors_[name], 1);
            if (pts.size() > 1) {
                for (size_t i = 1; i < pts.size(); ++i) {
                    cv::line(img_, pts[i - 1], pts[i], colors_[name], 1, cv::LINE_AA);
                }
            }
            lable_y += lable_dy;
        }
        for (int i = height_; i > 0; i -= 80) {
            cv::line(img_, cv::Point2f(0, i), cv::Point2f(width_, i), cv::Scalar(255, 255, 255), 1);
            std::string label = std::to_string(height_ - i);
            cv::putText(
                img_, label, cv::Point2f(0, i), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(255, 255, 255), 1);
        }
        return img_;
    }

    void set_interval(int interval) { interval_ = interval; }

    void add_point(std::string name, double value) {
        std::vector<cv::Point2f>& pts = points_map_[name];
        if (pts.empty()) {
            pts.push_back(cv::Point2f(0, height_ - value));
        } else {
            pts.push_back(cv::Point2f(pts.back().x + interval_, height_ - value));
        }
        if (points_map_[name].size() > cache_size_) {
            points_map_[name].erase(points_map_[name].begin());
            for (auto& pt : pts) {
                pt.x -= interval_;
            }
        }
    }

    void set_color(std::string name, cv::Scalar color) {
        colors_[name] = color;
        points_map_[name] = std::vector<cv::Point2f>{

        };
    }

private:
    PlotDrawer() = delete;
    std::map<std::string, std::vector<cv::Point2f>> points_map_;
    std::map<std::string, cv::Scalar> colors_;
    size_t height_, width_;
    size_t interval_;
    size_t cache_size_;
    cv::Mat img_;
};
}  // namespace ts
#endif
