#include "MapVisualization.h"

#include <utility>

namespace ld {
namespace model {
cv::Scalar color_gradient(double data, double max, double alpha = 255) {
    if (data > max) {
        return {255, 0, 0, alpha};
    }
    return {(255 * data / max), (255 * (max - data) / max), 0, alpha};
}

void draw_err_grid(cv::Mat& mat, const ld::model::DetectionMap& map, double max,
                   bool dist) {
    ld::Point<double> step = {mat.size().width / double(map.size().first),
                              (mat.size().height) / double(map.size().second)};
    int height = mat.size().height;
    for (unsigned int i = 0; i < map.size().first; i++) {
        for (unsigned int j = 0; j < map.size().second; j++) {
            cv::rectangle(
                mat, {int(step.x * i), height - int(step.y * (j + 1))},
                {int(step.x * (i + 1)), height - int(step.y * j)},
                (std::isnan(dist ? map.get(i, j).first : map.get(i, j).second))
                    ? cv::Scalar{0, 0, 255}
                    : color_gradient(
                          dist ? map.get(i, j).first : map.get(i, j).second,
                          max),
                cv::FILLED);
        }
    }
}

void draw_stations(cv::Mat& mat, const std::vector<ld::Point<>>& detectors,
                   const std::pair<ld::Point<>, ld::Point<>>& bound) {
    const ld::Point<double> zoom = {
        mat.size().width / double(bound.second.x - bound.first.x),
        (mat.size().height) / double(bound.second.y - bound.first.y)};
    for (auto& i : detectors) {
        cv::circle(mat,
                   {int((i.x - bound.first.x) * zoom.x),
                    mat.size().height - int((i.y - bound.first.y) * zoom.y)},
                   3, {0, 0, 0}, cv::FILLED);
    }
}

void draw_gradient_bar(cv::Mat& mat, double max) {
    const unsigned int start_text_length = 15;
    const unsigned int end_text_length = 90;
    const unsigned int line_number =
        mat.size().width - start_text_length - end_text_length;
    cv::rectangle(mat, {mat.size().width, mat.size().height}, {0, 0},
                  {255, 255, 255}, cv::FILLED);
    for (unsigned int i = 0; i < line_number; i++) {
        cv::line(mat, {int(start_text_length + i), 5},
                 {int(start_text_length + i), mat.size().height - 5},
                 color_gradient(i, line_number - 1), 1);
    }
    std::stringstream oss;
    oss << std::scientific << " " << max;
    cv::putText(mat, " 0", {0, mat.size().height - 12}, cv::FONT_HERSHEY_DUPLEX,
                0.3, {0, 0, 0, 0});
    cv::putText(
        mat, oss.str(),
        {int(mat.size().width - end_text_length), mat.size().height - 12},
        cv::FONT_HERSHEY_DUPLEX, 0.3, {0, 0, 0, 0});
}

cv::Mat map2mat(const ld::model::DetectionMap& map, const cv::Size& size,
                double max, bool dist) {
    const int gradient_bar_height = 30;

    cv::Mat mat(size, CV_8UC3);
    auto field = mat(cv::Rect(0, 0, mat.size().width,
                              mat.size().height - gradient_bar_height));
    auto gradient_bar = mat(cv::Rect(0, mat.size().height - gradient_bar_height,
                                     mat.size().width, gradient_bar_height));
    draw_err_grid(field, map, max, dist);
    draw_stations(field, map.detectors, map.bound);
    draw_gradient_bar(gradient_bar, max);
    return mat;
}

}  // namespace model
}  // namespace ld
