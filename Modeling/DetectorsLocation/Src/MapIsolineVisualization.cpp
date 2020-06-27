#include "MapIsolineVisualization.h"

#include "Mat.h"

#define LINE(TYPE) (draw_isoline_##TYPE(mat, rect, color, width))

#define DRAW_ISOLINES(LINES)                                        \
    [](cv::Mat& mat, const cv::Rect& rect, const cv::Scalar& color, \
       unsigned int width) { (void)(LINES); }

namespace ld {
namespace model {
typedef unsigned char IsolineType;

void draw_isoline_left_up(cv::Mat& mat, const cv::Rect& rect,
                          const cv::Scalar& color, unsigned int width) {
    cv::line(mat, {rect.x, rect.y + rect.height / 2},
             {rect.x + rect.width / 2, rect.y + rect.height}, color, width);
}

void draw_isoline_right_up(cv::Mat& mat, const cv::Rect& rect,
                           const cv::Scalar& color, unsigned int width) {
    cv::line(mat, {rect.x + rect.width / 2, rect.y + rect.height},
             {rect.x + rect.width, rect.y + rect.height / 2}, color, width);
}

void draw_isoline_right_down(cv::Mat& mat, const cv::Rect& rect,
                             const cv::Scalar& color, unsigned int width) {
    cv::line(mat, {rect.x + rect.width, rect.y + rect.height / 2},
             {rect.x + rect.width / 2, rect.y}, color, width);
}

void draw_isoline_left_down(cv::Mat& mat, const cv::Rect& rect,
                            const cv::Scalar& color, unsigned int width) {
    cv::line(mat, {rect.x, rect.y + rect.height / 2},
             {rect.x + rect.width / 2, rect.y}, color, width);
}

void draw_isoline_horizontal(cv::Mat& mat, const cv::Rect& rect,
                             const cv::Scalar& color, unsigned int width) {
    cv::line(mat, {rect.x, rect.y + rect.height / 2},
             {rect.x + rect.width, rect.y + rect.height / 2}, color, width);
}

void draw_isoline_vertical(cv::Mat& mat, const cv::Rect& rect,
                           const cv::Scalar& color, unsigned int width) {
    cv::line(mat, {rect.x + rect.width / 2, rect.y},
             {rect.x + rect.width / 2, rect.y + rect.height}, color, width);
}

void (*const draw_isoline[16])(cv::Mat&, const cv::Rect&, const cv::Scalar&,
                               unsigned int) = {
    [](cv::Mat&, const cv::Rect&, const cv::Scalar&, unsigned int) {},  // 0000
    DRAW_ISOLINES(LINE(left_up)),                                       // 0001
    DRAW_ISOLINES(LINE(right_up)),                                      // 0010
    DRAW_ISOLINES(LINE(horizontal)),                                    // 0011
    DRAW_ISOLINES(LINE(right_down)),                                    // 0100
    DRAW_ISOLINES((LINE(left_down), LINE(right_up))),                   // 0101
    DRAW_ISOLINES(LINE(vertical)),                                      // 0110
    DRAW_ISOLINES(LINE(left_down)),                                     // 0111
    DRAW_ISOLINES(LINE(left_down)),                                     // 1000
    DRAW_ISOLINES(LINE(vertical)),                                      // 1001
    DRAW_ISOLINES((LINE(left_up), LINE(right_down))),                   // 1010
    DRAW_ISOLINES(LINE(right_down)),                                    // 1011
    DRAW_ISOLINES(LINE(horizontal)),                                    // 1100
    DRAW_ISOLINES(LINE(right_up)),                                      // 1101
    DRAW_ISOLINES(LINE(left_up)),                                       // 1110
    [](cv::Mat&, const cv::Rect&, const cv::Scalar&, unsigned int) {},  // 1111
};

inline bool in_isoline(double isoline, const std::pair<double, double>& data,
                       bool dist) {
    return (dist ? data.first : data.second) >= isoline;
}

Mat<IsolineType> get_isolation_mat(const ld::model::DetectionMap& map,
                                   double isoline, bool dist) {
    const std::pair<unsigned int, unsigned int> isoline_mat_size = {
        map.size().first - 1, map.size().second - 1};
    Mat<IsolineType> isoline_mat(isoline_mat_size);
    for (unsigned int x = 0; x < isoline_mat_size.first; x++) {
        for (unsigned int y = 0; y < isoline_mat_size.second; y++) {
            isoline_mat.get(x, y) =
                (in_isoline(isoline, map.get(x + 1, y), dist) << 3) |
                (in_isoline(isoline, map.get(x + 1, y + 1), dist) << 2) |
                (in_isoline(isoline, map.get(x, y + 1), dist) << 1) |
                in_isoline(isoline, map.get(x, y), dist);
        }
    }
    return isoline_mat;
}

void add_isoline(cv::Mat& mat, const ld::model::DetectionMap& map,
                 Isoline isoline, bool dist) {
    auto isoline_mat = get_isolation_mat(map, isoline.value, dist);
    auto isoline_mat_size = isoline_mat.size();
    std::pair<double, double> step = {
        mat.size().width / isoline_mat_size.first,
        mat.size().height / isoline_mat_size.second};
    for (unsigned int x = 0; x < isoline_mat_size.first; x++) {
        for (unsigned int y = 0; y < isoline_mat_size.second; y++) {
            draw_isoline[isoline_mat.get(x, isoline_mat_size.second - y - 1)](
                mat,
                {static_cast<int>(step.first * x),
                 static_cast<int>(step.second * y),
                 static_cast<int>(step.first), static_cast<int>(step.second)},
                isoline.color, isoline.width);
        }
    }
}

void draw_stations(cv::Mat& mat, const ld::model::DetectionMap& map) {
    const ld::Point<double> zoom = {
        mat.size().width / double(map.bound.second.x - map.bound.first.x),
        (mat.size().height) / double(map.bound.second.y - map.bound.first.y)};
    for (auto& i : map.detectors) {
        cv::circle(
            mat,
            {int((i.x - map.bound.first.x) * zoom.x),
             mat.size().height - int((i.y - map.bound.first.y) * zoom.y)},
            3, {0, 0, 0}, cv::FILLED);
    }
}

cv::Mat map_isolines(const ld::model::DetectionMap& map, const cv::Size& size,
                     const std::vector<Isoline>& isolines, bool dist) {
    cv::Mat mat(size, CV_8UC3);
    cv::rectangle(mat, {0, 0}, {size.width, size.height}, {255, 255, 255},
                  cv::FILLED);
    for (unsigned int i = 0; i < isolines.size(); i++) {
        add_isoline(mat, map, isolines[i], dist);
    }
    draw_stations(mat, map);
    return mat;
}

}  // namespace model
}  // namespace ld
