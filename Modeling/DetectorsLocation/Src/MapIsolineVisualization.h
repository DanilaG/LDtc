#ifndef LDTC_MAPISOLINEVISUALIZATION_H
#define LDTC_MAPISOLINEVISUALIZATION_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "DetectionMap.h"

namespace ld {
namespace model {
struct Isoline {
    double value;
    cv::Scalar color;
    unsigned int width = 2;
};

[[nodiscard]] cv::Mat map_isolines(const ld::model::DetectionMap& map,
                                   const cv::Size& size,
                                   const std::vector<Isoline>& isolines,
                                   bool dist = true);  // Not advisable to use

}  // namespace model
}  // namespace ld

#endif  // LDTC_MAPISOLINEVISUALIZATION_H
