#ifndef LDTC_MAPVISUALIZATION_H
#define LDTC_MAPVISUALIZATION_H

#include <opencv2/opencv.hpp>

#include "DetectionMap.h"

namespace ld {
namespace model {
[[nodiscard]] cv::Mat map2mat(const ld::model::DetectionMap& map,
                              const cv::Size& size, double max,
                              bool dist = true);

}
}  // namespace ld

#endif  // LDTC_MAPVISUALIZATION_H
