#include <array>
#include <opencv2/opencv.hpp>
#include <utility>

#include "../../LDtc/Detection/DirectMethod.h"
#include "../../LDtc/Detection/QuadMethod.h"
#include "../../LDtc/Detection/VectorMethod.h"
#include "Src/DetectionMap.h"
#include "Src/MapIsolineVisualization.h"
#include "Src/MapVisualization.h"

int main() {
    const double c = 10;
    const double c_err = 0.005;
    const double time_err = 0.0005;

    const std::pair<unsigned int, unsigned int> map_size = {400, 400};
    cv::Size img_size = {400, 430};
    const std::pair<ld::Point<>, ld::Point<>> bound = {{-5, -5}, {5, 5}};

    const std::vector<ld::Point<>> detectors = {
        {-3, 0}, {3, 0}, {0, 3}, {-2, -2}};

    const std::array<std::pair<ld::model::FuncDetection, std::string>, 3>
        methods = {std::make_pair<ld::model::FuncDetection, std::string>(
                       ld::det::direct_detect, "direct_detect"),
                   std::make_pair<ld::model::FuncDetection, std::string>(
                       ld::det::vector_detect, "vector_detect"),
                   std::make_pair<ld::model::FuncDetection, std::string>(
                       ld::det::quad_detect, "quad_detect")};

    for (const auto& i : methods) {
        ld::model::DetectionMap map = ld::model::calculate_detection_map(
            i.first, map_size, bound, detectors, c, c * c_err, time_err);
        cv::imshow(i.second, ld::model::map2mat(map, img_size, 0.3));
    }

    cv::waitKey();
    return 0;
}
