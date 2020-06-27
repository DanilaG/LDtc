#ifndef LDTC_DETECTIONMAP_H
#define LDTC_DETECTIONMAP_H

#include <utility>
#include <vector>

#include "../../../LDtc/Structures/Geometric/Points/Point.h"
#include "../../../LDtc/Structures/Geometric/Points/TimePoint.h"
#include "Mat.h"

namespace ld {
namespace model {
typedef TimePoint<> (*FuncDetection)(const std::vector<TimePoint<>>&, double);

class DetectionMap;
DetectionMap calculate_detection_map(
    FuncDetection fun, const std::pair<unsigned int, unsigned int>& size,
    const std::pair<Point<>, Point<>>& bound,
    const std::vector<Point<>>& detectors, double c, double c_err = 0,
    double time_err = 0);

class DetectionMap : public Mat<std::pair<double, double>> {
   public:
    DetectionMap(const std::pair<unsigned int, unsigned int>& size,
                 const std::pair<Point<>, Point<>>& bound,
                 const std::vector<Point<>>& detectors)
        : Mat(size), bound(bound), detectors(detectors) {}

    friend DetectionMap calculate_detection_map(
        FuncDetection fun, const std::pair<unsigned int, unsigned int>& size,
        const std::pair<Point<>, Point<>>& bound,
        const std::vector<Point<>>& detectors, double c, double c_err,
        double time_err);

    std::pair<Point<>, Point<>> bound;
    std::vector<Point<>> detectors;
};

[[nodiscard]] DetectionMap blur(const DetectionMap&,
                                unsigned int);  // Not advisable to use

}  // namespace model
}  // namespace ld

#endif  // LDTC_DETECTIONMAP_H
