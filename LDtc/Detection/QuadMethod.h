#ifndef LDTC_QUADMETHOD_H
#define LDTC_QUADMETHOD_H

#include <vector>

#include "../Structures/Geometric/Points/TimePoint.h"

namespace ld {
namespace det {
TimePoint<> quad_detect(const std::vector<TimePoint<>>& data, double c);

}
}  // namespace ld

#endif  // LDTC_QUADMETHOD_H
