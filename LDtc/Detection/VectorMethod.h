#ifndef LDTC_VECTORMETHOD_H
#define LDTC_VECTORMETHOD_H

#include <vector>

#include "../Structures/Geometric/Points/TimePoint.h"

namespace ld {
namespace det {
TimePoint<> vector_detect(const std::vector<TimePoint<>>& data, double c);

}
}  // namespace ld

#endif  // LDTC_VECTORMETHOD_H
