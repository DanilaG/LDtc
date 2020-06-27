#ifndef LDTC_DIRECTMETHOD_H
#define LDTC_DIRECTMETHOD_H

#include <vector>

#include "../Structures/Geometric/Points/TimePoint.h"

namespace ld {
namespace det {
TimePoint<> direct_detect(const std::vector<TimePoint<>>& data, double c);

}
}

#endif  // LDTC_DIRECTMETHOD_H
