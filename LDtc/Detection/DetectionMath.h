#ifndef LDTC_DETECTIONMATH_H
#define LDTC_DETECTIONMATH_H

#include <array>

#include "../Structures/Geometric/Points/TimePoint.h"

namespace ld {
namespace det {
template <class T>
T sqr(const T& a) {
    return a * a;
}

template <class T>
inline void update_mass_data(T& a, const T& b, unsigned long mass) {
    a = (a * mass + b) / static_cast<double>(mass + 1);
}

}  // namespace det
}  // namespace ld

#endif  // LDTC_DETECTIONMATH_H
