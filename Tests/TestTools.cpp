#include "TestTools.h"

namespace ld {
namespace tst {
inline bool eq_double(double a, double b, double eps) {
    return (isnan(a) || isnan(b)) ? (isnan(a) == isnan(b))
                                  : (fabs(a - b) <= eps);
}

std::string comp_TimePoint(const TimePoint<>& real, const TimePoint<>& exp,
                           double eps) {
    std::string ans;
    if (!eq_double(real.x, exp.x, eps)) {
        ans += "Different in x:\nReal: " + std::to_string(real.x) +
               "\nExpected: " + std::to_string(exp.x) + "\n";
    }
    if (!eq_double(real.y, exp.y, eps)) {
        ans += "Different in x:\nReal: " + std::to_string(real.y) +
               "\nExpected: " + std::to_string(exp.y) + "\n";
    }
    if (!eq_double(real.time, exp.time, eps)) {
        ans += "Different in x:\nReal: " + std::to_string(real.time) +
               "\nExpected: " + std::to_string(exp.time) + "\n";
    }
    return ans;
}

}  // namespace tst
}  // namespace ld
