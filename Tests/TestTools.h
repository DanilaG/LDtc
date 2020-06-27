#ifndef LDTC_TESTTOOLS_H
#define LDTC_TESTTOOLS_H

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>

#include "../LDtc/Structures/Geometric/Points/TimePoint.h"

#define TIME_POINT_EQ(A, B)                                  \
    {                                                        \
        std::string s = ld::tst::comp_TimePoint(A, B, 1e-7); \
        if (!s.empty()) {                                    \
            FAIL() << s;                                     \
        }                                                    \
    }

namespace ld {
namespace tst {
std::string comp_TimePoint(const TimePoint<>& real, const TimePoint<>& exp,
                           double eps);

}
}  // namespace ld

#endif  // LDTC_TESTTOOLS_H
