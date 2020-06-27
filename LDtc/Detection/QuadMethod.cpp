#include "QuadMethod.h"

#include "DetectionMath.h"

namespace ld {
namespace det {
inline double count_det(const double* a, const double* b, const double* c) {
    return a[0] * b[1] * c[2] + a[1] * b[2] * c[0] + b[0] * c[1] * a[2] -
           c[0] * b[1] * a[2] - a[1] * b[0] * c[2] - b[2] * c[1] * a[0];
}

TimePoint<> quad_detect(const TimePoint<>& p0, TimePoint<> p1, TimePoint<> p2,
                        TimePoint<> p3, double c) {
    p1 = p1 - p0;
    p2 = p2 - p0;
    p3 = p3 - p0;

    const double dist_p1 = dist(p1);
    if (dist_p1 == 0) {
        return {NAN, NAN, NAN};
    }
    const double cos_a = p1.y / dist_p1;
    const double sin_a = p1.x / dist_p1;
    p1 = {p1.x * cos_a - p1.y * sin_a, p1.x * sin_a + p1.y * cos_a, p1.time};
    p2 = {p2.x * cos_a - p2.y * sin_a, p2.x * sin_a + p2.y * cos_a, p2.time};
    p3 = {p3.x * cos_a - p3.y * sin_a, p3.x * sin_a + p3.y * cos_a, p3.time};

    const double c_sqr = sqr(c);

    const double A[3][3] = {
        {0, -p2.x, -p3.x},
        {-p1.y, -p2.y, -p3.y},
        {p1.time * c_sqr, p2.time * c_sqr, p3.time * c_sqr}};
    const double B[3] = {(sqr(p1.time) * c_sqr - sqr(p1.y)) / 2,
                         (sqr(p2.time) * c_sqr - sqr(p2.y) - sqr(p2.x)) / 2,
                         (sqr(p3.time) * c_sqr - sqr(p3.y) - sqr(p3.x)) / 2};

    const double det = count_det(A[0], A[1], A[2]);
    if (det == 0) {
        return {NAN, NAN, NAN};
    }
    TimePoint<> ans = {count_det(B, A[1], A[2]) / det,
                       count_det(A[0], B, A[2]) / det,
                       count_det(A[0], A[1], B) / det};
    return {ans.x * cos_a + ans.y * sin_a + p0.x,
            -ans.x * sin_a + ans.y * cos_a + p0.y, ans.time + p0.time};
}

TimePoint<> quad_detect(const std::vector<TimePoint<>>& data, double c) {
    TimePoint ans = {NAN, NAN, NAN};
    unsigned long count_points = 0;
    for (unsigned int i = 0; i < data.size() - 3; i++) {
        for (unsigned int j = i + 1; j < data.size() - 2; j++) {
            for (unsigned int k = j + 1; k < data.size() - 1; k++) {
                for (unsigned int m = k + 1; m < data.size(); m++) {
                    TimePoint<> r =
                        quad_detect(data[i], data[j], data[k], data[m], c);
                    if (!std::isnan(r.x)) {
                        if (count_points == 0) [[unlikely]] {
                                ans = r;
                            }
                        else {
                            update_mass_data(ans.x, r.x, count_points);
                            update_mass_data(ans.y, r.y, count_points);
                            update_mass_data(ans.time, r.time, count_points);
                        }
                        count_points++;
                    }
                }
            }
        }
    }
    return ans;
}

}  // namespace det
}  // namespace ld
