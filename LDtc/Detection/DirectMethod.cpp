#include "DirectMethod.h"

#include <iostream>

#include "DetectionMath.h"

namespace ld {
namespace det {
TimePoint<> direct_detect(const TimePoint<>& p0, TimePoint<> p1, TimePoint<> p2,
                          double c) {
    p1.x = p1.x - p0.x;
    p1.y = p1.y - p0.y;
    p2.x = p2.x - p0.x;
    p2.y = p2.y - p0.y;
    double r[3] = {0, dist(p1), dist(p2)};
    if (r[1] == 0) {
        return {NAN, NAN, NAN};
    }
    const double cos_a = p1.y / r[1];
    const double sin_a = p1.x / r[1];
    p1 = {p1.x * cos_a - p1.y * sin_a, p1.x * sin_a + p1.y * cos_a, p1.time};
    p2 = {p2.x * cos_a - p2.y * sin_a, p2.x * sin_a + p2.y * cos_a, p2.time};

    if (p1.y == 0 || p2.x == 0) {
        return {NAN, NAN, NAN};
    }
    const double v = p2.y / p1.y;
    const double y1_sqr = sqr(p1.y);
    const double r2_sqr = sqr(r[2]);
    const double x2_sqr = sqr(p2.x);
    const double t0_sqr = sqr(p0.time);
    const double y2_y1 = p2.y * p1.y;
    const double r2_sqr_d_y1_sqr = r2_sqr / y1_sqr;
    const double c_sqr = sqr(c);

    const double d[3] = {0, p1.time - p0.time, p2.time - p0.time};
    const double q[3] = {0, sqr(p1.time) - t0_sqr, sqr(p2.time) - t0_sqr};

    const double A =
        4 * c_sqr *
        (c_sqr * (d[2] * (d[2] - 2 * v * d[1]) + r2_sqr_d_y1_sqr * sqr(d[1])) -
         x2_sqr);
    const double B =
        4 * c_sqr *
        (d[2] * (c_sqr * (v * q[1] - q[2]) - y2_y1 + r2_sqr) +
         2 * x2_sqr * p0.time +
         d[1] * (v * c_sqr * q[2] + r2_sqr * (1 - c_sqr * q[1] / y1_sqr - v)));
    const double C =
        c_sqr * (2 * (q[2] * (y2_y1 - r2_sqr) - 2 * x2_sqr * t0_sqr +
                      q[1] * (r2_sqr * (v - 1) - v * c_sqr * q[2])) +
                 c_sqr * (r2_sqr_d_y1_sqr * sqr(q[1]) + sqr(q[2]))) +
        r2_sqr * (p1.y * (p1.y - 2 * p2.y) + r2_sqr);

    double root = 0;
    const double min_time = std::min(p0.time, std::min(p1.time, p2.time));
    if (A == 0) [[unlikely]] {
            if (B == 0) [[unlikely]] {
                    return {NAN, NAN, NAN};
                }
            root = -C / B;
        }
    else {
        double dis = sqr(B) - 4 * A * C;
        if (dis < 0) {
            return {NAN, NAN, NAN};
        } else if (dis == 0)
            [[unlikely]] { root = -B / (2 * A); }
        else {
            double root1 = (-B + sqrt(dis)) / (2 * A);
            double root2 = (-B - sqrt(dis)) / (2 * A);
            if (std::signbit(root1 - min_time) !=
                std::signbit(root2 - min_time)) {
                if (std::signbit(root1 - min_time)) {
                    root = root1;
                } else {
                    root = root2;
                }
            } else {
                return {NAN, NAN, NAN};
            }
        }
    }

    const double t1_offset_sqr = sqr(p0.time - root);
    const double y =
        (y1_sqr - c_sqr * (sqr(p1.time - root) - t1_offset_sqr)) / (2 * p1.y);
    Point<> ans = {(r2_sqr - 2 * p2.y * y -
                    c_sqr * (sqr(p2.time - root) - t1_offset_sqr)) /
                       (2 * p2.x),
                   y};
    return {ans.x * cos_a + ans.y * sin_a + p0.x,
            -ans.x * sin_a + ans.y * cos_a + p0.y, root};
}

TimePoint<> direct_detect(const std::vector<TimePoint<>>& data, double c) {
    TimePoint ans = {NAN, NAN, NAN};
    unsigned long count_points = 0;
    for (unsigned int i = 0; i < data.size() - 2; i++) {
        for (unsigned int j = i + 1; j < data.size() - 1; j++) {
            for (unsigned int k = j + 1; k < data.size(); k++) {
                TimePoint<> r = direct_detect(data[i], data[j], data[k], c);
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
    return ans;
}

}  // namespace det
}  // namespace ld
