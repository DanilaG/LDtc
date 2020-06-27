#include "VectorMethod.h"

#include "DetectionMath.h"

namespace ld {
namespace det {
TimePoint<> vector_detect(const TimePoint<>& p0, TimePoint<> p1, TimePoint<> p2,
                          double c) {
    p1 = p1 - p0;
    p2 = p2 - p0;
    const unsigned int number_point = 3;

    const double c_sqr = sqr(c);
    const double v = (p1.x * p2.y - p2.x * p1.y);

    if (v == 0) {
        return {NAN, NAN, NAN};
    }

    const double r[number_point] = {0, dist(p1), dist(p2)};
    const double r_sqr[number_point] = {0, sqr(r[1]), sqr(r[2])};
    const double t_sqr[number_point] = {0, sqr(p1.time), sqr(p2.time)};
    const double q_[number_point] = {0, (r_sqr[1] - c_sqr * t_sqr[1]) / 2,
                                     (r_sqr[2] - c_sqr * t_sqr[2]) / 2};

    const double k = (p1.x * p2.x + p1.y * p2.y);
    const double A = (r_sqr[2] * t_sqr[1] - 2 * k * p1.time * p2.time +
                      r_sqr[1] * t_sqr[2]) *
                         c_sqr -
                     sqr(v);
    const double B =
        2 *
        (-r_sqr[2] * q_[1] * p1.time + k * (q_[1] * p2.time + q_[2] * p1.time) -
         r_sqr[1] * q_[2] * p2.time) *
        c;
    double C =
        r_sqr[2] * sqr(q_[1]) - 2 * k * q_[1] * q_[2] + r_sqr[1] * sqr(q_[2]);

    double root = NAN;
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
            if (std::signbit(root1) != std::signbit(root2)) {
                if (std::signbit(root1)) {
                    root = root2;
                } else {
                    root = root1;
                }
            } else {
                return {NAN, NAN, NAN};
            }
        }
    }

    double q[number_point - 1] = {0};
    q[0] = q_[1] - root * c * p1.time;
    q[1] = q_[2] - root * c * p2.time;

    return TimePoint<>{(p2.y * q[0] - p1.y * q[1]) / v,
                       (p1.x * q[1] - p2.x * q[0]) / v, -root / c} +
           p0;
}

TimePoint<> vector_detect(const std::vector<TimePoint<>>& data, double c) {
    TimePoint ans = {NAN, NAN, NAN};
    unsigned long count_points = 0;
    for (unsigned int i = 0; i < data.size() - 2; i++) {
        for (unsigned int j = i + 1; j < data.size() - 1; j++) {
            for (unsigned int k = j + 1; k < data.size(); k++) {
                TimePoint<> r = vector_detect(data[i], data[j], data[k], c);
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
