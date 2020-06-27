#include "DetectionMap.h"

#include <algorithm>
#include <ctime>
#include <iostream>
#include <random>
#include <thread>

namespace ld {
namespace model {
inline double time2detector(const Point<>& detector, const Point<>& lightning,
                            double c) {
    return dist(detector, lightning) / c;
}

[[nodiscard]] DetectionMap calculate_detection_map(
    FuncDetection fun, const std::pair<unsigned int, unsigned int>& size,
    const std::pair<Point<>, Point<>>& bound,
    const std::vector<Point<>>& detectors, double c, double c_err,
    double time_err) {
    std::mt19937 gen(time(0));
    std::uniform_real_distribution<> uid_c(-c_err, c_err);
    std::uniform_real_distribution<> uid_time(-time_err, time_err);
    Point<> step = {(bound.second.x - bound.first.x) / size.first,
                    (bound.second.y - bound.first.y) / size.second};
    std::vector<TimePoint<>> detectors_trig(detectors.size());
    for (unsigned int i = 0; i < detectors.size(); i++) {
        detectors_trig[i].x = detectors[i].x;
        detectors_trig[i].y = detectors[i].y;
    }
    DetectionMap map(size, bound, detectors);
    for (unsigned int x = 0; x < size.first; x++) {
        for (unsigned int y = 0; y < size.second; y++) {
            Point<> lightning = {(x + 0.5) * step.x + bound.first.x,
                                 (y + 0.5) * step.y + bound.first.y};
            for (auto& i : detectors_trig) {
                double c_offset = uid_c(gen);
                double time_offset = uid_time(gen);
                i.time =
                    time2detector(i, lightning, c + c_offset) + time_offset;
            }
            TimePoint<> ans = fun(detectors_trig, c);
            map.data_[map.get_ind(x, y)] = {dist(ans, lightning),
                                            fabs(ans.time)};
        }
    }
    return map;
}

void blur_count(const DetectionMap& in_map, DetectionMap& out_map,
                unsigned int r, unsigned int y_begin, unsigned int y_end) {
    const auto size = in_map.size();
    const unsigned int truncation = 0.1 * r * r;
    std::pair<std::vector<double>, std::vector<double>> elements;
    elements.first.reserve(4 * r * r + 2 * r);
    elements.second.reserve(4 * r * r + 2 * r);
    for (unsigned int x = 0; x < size.first; x++) {
        for (unsigned int y = y_begin; y < y_end; y++) {
            elements.first.clear();
            elements.second.clear();
            const Point<std::pair<unsigned int, unsigned int>> bound = {
                {(r > x) ? 0 : (x - r),
                 ((x + r >= size.first) ? size.first : (x + r + 1))},
                {(r > y) ? 0 : (y - r),
                 ((y + r >= size.second) ? size.second : (y + r + 1))}};
            std::pair<double, double> sum = {0, 0};
            std::pair<unsigned int, unsigned int> count_nan = {0, 0};
            for (unsigned int r_x = bound.x.first; r_x < bound.x.second;
                 r_x++) {
                for (unsigned int r_y = bound.y.first; r_y < bound.y.second;
                     r_y++) {
                    auto data = in_map.get(r_x, r_y);
                    if (!std::isnan(data.first)) {
                        elements.first.push_back(data.first);
                    } else {
                        count_nan.first++;
                    }
                    if (!std::isnan(data.second)) {
                        elements.second.push_back(data.second);
                    } else {
                        count_nan.second++;
                    }
                }
            }
            if (2 * count_nan.first > elements.first.size()) {
                out_map.get(x, y).first = NAN;
            } else {
                std::partial_sort(elements.first.begin(),
                                  elements.first.begin() + truncation,
                                  elements.first.end(), std::greater<double>());
                std::partial_sort(elements.first.begin() + truncation,
                                  elements.first.begin() + 2 * truncation,
                                  elements.first.end(), std::less<double>());
                out_map.get(x, y).first =
                    std::accumulate(elements.first.begin() + 2 * truncation,
                                    elements.first.end(), 0.0) /
                    (elements.first.size() - 2 * truncation);
            }
            if (2 * count_nan.second > elements.second.size()) {
                out_map.get(x, y).second = NAN;
            } else {
                std::partial_sort(elements.second.begin(),
                                  elements.second.begin() + truncation,
                                  elements.second.end(),
                                  std::greater<double>());
                std::partial_sort(elements.second.begin() + truncation,
                                  elements.second.begin() + 2 * truncation,
                                  elements.second.end(), std::less<double>());
                out_map.get(x, y).second =
                    std::accumulate(elements.second.begin() + 2 * truncation,
                                    elements.second.end(), 0.0) /
                    (elements.second.size() - 2 * truncation);
            }
        }
    }
}

DetectionMap blur(const DetectionMap& in_map, unsigned int r) {
    const auto size = in_map.size();
    DetectionMap out_map(size, in_map.bound, in_map.detectors);
    std::vector<std::thread> threads(std::thread::hardware_concurrency());
    double step = size.second / threads.size();
    for (unsigned int i = 0; i < threads.size(); i++) {
        threads[i] = std::thread(
            blur_count, std::cref(in_map), std::ref(out_map), r,
            static_cast<unsigned int>(step * i),
            std::min(static_cast<unsigned int>(step * (i + 1)), size.second));
    }
    for (unsigned int i = 0; i < threads.size(); i++) {
        threads[i].join();
    }
    return out_map;
}
/* Faster option. Needs refinement. */
// class Blur {
//   public:
//    Blur(const DetectionMap& in_map, unsigned int r, double filter = 0.1);
//
//    DetectionMap count();
//
//   private:
//    void add(const std::pair<double, double>&);
//    void subtract(const std::pair<double, double>& d);
//
//    void right_step();
//    void left_step();
//    void up_step();
//
//    std::pair<double, double> get_data();
//
//    const DetectionMap& in_map_;
//    unsigned int r_;
//    Point<unsigned int> pos_ = {0, 0};
//    std::pair<unsigned int, unsigned int> counter_nan_ = {0, 0};
//    std::pair<unsigned int, unsigned int> counter_numb_ = {0, 0};
//    std::unordered_set<double> max_filtered;
//    std::pair<double, double> numb_sum_ = {0, 0};
//    Point<unsigned int> min_bound_ = {0, 0};
//    Point<unsigned int> max_bound_ = {0, 0};
//};
//
// Blur::Blur(const DetectionMap& in_map, unsigned int r, double filter)
//    : in_map_(in_map), r_(r), max_filtered(static_cast<unsigned int>(r * r *
//    filter), 0) { max_bound_ = {std::min(r_ + 1, in_map_.size().first),
//                  std::min(r_ + 1, in_map_.size().second)};
//    for (unsigned int x = pos_.x; x < max_bound_.x; x++) {
//        for (unsigned int y = pos_.y; y < max_bound_.y; y++) {
//            add(in_map_.get(x, y));
//        }
//    }
//}
//
// DetectionMap Blur::count() {
//    auto size = in_map_.size();
//    DetectionMap out_map(size, in_map_.bound, in_map_.detectors);
//    out_map.get(0, 0) = get_data();
//    for (unsigned int y = 0; y < size.second; y++) {
//        if (!(y % 2)) {
//            for (unsigned int x = 1; x < size.first; x++) {
//                right_step();
//                out_map.get(x, y) = get_data();
//            }
//            if (y != size.second - 1) {
//                up_step();
//                out_map.get(size.first - 1, y + 1) =
//                    get_data();
//            }
//        } else {
//            for (long x = size.first - 2; x >= 0; x--) {
//                left_step();
//                out_map.get(x, y) = get_data();
//            }
//            if (y != size.second - 1) {
//                up_step();
//                out_map.get(0, y + 1) = get_data();
//            }
//        }
//    }
//    return out_map;
//}
//
// void Blur::add(const std::pair<double, double>& d) {
//    if (std::isnan(d.first)) {
//        counter_nan_.first++;
//    } else {
//        if (d.first > max_filtered.front()) {
//            std::push_heap(max_filtered.begin(), max_filtered.end(),
//            std::greater<>());
//        }
//        counter_numb_.first++;
//        numb_sum_.first += d.first;
//    }
//    if (std::isnan(d.second)) {
//        counter_nan_.second++;
//    } else {
//        counter_numb_.second++;
//        numb_sum_.second += d.second;
//    }
//}
//
// void Blur::subtract(const std::pair<double, double>& d) {
//    if (std::isnan(d.first)) {
//        counter_nan_.first--;
//    } else {
//        if (d.first > max_filtered.front()) {
//            std::pop_heap(max_filtered.begin(), max_filtered.end(),
//            std::greater<>());
//        }
//        counter_numb_.first--;
//        numb_sum_.first -= d.first;
//    }
//    if (std::isnan(d.second)) {
//        counter_nan_.second--;
//    } else {
//        counter_numb_.second--;
//        numb_sum_.second -= d.second;
//    }
//}
//
// void Blur::right_step() {
//    pos_.x++;
//    if (max_bound_.x < in_map_.size().first) {
//        for (unsigned int y = min_bound_.y; y < max_bound_.y; y++) {
//            add(in_map_.get(max_bound_.x, y));
//        }
//        max_bound_.x++;
//    }
//    if ((pos_.x - min_bound_.x) > r_) {
//        for (unsigned int y = min_bound_.y; y < max_bound_.y; y++) {
//            subtract(in_map_.get(min_bound_.x, y));
//        }
//        min_bound_.x++;
//    }
//}
//
// void Blur::left_step() {
//    pos_.x--;
//    if (min_bound_.x > 0) {
//        min_bound_.x--;
//        for (unsigned int y = min_bound_.y; y < max_bound_.y; y++) {
//            add(in_map_.get(min_bound_.x, y));
//        }
//    }
//    if ((max_bound_.x - pos_.x) > r_) {
//        max_bound_.x--;
//        for (unsigned int y = min_bound_.y; y < max_bound_.y; y++) {
//            subtract(in_map_.get(max_bound_.x, y));
//        }
//    }
//}
//
// void Blur::up_step() {
//    pos_.y++;
//    if (max_bound_.y < in_map_.size().second) {
//        for (unsigned int x = min_bound_.x; x < max_bound_.x; x++) {
//            add(in_map_.get(x, max_bound_.y));
//        }
//        max_bound_.y++;
//    }
//    if ((pos_.y - min_bound_.y) > r_) {
//        for (unsigned int x = min_bound_.x; x < max_bound_.x; x++) {
//            subtract(in_map_.get(x, min_bound_.y));
//        }
//        min_bound_.y++;
//    }
//}
//
// std::pair<double, double> Blur::get_data() {
//    return {(counter_numb_.first < 2 * counter_nan_.first)
//                ? (NAN)
//                : (numb_sum_.first / counter_numb_.first),
//            (counter_numb_.second < 2 * counter_nan_.second)
//                ? (NAN)
//                : (numb_sum_.second / counter_numb_.second)};
//}
//
// DetectionMap blur(const DetectionMap& in_map, unsigned int r) {
//    return std::move(Blur(in_map, r).count());
//}

}  // namespace model
}  // namespace ld
