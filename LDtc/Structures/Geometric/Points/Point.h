#ifndef LDTC_POINT_H
#define LDTC_POINT_H

#include <string>
#include <cmath>

namespace ld {
template <class T = double>
class Point {
   public:
    T x;
    T y;
};
template <class T>
Point<T> operator+(const Point<T>& a, const Point<T>& b) {
    return {a.x + b.x, a.y + b.y};
}

template <class T>
Point<T> operator-(const Point<T>& a, const Point<T>& b) {
    return {a.x - b.x, a.y - b.y};
}

template <class T>
double dist(const Point<T>& a, const Point<T>& b = {0, 0}) {
    auto k1 = a.x - b.x;
    auto k2 = a.y - b.y;
    return sqrt(k1 * k1 + k2 * k2);
}

template <class T>
std::string to_string(const Point<T>& p) {
    return std::string("x: ") + std::to_string(p.x) +
           " y: " + std::to_string(p.y);
}

}  // namespace ld

#endif  // LDTC_POINT_H
