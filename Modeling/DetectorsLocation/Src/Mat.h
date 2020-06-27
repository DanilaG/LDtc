#ifndef LDTC_MAT_H
#define LDTC_MAT_H

#include <utility>
#include <vector>

namespace ld {
namespace model {
template <class T>
class Mat {
   public:
    Mat(const std::pair<unsigned int, unsigned int>& size)
        : width_size_(size.first), data_(size.first * size.second) {}

    T& get(unsigned int i, unsigned int j) { return data_[get_ind(i, j)]; }
    T get(unsigned int i, unsigned int j) const { return data_[get_ind(i, j)]; }

    std::pair<unsigned int, unsigned int> size() const;

   protected:
    unsigned int get_ind(unsigned int i, unsigned int j) const;

    const unsigned int width_size_;
    std::vector<T> data_;
};

template <class T>
std::pair<unsigned int, unsigned int> Mat<T>::size() const {
    return {width_size_, data_.size() / width_size_};
}

template <class T>
unsigned int Mat<T>::get_ind(unsigned int i, unsigned int j) const {
    return j * width_size_ + i;
}

}  // namespace model
}  // namespace ld

#endif  // LDTC_MAT_H
