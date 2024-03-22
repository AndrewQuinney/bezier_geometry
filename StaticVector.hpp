#ifndef STATICVECTOR_HPP
#define STATICVECTOR_HPP

#include <array>
#include <iterator>
#include <ostream>
#include <utility>

namespace bezier_geometry {
template <class T, std::size_t MAX_SIZE> class StaticVector {
public:
  typedef T *iterator;
  typedef const T *const_iterator;

  StaticVector() {}

  StaticVector(const std::initializer_list<T> &input) {
    for (const T &current : input) {
      push_back(current);
    }
  }

  template <std::size_t INPUT_MAX_SIZE>
  StaticVector(const StaticVector<T, INPUT_MAX_SIZE> &input) {
    for (const T &current : input) {
      push_back(current);
    }
  }

  StaticVector(const StaticVector<T, MAX_SIZE> &input) {
    for (const T &current : input) {
      push_back(current);
    }
  }

  template <std::size_t INPUT_MAX_SIZE>
  StaticVector(StaticVector<T, INPUT_MAX_SIZE> &&input) {
    for (T &current : input) {
      push_back(std::move(current));
    }
    input.clear();
  }

  StaticVector(StaticVector<T, MAX_SIZE> &&input) {
    for (T &current : input) {
      push_back(std::move(current));
    }
    input.clear();
  }

  ~StaticVector() { clear(); }

  void clear() {
    while (_size > 0) {
      erase(std::prev(end()));
    }
  }

  bool empty() const { return _size == 0; }

  const std::size_t &size() const { return _size; }

  iterator begin() noexcept { return reinterpret_cast<iterator>(&data[0]); }

  iterator end() noexcept {
    iterator result = begin();
    std::advance(result, _size);
    return result;
  }

  const_iterator begin() const noexcept {
    return reinterpret_cast<const_iterator>(&data[0]);
  }

  const_iterator end() const noexcept {
    const_iterator result = begin();
    std::advance(result, _size);
    return result;
  }

  void push_back(const T &value) {
    assert(_size < MAX_SIZE);
    new (&data[_size * sizeof(T)]) T(value);
    _size++;
  }

  void push_back(T &&value) {
    assert(_size < MAX_SIZE);
    new (&data[_size * sizeof(T)]) T(std::forward<T>(value));
    _size++;
  }

  T &front() {
    assert(!empty());
    return *begin();
  }

  const T &front() const {
    assert(!empty());
    return *begin();
  }

  T &back() {
    assert(!empty());
    return *std::prev(end());
  }

  const T &back() const {
    assert(!empty());
    return *std::prev(end());
  }

  iterator erase(iterator first, iterator last) {
    assert(std::distance(first, last) >= 0);
    assert(std::distance(begin(), first) >= 0);
    assert(std::distance(last, end()) >= 0);
    const auto distance = std::distance(first, last);
    if (distance <= 0) {
      return last;
    }
    iterator i = first;
    for (; std::distance(i, end()) > distance; i++) {
      *i = std::move(*(i + distance));
    }
    for (; std::distance(i, end()) > 0; i++) {
      i->~T();
    }
    _size -= distance;
    return first;
  }

  iterator erase(iterator pos) {
    assert(std::distance(begin(), pos) >= 0);
    assert(std::distance(pos, end()) > 0);
    return erase(pos, std::next(pos));
  }

  iterator erase(const_iterator pos) {
    iterator i = begin();
    std::advance(
        i, std::distance(
               static_cast<const StaticVector<T, MAX_SIZE> *>(this)->begin(),
               pos));
    return erase(i);
  }

  T &operator[](const std::size_t &index) { return *std::next(begin(), index); }

  const T &operator[](const std::size_t &index) const {
    return *std::next(begin(), index);
  }

  StaticVector<T, MAX_SIZE> &operator=(const StaticVector<T, MAX_SIZE> &input) {
    clear();
    for (const T &current : input) {
      push_back(current);
    }
    return *this;
  }

  StaticVector<T, MAX_SIZE> &operator=(StaticVector<T, MAX_SIZE> &&input) {
    clear();
    for (T &current : input) {
      push_back(std::move(current));
    }
    input.clear();
    return *this;
  }

  bool operator==(const StaticVector<T, MAX_SIZE> &input) const {
    if (size() != input.size()) {
      return false;
    }
    const_iterator myItr = begin();
    const_iterator inputItr = input.begin();
    while (myItr != end()) {
      if (!(*myItr == *inputItr)) {
        return false;
      }
      myItr = std::next(myItr);
      inputItr = std::next(inputItr);
    }
    return true;
  }

  bool operator!=(const StaticVector<T, MAX_SIZE> &input) {
    return !(*this == input);
  }

private:
  std::size_t _size = 0;
  char data[MAX_SIZE * sizeof(T)];
};

template <class T, class U>
std::ostream &operator<<(std::ostream &os, const std::pair<T, U> &input) {
  os << "(" << input.first << "," << input.second << ")";
  return os;
}

template <class T, std::size_t MAX_SIZE>
std::ostream &operator<<(std::ostream &os,
                         const StaticVector<T, MAX_SIZE> &input) {
  os << "[";
  for (const T &current : input) {
    if (&current != &(*input.begin())) {
      os << ",";
    }
    os << current;
  }
  os << "]";
  return os;
}
} // namespace bezier_geometry
#endif