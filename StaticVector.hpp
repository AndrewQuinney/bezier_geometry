#ifndef STATICVECTOR_HPP
#define STATICVECTOR_HPP

#include <array>
#include <iterator>
#include <ostream>
#include <utility>

namespace bezier_geometry {
/**
 * \brief A simple expandable list that avoids dynamic allocation.
 *
 * This is designed to operate as a drop-in replacement for std::vector, the
 * difference is that a maximum size must be specified and it uses memory
 * equivalent to a std::array of the same type and size.
 *
 * Since the module makes use of functions of limited degree, reductions,
 * solutions, intervals always have a guaranteed maximum size (eg. a function of
 * degree 3 cannot have more than 3 real roots). This property, coupled with
 * this class, enables the avoidance of dynamic (heap-based) memory allocation
 * for most operations, resulting in greatly improved performance.
 *
 * There are other STL allocators that use a stack buffer, such as Chromium's
 * 'StackAllocator', which would enable this type of behaviour from a
 * std::vector, but that allocator has additional logic to overflow onto the
 * heap if it runs out of stack space. This means that there is always some
 * additional checking compared to this class, which is unnecessary overhead
 * when it is guaranteed to never overflow.
 */
template <class T, std::size_t MAX_SIZE> class StaticVector {
public:
  typedef T *iterator; /**< The type of an iterator for this list. */
  typedef const T
      *const_iterator; /**< The type of a constant iterator for this list. */

  /**
   * \brief Constructs a new, empty list.
   *
   * No instances of the contained type are constructed, though this instance
   * will occupy the stack space of such types in the specified count.
   */
  StaticVector() {}

  /**
   * \brief Initializer list constructor.
   *
   * Attempts to copy every entry in the input list into this list.
   */
  StaticVector(
      const std::initializer_list<T> &input /**< The list to be copied. */
  ) {
    for (const T &current : input) {
      push_back(current);
    }
  }

  /**
   * \brief Copy constructor.
   *
   * Attempts to copy every entry in the input list into this list.
   */
  template <std::size_t INPUT_MAX_SIZE>
  StaticVector(const StaticVector<T, INPUT_MAX_SIZE>
                   &input /**< The list to be copied. */
  ) {
    for (const T &current : input) {
      push_back(current);
    }
  }

  /**
   * \brief Copy constructor.
   *
   * Attempts to copy every entry in the input list into this list.
   */
  StaticVector(
      const StaticVector<T, MAX_SIZE> &input /**< The list to be copied. */
  ) {
    for (const T &current : input) {
      push_back(current);
    }
  }

  /**
   * \brief Move constructor.
   *
   * Attempts to move every entry in the input list into this list then clears
   * the input list.
   */
  template <std::size_t INPUT_MAX_SIZE>
  StaticVector(
      StaticVector<T, INPUT_MAX_SIZE> &&input /**< The list to be moved. */
  ) {
    for (T &current : input) {
      push_back(std::move(current));
    }
    input.clear();
  }

  /**
   * \brief Move constructor.
   *
   * Attempts to move every entry in the input list into this list then clears
   * the input list.
   */
  StaticVector(StaticVector<T, MAX_SIZE> &&input /**< The list to be moved. */
  ) {
    for (T &current : input) {
      push_back(std::move(current));
    }
    input.clear();
  }

  /**
   * \brief Destructor - clears this list.
   */
  ~StaticVector() { clear(); }

  /**
   * \brief Removes and destroys every entry in this list.
   */
  void clear() {
    while (_size > 0) {
      erase(std::prev(end()));
    }
  }

  /**
   * \brief Returns true if this list is empty.
   *
   * @return True if this list is empty.
   */
  bool empty() const { return _size == 0; }

  /**
   * \brief Returns the number of items in this list.
   *
   * @return The number of items in this list.
   */
  const std::size_t &size() const { return _size; }

  /**
   * \brief Gets an iterator to the beginning of this list.
   *
   * Behaviour is undefined if this list is empty.
   *
   * @return An iterator to the beginning of this list.
   */
  iterator begin() noexcept { return reinterpret_cast<iterator>(&data[0]); }

  /**
   * \brief Gets an iterator pointing just past the end of this list.
   *
   * @return An iterator pointing past the end of this list.
   */
  iterator end() noexcept {
    iterator result = begin();
    std::advance(result, _size);
    return result;
  }

  /**
   * \brief Gets an iterator to the beginning of this list.
   *
   * Behaviour is undefined if this list is empty.
   *
   * @return An iterator to the beginning of this list.
   */
  const_iterator begin() const noexcept {
    return reinterpret_cast<const_iterator>(&data[0]);
  }

  /**
   * \brief Gets an iterator pointing just past the end of this list.
   *
   * @return An iterator pointing past the end of this list.
   */
  const_iterator end() const noexcept {
    const_iterator result = begin();
    std::advance(result, _size);
    return result;
  }

  /**
   * \brief Appends a value to the end of this list.
   *
   * The input value is copied.
   */
  void push_back(const T &value /**< The value to copy into this list. */
  ) {
    assert(_size < MAX_SIZE);
    new (&data[_size * sizeof(T)]) T(value);
    _size++;
  }

  /**
   * \brief Appends a value to the end of this list.
   *
   * The input value is moved.
   */
  void push_back(T &&value /**< The value to move into this list. */
  ) {
    assert(_size < MAX_SIZE);
    new (&data[_size * sizeof(T)]) T(std::forward<T>(value));
    _size++;
  }

  /**
   * \brief Gets the first element in this list.
   *
   * Behaviour is undefined if this list is empty.
   *
   * @return The first value in this list.
   */
  T &front() {
    assert(!empty());
    return *begin();
  }

  /**
   * \brief Gets the first element in this list.
   *
   * Behaviour is undefined if this list is empty.
   *
   * @return The first value in this list.
   */
  const T &front() const {
    assert(!empty());
    return *begin();
  }

  /**
   * \brief Gets the last element in this list.
   *
   * Behaviour is undefined if this list is empty.
   *
   * @return The last value in this list.
   */
  T &back() {
    assert(!empty());
    return *std::prev(end());
  }

  /**
   * \brief Gets the last element in this list.
   *
   * Behaviour is undefined if this list is empty.
   *
   * @return The last value in this list.
   */
  const T &back() const {
    assert(!empty());
    return *std::prev(end());
  }

  /**
   * \brief Erase a block of entries from this list.
   *
   * The entries in a given range are destroyed and removed from this list.
   *
   * @return An iterator to the end of the erased block, possibly equivalent to
   * \ref StaticVector::end.
   */
  iterator erase(iterator first, /**< An iterator pointing to the first element
                                    to be removed. */
                 iterator last   /**< An iterator pointing past the last element
                                    to be removed. */
  ) {
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

  /**
   * \brief Erases the element at a given position.
   *
   * This is equivalent to calling erase(pos, std::next(pos)).
   * \see StaticVector::erase
   *
   * @return An iterator pointing at the element past the removed element,
   * possibly past the end of this list.
   */
  iterator erase(iterator pos /**< The position of the element to be erased. */
  ) {
    assert(std::distance(begin(), pos) >= 0);
    assert(std::distance(pos, end()) > 0);
    return erase(pos, std::next(pos));
  }

  /**
   * \brief Erases the element at a given position.
   *
   * This is equivalent to calling erase(pos, std::next(pos)).
   * \see StaticVector::erase
   *
   * @return An iterator pointing at the element past the removed element,
   * possibly past the end of this list.
   */
  iterator
  erase(const_iterator pos /**< The position of the element to be erased. */
  ) {
    iterator i = begin();
    std::advance(
        i, std::distance(
               static_cast<const StaticVector<T, MAX_SIZE> *>(this)->begin(),
               pos));
    return erase(i);
  }

  /**
   * \brief Gets the element at an index.
   *
   * Undefined behaviour if the index is outside the range [0, size).
   *
   * @return The element at the specified index.
   */
  T &operator[](
      const std::size_t &index /**< The index of the element to return. */
  ) {
    return *std::next(begin(), index);
  }

  /**
   * \brief Gets the element at an index.
   *
   * Undefined behaviour if the index is outside the range [0, size).
   *
   * @return The element at the specified index.
   */
  const T &operator[](
      const std::size_t &index /**< The index of the element to return. */
  ) const {
    return *std::next(begin(), index);
  }

  /**
   * \brief Copy-assignment operator.
   *
   * Clears this list and copies every element from the input list to this list.
   *
   * @return This list.
   */
  StaticVector<T, MAX_SIZE> &operator=(
      const StaticVector<T, MAX_SIZE> &input /**< The list to be copied. */
  ) {
    clear();
    for (const T &current : input) {
      push_back(current);
    }
    return *this;
  }

  /**
   * \brief Move-assignment operator.
   *
   * Clears this list, moves every element from the input list to this list, and
   * clears the input list.
   *
   * @return This list.
   */
  StaticVector<T, MAX_SIZE> &
  operator=(StaticVector<T, MAX_SIZE> &&input /**< The list to be moved. */
  ) {
    clear();
    for (T &current : input) {
      push_back(std::move(current));
    }
    input.clear();
    return *this;
  }

  /**
   * \brief Tests this list for equivalence to another list.
   *
   * Two lists are equal if they are the same size and every element at the same
   * index in each list is equivalent.
   *
   * @return True if this and the input list are equal.
   */
  bool operator==(const StaticVector<T, MAX_SIZE>
                      &input /**< The list to be tested for equality. */
  ) const {
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

  /**
   * \brief Tests this list for non-equivalence to another list.
   *
   * This is just the inverse of \ref StaticVector::operator==.
   *
   * @return True if this list is not equal to the input list.
   */
  bool operator!=(const StaticVector<T, MAX_SIZE>
                      &input /**< The list to be tested against. */
  ) {
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