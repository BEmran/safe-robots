#pragma once

#include <utility>
#include <initializer_list>
#include <type_traits>
#include <bits/functexcept.h>
#include <bits/stl_algobase.h>

template <typename Tp, std::size_t SIZE>
struct ArrayTraits {
  using Type = Tp[SIZE];
  using _Is_swappable = std::__is_swappable<Tp>;
  using _Is_nothrow_swappable = std::__is_nothrow_swappable<Tp>;

  static constexpr Tp& ref(const Type& t, std::size_t n) noexcept {
    return const_cast<Tp&>(t[n]);
  }

  static constexpr Tp* ptr(const Type& t) noexcept {
    return const_cast<Tp*>(t);
  }
};

template <typename Tp>
struct ArrayTraits<Tp, 0> {
  struct Type {};
  using _Is_swappable = std::true_type;
  using _Is_nothrow_swappable = std::true_type;

  static constexpr Tp& ref(const Type&, std::size_t) noexcept {
    return *static_cast<Tp*>(nullptr);
  }

  static constexpr Tp* ptr(const Type&) noexcept {
    return nullptr;
  }
};

/**
 *  @brief A standard container for storing a fixed size sequence of elements.
 *
 *  Sets support random access iterators.
 *
 *  @tparam  Tp  Type of element. Required to be a complete type.
 *  @tparam  SIZE  Number of elements.
 */
template <typename Tp, std::size_t SIZE>
struct Array {
  using value_type = Tp;
  using pointer = value_type*;
  using const_pointer = const value_type*;
  using reference = value_type&;
  using const_reference = const value_type&;
  using iterator = value_type*;
  using const_iterator = const value_type*;
  using size_type = std::size_t;
  using difference_type = std::ptrdiff_t;
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

  // Support for zero-sized arrays mandatory.
  using AT_Type = ArrayTraits<Tp, SIZE>;
  typename AT_Type::Type m_data;

  Array() = default;

  Array(const Array& other) {
    std::copy(other.cbegin(), other.cend(), begin());
  }

  Array(Array& other) {
    swap(other);
  }

  Array(Tp init) {
    fill(init);
  }

  void swap(Array& other) noexcept(AT_Type::_Is_nothrow_swappable::value) {
    std::swap_ranges(begin(), end(), other.begin());
  }

  void fill(const value_type& u) {
    std::fill_n(begin(), size(), u);
  }

  // iterator
  constexpr iterator begin() noexcept {
    return iterator(data());
  }

  constexpr const_iterator begin() const noexcept {
    return const_iterator(data());
  }

  constexpr iterator end() noexcept {
    return iterator(data() + SIZE);
  }

  constexpr const_iterator end() const noexcept {
    return const_iterator(data() + SIZE);
  }

  constexpr reverse_iterator rbegin() noexcept {
    return reverse_iterator(end());
  }

  constexpr const_reverse_iterator rbegin() const noexcept {
    return const_reverse_iterator(end());
  }

  constexpr reverse_iterator rend() noexcept {
    return reverse_iterator(begin());
  }

  constexpr const_reverse_iterator rend() const noexcept {
    return const_reverse_iterator(begin());
  }

  constexpr const_iterator cbegin() const noexcept {
    return const_iterator(data());
  }

  constexpr const_iterator cend() const noexcept {
    return const_iterator(data() + SIZE);
  }

  constexpr const_reverse_iterator crbegin() const noexcept {
    return const_reverse_iterator(end());
  }

  constexpr const_reverse_iterator crend() const noexcept {
    return const_reverse_iterator(begin());
  }

  // Capacity
  constexpr size_type size() const noexcept {
    return SIZE;
  }

  constexpr size_type max_size() const noexcept {
    return SIZE;
  }

  [[__nodiscard__]] constexpr bool empty() const noexcept {
    return size() == 0;
  }

  // Element access
  constexpr reference operator[](size_type n) noexcept {
    return AT_Type::ref(m_data, n);
  }

  constexpr const_reference operator[](size_type n) const noexcept {
    return AT_Type::ref(m_data, n);
  }

  constexpr reference at(size_type n) {
    if (n >= SIZE) {
      std::__throw_out_of_range_fmt(__N("array::at: n (which is %zu) "
                                        ">= SIZE (which is %zu)"),
                                    n, SIZE);
    }
    return AT_Type::ref(m_data, n);
  }

  constexpr const_reference at(size_type n) const {
    // Result of conditional expression must be an lvalue so use
    // boolean ? lvalue : (throw-expr, lvalue)
    return n < SIZE ? AT_Type::ref(m_data, n) :
                      (std::__throw_out_of_range_fmt(__N("array::at: n "
                                                         "(which is %zu) "
                                                         ">= SIZE (which is "
                                                         "%zu)"),
                                                     n, SIZE),
                       AT_Type::ref(m_data, 0));
  }

  constexpr reference front() noexcept {
    return *begin();
  }

  constexpr const_reference front() const noexcept {
    return AT_Type::ref(m_data, 0);
  }

  constexpr reference back() noexcept {
    return SIZE ? *(end() - 1) : *end();
  }

  constexpr const_reference back() const noexcept {
    return SIZE ? AT_Type::ref(m_data, SIZE - 1) : AT_Type::ref(m_data, 0);
  }

  constexpr pointer data() noexcept {
    return AT_Type::ptr(m_data);
  }

  constexpr const_pointer data() const noexcept {
    return AT_Type::ptr(m_data);
  }
};

// Array comparisons.
template <typename Tp, std::size_t SIZE>
constexpr inline bool operator==(const Array<Tp, SIZE>& one,
                                 const Array<Tp, SIZE>& two) {
  return std::equal(one.begin(), one.end(), two.begin());
}

template <typename Tp, std::size_t SIZE>
constexpr inline bool operator!=(const Array<Tp, SIZE>& one,
                                 const Array<Tp, SIZE>& two) {
  return !(one == two);
}

// Specialized algorithms.
template <typename Tp, std::size_t SIZE>
void swap(Array<Tp, SIZE>& one,
          Array<Tp, SIZE>& two) noexcept(noexcept(one.swap(two))) {
  one.swap(two);
}

// template <typename Tp, typename... Up>
// Array(Tp, Up...) -> Array<std::enable_if_t<(std::is_same_v<Tp, Up> && ...),
// Tp>, 1 + sizeof...(Up)>;
