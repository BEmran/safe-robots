#include <iostream>
#include <ostream>
#include <array>
#include <chrono>
#include <functional>

namespace vecvec {

template <size_t N>
struct Vector {
  std::array<float, N> data;
  Vector() {
    if (DEBUG) {
      std::cout << "default Constructor Vector [" << N << "]: " << this
                << std::endl;
    }
  }

  Vector(const float& c) noexcept {
    if (DEBUG) {
      std::cout << "Constructor Vector(" << c << ") [" << N << "]: " << this
                << std::endl;
    }
    data.fill(c);
  }

  Vector(const std::array<float, N>& arr) noexcept {
    std::copy(arr.begin(), arr.end(), data.begin());
    if (DEBUG) {
      std::cout << "Constructor Vector array [" << N << "]: " << this
                << std::endl;
    }
  }

  Vector(Vector&& other) : data{std::move(other.data)} {
    if (DEBUG) {
      std::cout << "Constructor Vector&& [" << N << "]: " << this << std::endl;
    }
  }

  Vector(const Vector& other) : data{other.data} {
    if (DEBUG) {
      std::cout << "Constructor const Vector& [" << N << "]: " << this
                << std::endl;
    }
  }

  Vector operator=(Vector&& other) {
    data = std::move(other.data);
    if (DEBUG) {
      std::cout << "Constructor operator= Vector&& [" << N << "]: " << this
                << std::endl;
    }
  }

  Vector operator=(const Vector& other) {
    std::copy(other.data.begin(), other.data.end(), data.begin());
    if (DEBUG) {
      std::cout << "Constructor operator= const Vector& [" << N << "]: " << this
                << std::endl;
    }
  }

  ~Vector() {
    if (DEBUG) {
      std::cout << "Destructor Vector [" << N << "] " << this << std::endl;
    }
  }

  float operator[](size_t idx) const {
    return data[idx];
  }

  float& operator[](size_t idx) {
    return data[idx];
  }

  Vector<N> operator*(const Vector<N>& other) const {
    Vector<N> result(*this);
    result*=other;
    // Vector<N> result;
    // for (size_t i = 0; i < N; ++i) {
    //   result[i] = data[i] * other[i];
    // }
    if (DEBUG) {
      std::cout << "operator* [" << N << "] " << this << std::endl;
    }
    return result;
  }

  void operator*=(const Vector<N>& other) {
    std::transform(other.data.cbegin(), other.data.cend(), data.cbegin(),
                   data.begin(), std::multiplies<>{});
    if (DEBUG) {
      std::cout << "operator*= [" << N << "] " << this << std::endl;
    }
  }
};
}  // namespace vecvec
template <size_t N>
std::ostream& operator<<(std::ostream& os, const vecvec::Vector<N>& vec) {
  os << "[";
  for (size_t i = 0; i < N; ++i) {
    os << vec.data[i];
    if (i < N - 1) {
      os << ", ";
    }
  }
  os << "]";
  return os;
}