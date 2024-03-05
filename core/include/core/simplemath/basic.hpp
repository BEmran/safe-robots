#pragma once
#include <cstddef>

template<typename T>
struct VF2 {
  union {
    struct {
      T x, y;
    };
    T v[2];
  };

  VF2<T>() = default;
  VF2<T>(VF2<T>&&) = default;
  VF2<T>(const VF2<T>&) = default;
  VF2<T>& operator=(VF2<T>&&) = default;
  VF2<T>& operator=(const VF2<T>&) = default;

  constexpr VF2(T _x, T _y) noexcept : x{_x}, y{_y} {
  }

  // explicit VF2(_In_reads_(2) const float* pArray) noexcept;

  T operator()(size_t _idx) const noexcept {
    return v[_idx];
  }

  T& operator()(size_t _idx) noexcept {
    return v[_idx];
  }
};

template<typename T>
struct VF3 {
  union {
    struct {
      T x, y, z;
    };
    T v[3];
  };

  VF3() = default;
  VF3(VF3&&) = default;
  VF3(const VF3&) = default;
  VF3& operator=(VF3&&) = default;
  VF3& operator=(const VF3&) = default;

  constexpr VF3(T _x, T _y, T _z) noexcept : x{_x}, y{_y}, z{_z} {
  }

  // explicit VF3(_In_reads_(3) const float* pArray) noexcept;

  T operator()(size_t _idx) const noexcept {
    return v[_idx];
  }

  T& operator()(size_t _idx) noexcept {
    return v[_idx];
  }
};


template<typename T>
struct MF2X2 {
  union {
    struct {
      T m00, m01;
      T m10, m11;
    };
    VF2<T> row[2];
    T mat[2][2];
    T data[4];
  };

  MF2X2() = default;
  MF2X2(MF2X2&&) = default;
  MF2X2(const MF2X2&) = default;
  MF2X2& operator=(MF2X2&&) = default;
  MF2X2& operator=(const MF2X2&) = default;

  constexpr MF2X2(T _m00, T _m01, T _m10, T _m11) noexcept
    : m00{_m00}
    , m01{_m01}
    , m10{_m10}
    , m11{_m11} {
  }

  constexpr MF2X2(const VF2<T>& _row0, const VF2<T>& _row1) noexcept
    : row{_row0, _row1} {
  }
  // explicit MF2X2(_In_reads_(4) const float* pArray) noexcept;

  T operator()(size_t _row, size_t _col) const noexcept {
    return mat[_row][_col];
  }

  T& operator()(size_t _row, size_t _col) noexcept {
    return mat[_row][_col];
  }

  T operator[](size_t _idx) const noexcept {
    return data[_idx];
  }

  T& operator[](size_t _idx) noexcept {
    return data[_idx];
  }
};


template<typename T>
struct MF3X3 {
  union {
    struct {
      T m00, m01, m02;
      T m10, m11, m12;
      T m20, m21, m22;
    };
    VF3<T> row[3];
    T mat[3][3];
    T data[9];
  };

  MF3X3() = default;
  MF3X3(MF3X3&&) = default;
  MF3X3(const MF3X3&) = default;
  MF3X3& operator=(MF3X3&&) = default;
  MF3X3& operator=(const MF3X3&) = default;

  constexpr MF3X3(T _m00, T _m01, T _m02, T _m10, T _m11,
                  T _m12, T _m20, T _m21, T _m22) noexcept
    : m00{_m00}
    , m01{_m01}
    , m02{_m02}
    , m10{_m10}
    , m11{_m11}
    , m12{_m12}
    , m20{_m20}
    , m21{_m21}
    , m22{_m22} {
  }

  constexpr MF3X3(const VF3<T>& _row0, const VF3<T>& _row1, const VF3<T>& _row2) noexcept
    : row{_row0, _row1, _row2} {
  }
  // explicit MF3X3(_In_reads_(9) const float* pArray) noexcept;

  T operator()(size_t _row, size_t _col) const noexcept {
    return mat[_row][_col];
  }

  T& operator()(size_t _row, size_t _col) noexcept {
    return mat[_row][_col];
  }

  T operator[](size_t _idx) const noexcept {
    return data[_idx];
  }

  T& operator[](size_t _idx) noexcept {
    return data[_idx];
  }
};

// float sum_array(float* array, size_t size) {
//   float sum{0.f};
//   for(size_t i=0; i<size; ++i) {
//     sum += array[i];
//   }
//   return sum;
// }