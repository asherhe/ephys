#ifndef EPHYS_MATH_H
#define EPHYS_MATH_H

#include <cmath>
#include <cassert>
#include <ostream>

namespace ephys
{
  // represents angles and torques and whatnot
  // mostly for making clear that a value is an angle
  typedef float Pseudovec;

  class Vec2
  {
  public:
    float x;
    float y;

    // creates a zero vector by default
    Vec2() : x(0), y(0) {}
    Vec2(const float x, const float y) : x(x), y(y) {}
    Vec2(const Vec2 &v) : x(v.x), y(v.y) {}
    Vec2 &operator=(const Vec2 &v);

    Vec2 &set(const float x, const float y);

    inline float norm() const
    {
      return sqrtf(x * x + y * y);
    };
    // the square of the magnitude of this vector
    inline float norm2() const
    {
      return x * x + y * y;
    };

    // normalizes this vector and returns the result
    Vec2 &normalize();

    inline Vec2 operator-() const { return Vec2(-x, -y); }

    inline Vec2 operator+(const Vec2 &v) const { return Vec2(x + v.x, y + v.y); }
    Vec2 &operator+=(const Vec2 &v);

    inline Vec2 operator-(const Vec2 &v) const { return Vec2(x - v.x, y - v.y); }
    Vec2 &operator-=(const Vec2 &v);

    inline Vec2 operator*(float k) const { return Vec2(x * k, y * k); }
    friend inline Vec2 operator*(float k, const Vec2 &v) { return Vec2(v.x * k, v.y * k); }
    Vec2 &operator*=(float k);

    inline Vec2 operator/(float k) const { return Vec2(x / k, y / k); }
    Vec2 &operator/=(float k);

    inline Vec2 abs() const { return Vec2(fabsf(x), fabsf(y)); }

    // dot product
    inline float operator*(const Vec2 &v) const { return x * v.x + y * v.y; }

    // z-component of cross product
    inline Pseudovec cross(const Vec2 &v) const { return x * v.y - y * v.x; }
    inline Vec2 cross(Pseudovec v) const { return Vec2(y, -x) * v; }

    friend std::ostream &operator<<(std::ostream &out, const Vec2 &v)
    {
      return out << "(" << v.x << "," << v.y << ")";
    }
  };

  // 2x2 matrix
  class Mat2
  {
  public:
    float data[4];

    Mat2() : data{0, 0, 0, 0} {}
    Mat2(float d11, float d12,
         float d21, float d22) : data{d11, d12, d21, d22} {}
    Mat2(const Mat2 &m) : data{m.data[1], m.data[2], m.data[3], m.data[4]} {}
    Mat2 &operator=(const Mat2 &m)
    {
      data[0] = m.data[0];
      data[1] = m.data[1];
      data[2] = m.data[2];
      data[3] = m.data[3];
      return *this;
    }

    static inline Mat2 zero() { return Mat2(); }
    static inline Mat2 identity() { return Mat2(1, 0, 0, 1); }

    inline float &operator[](size_t i) { return data[i]; }
    inline float &at(size_t m, size_t n) { return data[2 * m + n]; }
    inline void set(size_t m, size_t n, float val) { at(m, n) = val; }
    inline Vec2 column(size_t col) const { return Vec2(data[col], data[col + 2]); }

    Mat2
    operator+(const Mat2 &m) const;
    Mat2 &operator+=(const Mat2 &m);

    Mat2 operator-(const Mat2 &m) const;
    Mat2 &operator-=(const Mat2 &m);

    Mat2 operator*(float k) const;
    friend Mat2 operator*(float k, const Mat2 &m);
    Mat2 operator/(float k) const;
    Mat2 &operator*=(float k);
    Mat2 &operator/=(float k);

    Mat2 operator*(const Mat2 &m) const;
    Mat2 &operator*=(const Mat2 &m);
    Vec2 operator*(const Vec2 &v) const;

    inline Mat2 abs() const { return Mat2(fabsf(data[0]), fabsf(data[1]), fabsf(data[2]), fabsf(data[3])); }

    inline float determinant() const
    {
      return data[0] * data[3] - data[1] * data[2];
    }

    Mat2 inverse() const;
    Mat2 &invert();

    Mat2 transpose() const;

    friend std::ostream &operator<<(std::ostream &out, const Mat2 &m)
    {
      return out << "[" << m.data[0] << "," << m.data[1] << ";" << m.data[2] << "," << m.data[3] << "]";
    }
  };

  // holds a 3x3 matrix.
  // used to express transformations in homogenous coordinates.
  // in reality the bottom row is always [0 0 1] since the third component is only used for transformations anyway
  class Mat3
  {
  public:
    float data[6];

    Mat3() : data{0, 0, 0, 0, 0, 0} {}
    Mat3(float d11, float d12, float d13,
         float d21, float d22, float d23) : data{d11, d12, d13, d21, d22, d23} {}
    Mat3(const Mat3 &m) : data{m.data[0], m.data[1], m.data[2], m.data[3], m.data[4], m.data[5]} {}

    static inline Mat3 identity() { return Mat3(1, 0, 0, 0, 1, 0); }

    inline float &operator[](size_t i) { return data[i]; }
    // 0-indexed
    inline float &at(size_t m, size_t n) { return data[3 * m + n]; }
    inline void set(size_t m, size_t n, float val) { at(m, n) = val; }

    // get the i-th column as a column vector
    inline Vec2 getColumn(size_t i) const { return Vec2(data[i], data[i + 3]); }

    // obtains a matrix representing the rotational portion of this transformatoin
    inline Mat2 getRotation() const { return Mat2(data[0], data[1], data[3], data[4]); }

    Mat3 operator*(const Mat3 &m) const;
    Mat3 operator*=(const Mat3 &m);
    Vec2 operator*(const Vec2 &v) const;

    inline float determinant() const
    {
      return data[0] * data[4] - data[1] * data[3];
    }

    Mat3 inverse() const;
    Mat3 &invert();

    friend std::ostream &operator<<(std::ostream &out, const Mat3 &m)
    {
      return out << "[" << m.data[0] << "," << m.data[1] << "," << m.data[2] << ";"
                 << m.data[3] << "," << m.data[4] << "," << m.data[5] << "]";
    }
  };
}

#endif // EPHYS_MATH_H
