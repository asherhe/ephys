#ifndef _EPHYS_MATH_H
#define _EPHYS_MATH_H

#include <math.h>
#include <ostream>

namespace ephys
{
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

    void set(const float x, const float y);

    inline float norm() const
    {
      return sqrtf(x * x + y * y);
    };
    // the square of the magnitude of this vector
    inline float norm2() const
    {
      return x * x + y * y;
    };

    // returns a normalized copy of this vector (does not modify this vector)
    Vec2 &normalize();

    Vec2 operator-() const;

    Vec2 operator+(const Vec2 &v) const;
    Vec2 &operator+=(const Vec2 &v);

    Vec2 operator-(const Vec2 &v) const;
    Vec2 &operator-=(const Vec2 &v);

    Vec2 operator*(float k) const;
    friend Vec2 operator*(float k, const Vec2 &v);
    Vec2 &operator*=(float k);

    Vec2 operator/(float k) const;
    Vec2 &operator/=(float k);

    // dot product
    float operator*(const Vec2 &v) const;

    friend std::ostream &operator<<(std::ostream &out, const Vec2 &v) { return out << v.x << ", " << v.y; }
  };
}

#endif // _EPHYS_MATH_H