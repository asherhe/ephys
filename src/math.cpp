#include "ephys/math.h"

#include <math.h>
#include <assert.h>

using namespace ephys;

Vec2 &Vec2::operator=(const Vec2 &v)
{
  x = v.x;
  y = v.y;
  return *this;
}

void Vec2::set(const float x, const float y)
{
  this->x = x;
  this->y = y;
}

Vec2 &Vec2::normalize()
{
  float n = norm();
  if (n == 0)
  {
    // some arbitrary values
    x = 0;
    y = 0;
  }
  else
  {
    x /= n;
    y /= n;
  }
  return *this;
}

Vec2 Vec2::operator-() const
{
  return Vec2(-x, -y);
}

Vec2 Vec2::operator+(const Vec2 &v) const
{
  return Vec2(x + v.x, y + v.y);
}
Vec2 &Vec2::operator+=(const Vec2 &v)
{
  x += v.x;
  y += v.y;
  return *this;
}

Vec2 Vec2::operator-(const Vec2 &v) const
{
  return Vec2(x - v.x, y - v.y);
}
Vec2 &Vec2::operator-=(const Vec2 &v)
{
  x -= v.x;
  y -= v.y;
  return *this;
}

Vec2 Vec2::operator*(float k) const
{
  return Vec2(x * k, y * k);
}
// apparently you have to add the namespace in front
// otherwise the linker can't find the operator
// https://stackoverflow.com/a/29067357/11389823
Vec2 ephys::operator*(float k, const Vec2 &v)
{
  return Vec2(v.x * k, v.y * k);
}
Vec2 &Vec2::operator*=(float k)
{
  x *= k;
  y *= k;
  return *this;
}

Vec2 Vec2::operator/(float k) const
{
  return Vec2(x / k, y / k);
}
Vec2 &Vec2::operator/=(float k)
{
  x /= k;
  y /= k;
  return *this;
}

float Vec2::operator*(const Vec2 &v) const
{
  return x * v.x + y * v.y;
}