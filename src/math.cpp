#include "ephys/math.h"

#include <cmath>
#include <cassert>

using namespace ephys;

Vec2 &Vec2::operator=(const Vec2 &v)
{
  x = v.x;
  y = v.y;
  return *this;
}

Vec2 &Vec2::set(const float x, const float y)
{
  this->x = x;
  this->y = y;
  return *this;
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

Vec2 &Vec2::operator+=(const Vec2 &v)
{
  x += v.x;
  y += v.y;
  return *this;
}

Vec2 &Vec2::operator-=(const Vec2 &v)
{
  x -= v.x;
  y -= v.y;
  return *this;
}

Vec2 &Vec2::operator*=(float k)
{
  x *= k;
  y *= k;
  return *this;
}

Vec2 &Vec2::operator/=(float k)
{
  x /= k;
  y /= k;
  return *this;
}

Mat2 Mat2::operator+(const Mat2 &m) const
{
  return Mat2(data[0] + m.data[0],
              data[1] + m.data[1],
              data[2] + m.data[2],
              data[3] + m.data[3]);
}
Mat2 &Mat2::operator+=(const Mat2 &m)
{
  data[0] += m.data[0];
  data[1] += m.data[1];
  data[2] += m.data[2];
  data[3] += m.data[3];
  return *this;
}

Mat2 Mat2::operator-(const Mat2 &m) const
{
  return Mat2(data[0] - m.data[0],
              data[1] - m.data[1],
              data[2] - m.data[2],
              data[3] - m.data[3]);
}
Mat2 &Mat2::operator-=(const Mat2 &m)
{
  data[0] -= m.data[0];
  data[1] -= m.data[1];
  data[2] -= m.data[2];
  data[3] -= m.data[3];
  return *this;
}

Mat2 Mat2::operator*(float k) const
{
  return Mat2(data[0] * k,
              data[1] * k,
              data[2] * k,
              data[3] * k);
}
Mat2 ephys::operator*(float k, const Mat2 &m)
{
  return Mat2(m.data[0] * k,
              m.data[1] * k,
              m.data[2] * k,
              m.data[3] * k);
}
Mat2 Mat2::operator/(float k) const
{
  return Mat2(data[0] / k,
              data[1] / k,
              data[2] / k,
              data[3] / k);
}

Mat2 &Mat2::operator*=(float k)
{
  data[0] *= k;
  data[1] *= k;
  data[2] *= k;
  data[3] *= k;
  return *this;
}
Mat2 &Mat2::operator/=(float k)
{
  data[0] /= k;
  data[1] /= k;
  data[2] /= k;
  data[3] /= k;
  return *this;
}

Mat2 Mat2::operator*(const Mat2 &m) const
{
  return Mat2(data[0] * m.data[0] + data[1] * m.data[2],
              data[0] * m.data[1] + data[1] * m.data[3],
              data[2] * m.data[0] + data[3] * m.data[2],
              data[2] * m.data[1] + data[3] * m.data[3]);
}
Mat2 &Mat2::operator*=(const Mat2 &m)
{
  float *oldData = data;
  data[0] = oldData[0] * m.data[0] + oldData[1] * m.data[2];
  data[1] = oldData[0] * m.data[1] + oldData[1] * m.data[3];
  data[2] = oldData[2] * m.data[0] + oldData[3] * m.data[2];
  data[3] = oldData[2] * m.data[1] + oldData[3] * m.data[3];
  return *this;
}

Vec2 Mat2::operator*(const Vec2 &v) const
{
  return Vec2(data[0] * v.x + data[1] * v.y,
              data[2] * v.x + data[3] * v.y);
}

Mat2 Mat2::inverse() const
{
  return Mat2(data[3], -data[1],
              -data[2], data[0]) /
         determinant();
}
Mat2 &Mat2::invert()
{
  float *oldData = data;
  float det = determinant();
  data[0] = oldData[3] / det;
  data[1] = -oldData[1] / det;
  data[2] = -oldData[2] / det;
  data[3] = oldData[0] / det;
  return *this;
}

Mat2 Mat2::transpose() const
{
  return Mat2(data[0], data[2],
              data[1], data[3]);
}

Mat3 Mat3::operator*(const Mat3 &m) const
{
  return Mat3(data[0] * m.data[0] + data[1] * m.data[3],
              data[0] * m.data[1] + data[1] * m.data[4],
              data[0] * m.data[2] + data[1] * m.data[5] + data[2],
              data[3] * m.data[0] + data[4] * m.data[3],
              data[3] * m.data[1] + data[4] * m.data[4],
              data[3] * m.data[2] + data[4] * m.data[5] + data[5]);
}
Mat3 Mat3::operator*=(const Mat3 &m)
{
  float *oldData = data;
  data[0] = oldData[0] * m.data[0] + oldData[1] * m.data[3];
  data[1] = oldData[0] * m.data[1] + oldData[1] * m.data[4];
  data[2] = oldData[0] * m.data[2] + oldData[1] * m.data[5];
  data[3] = oldData[3] * m.data[0] + oldData[4] * m.data[3];
  data[4] = oldData[3] * m.data[1] + oldData[4] * m.data[4];
  data[5] = oldData[3] * m.data[2] + oldData[4] * m.data[5];
  return *this;
}
Vec2 Mat3::operator*(const Vec2 &v) const
{
  return Vec2(data[0] * v.x + data[1] * v.y + data[2],
              data[3] * v.x + data[4] * v.y + data[5]);
}

Mat3 Mat3::inverse() const
{
  Mat3 inverse;

  float det = determinant();

  inverse.data[0] = data[4] / det;
  inverse.data[1] = -data[1] / det;
  inverse.data[2] = (data[1] * data[5] - data[2] * data[4]) / det;
  inverse.data[3] = -data[3] / det;
  inverse.data[4] = data[0] / det;
  inverse.data[5] = (data[2] * data[3] - data[0] * data[5]) / det;

  return inverse;
}

Mat3 &Mat3::invert()
{
  float *oldData = data;

  float det = determinant();

  data[0] = oldData[4] / det;
  data[1] = -oldData[1] / det;
  data[2] = (oldData[1] * oldData[5] - oldData[2] * oldData[4]) / det;
  data[3] = oldData[3] / det;
  data[4] = -oldData[0] / det;
  data[5] = (oldData[2] * oldData[3] - oldData[0] * oldData[5]) / det;

  return *this;
}
