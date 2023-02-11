#ifndef EPHYS_COLLIDER_H
#define EPHYS_COLLIDER_H

#include "ephys/math.h"

namespace ephys
{
  class Rigidbody;

  class Collider
  {
    friend class Rigidbody;

  protected:
    Collider(Rigidbody &body, const Mat3 &transform) : body(&body), transform(transform), invTransform(transform.inverse()) {}
    Rigidbody *body;

    // offset of the collider in object space
    Mat3 transform, invTransform;

  public:
    virtual ~Collider(){};

    // gets the origin in object space
    inline Vec2 origin() const { return transform.getColumn(2); }

    inline Rigidbody *getBody() const { return body; }
    inline Mat3 getTransform() const { return transform; }
    inline Mat3 getInvTransform() const { return invTransform; }

    inline void setTransform(const Mat3 &transform)
    {
      this->transform = transform;
      invTransform = transform.inverse();
    }
    inline void setInvTransform(const Mat3 &invTarnsform)
    {
      this->invTransform = invTarnsform;
      transform = invTarnsform.inverse();
    }

    inline Vec2 collider2Object(const Vec2 &v) const { return transform * v; }
    inline Vec2 object2Collider(const Vec2 &v) const { return invTransform * v; }
    inline Vec2 rotCollider2Object(const Vec2 &v) const { return transform.getRotation() * v; }
    inline Vec2 rotObject2Collider(const Vec2 &v) const { return invTransform.getRotation() * v; }
  };

  class CircleCollider : public Collider
  {
  public:
    // center is based on offset
    float radius;

    CircleCollider(float radius, Rigidbody &body, const Mat3 &transform = Mat3::identity()) : radius(abs(radius)), Collider(body, transform) {}
  };

  class BoxCollider : public Collider
  {
  public:
    Vec2 halfSize;

    BoxCollider(const Vec2 &halfSize, Rigidbody &body, const Mat3 &transform = Mat3::identity()) : halfSize(abs(halfSize.x), abs(halfSize.y)), Collider(body, transform) {}
  };
}

#endif // EPHYS_COLLIDER_H