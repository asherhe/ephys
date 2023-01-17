#ifndef _EPHYS_FORCEGEN_H
#define _EPHYS_FORCEGEN_H

#include "ephys/rigidbody.h"

#include <list>

namespace ephys
{
  class ForceGenerator
  {
  public:
    virtual void updateForce(RigidBody &body, float dt) = 0;
  };

  class ForceRegistry
  {
  protected:
    struct ForcePair
    {
      RigidBody *body;
      ForceGenerator *fgen;
    };
    std::list<ForcePair> reg;

  public:
    ForceRegistry() {}

    void add(RigidBody &body, ForceGenerator &fgen);
    void remove(RigidBody &body, ForceGenerator &fgen);
    void clear();

    void step(float dt);
  };

  class Gravity : ForceGenerator
  {
  private:
    Vec2 gravity;

  public:
    Gravity(const Vec2 &gravity) : gravity(gravity) {}

    void updateForce(RigidBody &body, float dt);

    inline Vec2 getGravity() const
    {
      return gravity;
    }
    inline void setGravity(const Vec2 &gravity)
    {
      this->gravity = gravity;
    }
  };

  class Spring : ForceGenerator
  {
  protected:
    RigidBody &end;
    // the connection points, in local coordinates
    Vec2 anchor, endAnchor;

  public:
    float k, length;

    Spring(RigidBody &end, const Vec2 &anchor, const Vec2 &endAnchor, float k, float length) : end(end), anchor(anchor), endAnchor(endAnchor), k(k), length(length) {}

    inline RigidBody getEnd() const { return end; }
    inline void setEnd(const RigidBody &body) { end = body; }

    void updateForce(RigidBody &body, float dt);
  };
}

#endif // _EPHYS_FORCEGEN_H
