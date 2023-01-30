#ifndef EPHYS_FORCEGEN_H
#define EPHYS_FORCEGEN_H

#include "ephys/rigidbody.h"

#include <list>

namespace ephys
{
  class ForceGenerator
  {
  public:
    virtual void updateForce(Rigidbody &body, float dt) = 0;
  };

  class ForceRegistry
  {
  protected:
    struct ForcePair
    {
      Rigidbody *body;
      ForceGenerator *fgen;
    };
    std::list<ForcePair> reg;

  public:
    ForceRegistry() {}

    void add(Rigidbody &body, ForceGenerator &fgen);
    void remove(Rigidbody &body, ForceGenerator &fgen);
    void clear();

    void step(float dt);
  };

  class Gravity : public ForceGenerator
  {
  private:
    Vec2 gravity;

  public:
    Gravity(const Vec2 &gravity) : gravity(gravity) {}

    void updateForce(Rigidbody &body, float dt);

    inline Vec2 getGravity() const
    {
      return gravity;
    }
    inline void setGravity(const Vec2 &gravity)
    {
      this->gravity = gravity;
    }
  };

  class Spring : public ForceGenerator
  {
  protected:
    Rigidbody &end;
    // the connection points, in local coordinates
    Vec2 anchor, endAnchor;

  public:
    float k, length;

    Spring(Rigidbody &end, const Vec2 &anchor, const Vec2 &endAnchor, float k, float length) : end(end), anchor(anchor), endAnchor(endAnchor), k(k), length(length) {}

    inline Rigidbody getEnd() const { return end; }
    inline void setEnd(const Rigidbody &body) { end = body; }

    void updateForce(Rigidbody &body, float dt);
  };
}

#endif // EPHYS_FORCEGEN_H
