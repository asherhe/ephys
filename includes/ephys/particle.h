#ifndef _EPHYS_PARTICLE_H
#define _EPHYS_PARTICLE_H

#include "ephys/math.h"

#include <vector>

namespace ephys
{
  class Particle
  {
  protected:
    Vec2 pos;
    Vec2 vel;
    Vec2 acc;

    // the net force applied to the particle during this tick
    Vec2 forceAccum;

    // the amount of damping applied to linear motion
    // migitates any instability introduced by the integrator
    float damping;

    float mass;
    // 1 / mass; used to simplify physics calculations
    float invMass;

  public:
    // creates a new particle
    Particle(float mass = 1, float damping = 0.9)
    {
      bool isStatic = false;
      setMass(mass);
      setDamping(damping);
    }

    inline Vec2 getPos() const { return pos; }
    inline Vec2 getVel() const { return vel; }
    inline Vec2 getAcc() const { return acc; }
    inline float getMass() const { return mass; }
    inline float getInvMass() const { return invMass; }
    inline float getDamping() const { return damping; }

    void setPos(const Vec2 &pos);
    void setVel(const Vec2 &vel);
    void setAcc(const Vec2 &acc);
    void setMass(float mass);
    void setInvMass(float invMass);
    void setDamping(float damping);

    void addForce(const Vec2 &force);
    void addImpulse(const Vec2 &impulse);

    void clearForceAccum();

    // static particles have infinite mass and cannot be influenced by forces
    // to un-static a particle, set a finite mass
    void setStatic();
    bool isStatic() const;

    void step(float dt);
  };
}

#endif // _EPHYS_PARTICLE_H