#ifndef EPHYS_RIGIDBODY_H
#define EPHYS_RIGIDBODY_H

#include "ephys/math.h"

namespace ephys
{
  class Collider;

  class Rigidbody
  {
  protected:
    Vec2 pos, vel, acc;
    Pseudovec angle, angVel;

    float linearDamping, angularDamping;

    float mass, invMass;
    float inertia, invInertia;

    Vec2 forceAccum;
    Pseudovec torqueAccum;

    // derived from position and rotation
    Mat3 transform, invTransform;

    Collider *collider;

    // restitution of this rigidbody
    float restitution;

    // calculates extra state data
    // should be called every time something changes
    void calcDerivedData();

  private:
    // calculates the transform matrix based on the position and rotation
    void calcTransform();

  public:
    Rigidbody() {}

    inline Vec2 getPos() const { return pos; }
    inline Vec2 getVel() const { return vel; }
    inline Vec2 getAcc() const { return acc; }
    inline Pseudovec getAngle() const { return angle; }
    inline Pseudovec getAngVel() const { return angVel; }
    inline float getMass() const { return mass; }
    inline float getInvMass() const { return invMass; }
    inline float getInertia() const { return inertia; }
    inline float getInvInertia() const { return invInertia; }
    inline Collider *getCollider() const { return collider; }
    inline float getRestitution() const { return restitution; }
    inline Mat3 getTransform() const { return transform; }
    inline Mat3 getInvTransform() const { return invTransform; }
    inline Mat2 getTransformRot() const { return transform.getRotation(); }
    inline Mat2 getInvTransformRot() const { return invTransform.getRotation(); }

    inline void setPos(const Vec2 &pos) { this->pos = pos; }
    inline void setVel(const Vec2 &vel) { this->vel = vel; }
    inline void setAcc(const Vec2 &acc) { this->acc = acc; }
    inline void setAngle(Pseudovec angle) { this->angle = angle; }
    inline void setAngVel(Pseudovec angVel) { this->angVel = angVel; }
    inline void setCollider(Collider &collider) { this->collider = &collider; }
    inline void setRestitution(float restitution) { this->restitution = restitution; }

    inline void setMass(float mass)
    {
      this->mass = mass;
      this->invMass = 1 / mass;
    }
    inline void setInvMass(float invMass)
    {
      this->invMass = invMass;
      this->mass = 1 / invMass;
    }
    inline void setInertia(float inertia)
    {
      this->inertia = inertia;
      this->invInertia = 1 / inertia;
    }
    inline void setInvInertia(float invInertia)
    {
      this->invInertia = invInertia;
      this->inertia = 1 / invInertia;
    }

    inline Vec2 local2World(const Vec2 &pos) const { return transform * pos; }
    inline Vec2 world2Local(const Vec2 &pos) const { return invTransform * pos; }

    inline Vec2 rotLocal2World(const Vec2 &pos) const { return transform.getRotation() * pos; }
    inline Vec2 rotWorld2Local(const Vec2 &pos) const { return invTransform.getRotation() * pos; }

    // applies a force at the center of mass
    void addForce(const Vec2 &force);
    // applies a force at a given point (in world space)
    void addForceAt(const Vec2 &force, const Vec2 &point);
    // applies a force at a given point (in local space)
    void addForceAtLocal(const Vec2 &force, const Vec2 &point);
    // applies a torque to this object
    void addTorque(Pseudovec torque);

    void clearAccums();

    void addImpulse(const Vec2 &impulse);
    void addImpulsiveTorque(Pseudovec torque);

    void displace(const Vec2 &displacement);
    void rotate(Pseudovec v);

    void step(float dt);
  };
}

#endif // EPHYS_RIGIDBODY_H
