#include "ephys/rigidbody.h"
#include "ephys/math.h"

#include <assert.h>

using namespace ephys;

void RigidBody::calcDerivedData()
{
  calcTransform();
}

void RigidBody::calcTransform()
{
  transform[0] = cosf(angle);
  transform[1] = sinf(angle);

  transform[3] = -sinf(angle);
  transform[4] = cosf(angle);

  transform[2] = pos.x;
  transform[5] = pos.y;

  invTransform = transform.inverse();
}

void RigidBody::addForce(const Vec2 &force)
{
  forceAccum += force;
}

void RigidBody::addForceAt(const Vec2 &force, const Vec2 &pos)
{
  addForceAtLocal(force, world2Local(pos));
}

void RigidBody::addForceAtLocal(const Vec2 &force, const Vec2 &pos)
{
  forceAccum += force;
  torqueAccum += pos.cross(force);
}

void RigidBody::addTorque(float torque)
{
  torqueAccum += torque;
}

void RigidBody::clearAccums()
{
  forceAccum.set(0, 0);
  torqueAccum = 0;
}

void RigidBody::step(float dt)
{
  assert(dt > 0);

  Vec2 totalAcc = acc + forceAccum * invMass;
  vel += totalAcc * dt;
  vel *= powf(linearDamping, dt);
  pos += vel * dt;

  float angAcc = torqueAccum * invInertia;
  angVel += angAcc * dt;
  angVel *= powf(angularDamping, dt);
  angle += angVel * dt;

  // calculate new data since objects have moved
  calcDerivedData();

  clearAccums();
}
