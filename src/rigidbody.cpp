#include "ephys/rigidbody.h"
#include "ephys/math.h"

#include <cassert>

using namespace ephys;

void Rigidbody::calcDerivedData()
{
  calcTransform();
}

void Rigidbody::calcTransform()
{
  transform[0] = cosf(angle);
  transform[1] = sinf(angle);

  transform[3] = -sinf(angle);
  transform[4] = cosf(angle);

  transform[2] = pos.x;
  transform[5] = pos.y;

  invTransform = transform.inverse();
}

void Rigidbody::addForce(const Vec2 &force)
{
  forceAccum += force;
}

void Rigidbody::addForceAt(const Vec2 &force, const Vec2 &pos)
{
  addForceAtLocal(force, world2Local(pos));
}

void Rigidbody::addForceAtLocal(const Vec2 &force, const Vec2 &pos)
{
  forceAccum += force;
  torqueAccum += pos.cross(force);
}

void Rigidbody::addTorque(float torque)
{
  torqueAccum += torque;
}

void Rigidbody::clearAccums()
{
  forceAccum.set(0, 0);
  torqueAccum = 0;
}

void Rigidbody::step(float dt)
{
  assert(dt > 0);

  Vec2 totalAcc = acc + forceAccum * invMass;

  pos += (vel + 0.5 * totalAcc * dt) * dt;

  vel += totalAcc * dt;
  vel *= powf(linearDamping, dt);

  float angAcc = torqueAccum * invInertia;
  angle += (angVel + 0.5 * angAcc * dt) * dt;
  angVel += angAcc * dt;
  angVel *= powf(angularDamping, dt);

  // calculate new data since objects have moved
  calcDerivedData();

  clearAccums();
}
