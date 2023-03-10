#include "ephys/rigidbody.h"
#include "ephys/math.h"

#include <cassert>

using namespace ephys;

size_t Rigidbody::idCount = 0;

void Rigidbody::calcDerivedData()
{
  calcTransform();
}

void Rigidbody::calcTransform()
{
  float cosAngle = cosf(angle), sinAngle = sinf(angle);
  transform[0] = cosAngle;
  transform[1] = sinAngle;
  transform[2] = pos.x;

  transform[3] = -sinAngle;
  transform[4] = cosAngle;
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

void Rigidbody::addTorque(Pseudovec torque)
{
  torqueAccum += torque;
}

void Rigidbody::clearAccums()
{
  forceAccum.set(0, 0);
  torqueAccum = 0;
}

void Rigidbody::addImpulse(const Vec2 &impulse)
{
  vel += invMass * impulse;
}
void Rigidbody::addImpulsiveTorque(Pseudovec torque)
{
  angVel += invInertia * torque;
}

void Rigidbody::displace(const Vec2 &displacement)
{
  pos += displacement;
}

void Rigidbody::rotate(Pseudovec v)
{
  angle += v;
}

void Rigidbody::step(float dt)
{
  assert(dt > 0);

  if (!getStatic())
  {
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
  }

  clearAccums();
}
