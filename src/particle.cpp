#include "ephys/math.h"
#include "ephys/particle.h"

#include <cassert>
#include <cmath>

using namespace ephys;

void Particle::setPos(const Vec2 &pos) { this->pos = pos; }
void Particle::setVel(const Vec2 &vel) { this->vel = vel; }
void Particle::setAcc(const Vec2 &acc) { this->acc = acc; }

void Particle::setMass(float mass)
{
  assert(mass > 0);
  this->mass = mass;
  invMass = 1 / mass;
}
void Particle::setInvMass(float invMass)
{
  assert(invMass >= 0);
  this->invMass = invMass;
  mass = 1 / invMass;
}
void Particle::setDamping(float damping)
{
  this->damping = damping;
}

void Particle::addForce(const Vec2 &force)
{
  forceAccum += force;
}

void Particle::addImpulse(const Vec2 &impulse)
{
  vel += impulse * invMass;
}

void Particle::clearForceAccum()
{
  forceAccum = Vec2();
}

void Particle::setStatic() { setInvMass(0); };
bool Particle::isStatic() const { return this->invMass == 0; };

void Particle::step(float dt)
{
  assert(dt > 0);

  Vec2 totalAcc = acc + forceAccum * invMass;

  pos += (vel + 0.5 * totalAcc * dt) * dt;

  vel += totalAcc * dt;
  vel *= powf(damping, dt);

  clearForceAccum();
}
