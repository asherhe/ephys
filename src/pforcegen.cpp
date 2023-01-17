#include "ephys/math.h"
#include "ephys/pforcegen.h"

#include <list>

using namespace ephys;

void ParticleForceRegistry::add(Particle &particle, ParticleForceGenerator &pfgen)
{
  ParticleForcePair *pair = new ParticleForcePair;
  pair->particle = &particle;
  pair->pfgen = &pfgen;
  reg.push_back(*pair);
}
void ParticleForceRegistry::remove(Particle &particle, ParticleForceGenerator &pfgen)
{
  for (auto it = reg.begin(); it != reg.end(); ++it)
    if (it->particle == &particle && it->pfgen == &pfgen)
    {
      reg.erase(it);
      return;
    }
}
void ParticleForceRegistry::clear()
{
  reg.clear();
}
void ParticleForceRegistry::step(float dt)
{
  for (auto it = reg.begin(); it != reg.end(); ++it)
    it->pfgen->updateForce(*(it->particle), dt);
}

void ParticleGravity::updateForce(Particle &particle, float dt) const
{
  particle.addForce(gravity * particle.getMass());
}

void ParticleDrag::updateForce(Particle &particle, float dt) const
{
  Vec2 force = particle.getVel();

  float drag = force.norm(); // temporarily stores velocity
  force /= drag;             // get direction of force
  drag = k1 * drag + k2 * drag * drag;

  force *= -drag;
  particle.addForce(force);
}

void ParticleSpring::updateForce(Particle &particle, float dt) const
{
  Vec2 displacement = end.getPos() - particle.getPos();
  float restDisplacement = displacement.norm() - length; // already negated; actually equal to -x
  particle.addForce(displacement.normalize() * k * restDisplacement);
}
