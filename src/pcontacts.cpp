#include "ephys/math.h"
#include "ephys/pcontact.h"

#include <algorithm>

using namespace ephys;

bool ParticleContact::operator<(const ParticleContact &other) const
{
  float vSep = separatingVelocity(),
        otherVSep = other.separatingVelocity();

  if (vSep != otherVSep)
    return vSep < otherVSep;
  else
    return penetration < other.penetration; // compare by penetration
}

void ParticleContact::solve(float dt)
{
  solveVelocity(dt);
  solvePenetration(dt);
}

float ParticleContact::separatingVelocity() const
{
  Vec2 vRelative = particles[0]->getVel() - particles[1]->getVel();
  return vRelative * normal;
}

void ParticleContact::solveVelocity(float dt)
{
  float vSep = separatingVelocity();
  if (vSep > 0) // objects are moving apart
    return;

  float newVSep = -vSep;

  // separating velocity as a result of acceleration
  float vAcc = (particles[0]->getAcc() - particles[1]->getAcc()) * normal * dt;
  if (vAcc < 0) // acceleration causes inward velocity
  {
    newVSep += vAcc;
    if (newVSep < 0) // positions still can't be converging
      newVSep = 0;
  }
  newVSep *= restitution;

  float dv = newVSep - vSep;

  float sumInvMass = particles[0]->getInvMass() + particles[1]->getInvMass();

  if (sumInvMass <= 0) // one of the particles has infinite mass
    return;

  Vec2 impulse = normal * dv / sumInvMass;
  particles[0]->addImpulse(impulse);
  particles[1]->addImpulse(-impulse);
}

void ParticleContact::solvePenetration(float dt)
{
  if (penetration <= 0)
    return;

  float sumInvMass = particles[0]->getInvMass() + particles[1]->getInvMass();
  if (sumInvMass <= 0)
    return;

  // (m1 * m2) / (m1 + m2) * d * n
  Vec2 displacementPerInvMass = penetration / sumInvMass * normal;

  particles[0]->setPos(particles[0]->getPos() - particles[0]->getInvMass() * displacementPerInvMass);
  particles[1]->setPos(particles[1]->getPos() + particles[1]->getInvMass() * displacementPerInvMass);
}

void ParticleContactSolver::solveContacts(std::list<ParticleContact> &contacts, float dt) const
{
  for (int i = 0; i < maxIterations; ++i)
  {
    auto max = contacts.begin();
    for (auto it = contacts.begin(); it != contacts.end(); ++it)
      if (*max < *it)
        max = it;

    max->solve(dt);
  }
}
