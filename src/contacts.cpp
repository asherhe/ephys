#include "ephys/contacts.h"
#include "ephys/math.h"

#include <list>

#include <iostream>

using namespace ephys;

void Contact::calculateDerivedData()
{
  if (bodies[0])
    relativePos[0] = contactPoint - bodies[0]->getPos();
  if (bodies[1])
    relativePos[1] = contactPoint - bodies[1]->getPos();

  // we can't cross a pseudovector with a vector so instead we use axb=-bxa
  closingVelocity = (bodies[1]->getVel() - relativePos[0].cross(bodies[1]->getAngVel())) - (bodies[0]->getVel() - relativePos[0].cross(bodies[0]->getAngVel()));

  calculateDesiredDv();
}

void Contact::calculateDesiredDv()
{
  float velFromAcc = bodies[0]->getAcc() * normal -
                     bodies[1]->getAcc() * normal,
        normalClosingVel = closingVelocity * normal;

  desiredDv = -normalClosingVel - restitution * (normalClosingVel - velFromAcc);
}

void Contact::solveVelocity()
{
  Vec2 impulse = normalImpulse() * normal;
  bodies[1]->addImpulse(impulse);
  bodies[1]->addImpulsiveTorque(relativePos[1].cross(impulse));
  impulse *= -1;
  bodies[0]->addImpulse(impulse);
  bodies[0]->addImpulsiveTorque(relativePos[0].cross(impulse));

  calculateDerivedData();
}

void Contact::solvePenetration()
{
  if (penetration <= 0)
    return;

  float sumInvMass = bodies[0]->getInvMass() + bodies[1]->getInvMass();
  if (sumInvMass <= 0)
    return;

  Vec2 displacementPerInvMass = penetration / sumInvMass * normal;

  bodies[0]->displace(-displacementPerInvMass * bodies[0]->getInvMass());
  bodies[1]->displace(displacementPerInvMass * bodies[1]->getInvMass());

  calculateDerivedData();
}

float Contact::normalImpulse()
{
  Pseudovec impulsiveTorquePerImpulse1 = relativePos[0].cross(normal),
            dOmegaPerImpulse1 = bodies[0]->getInvInertia() * impulsiveTorquePerImpulse1;
  Vec2 dvPerImpulse1 = -relativePos[0].cross(dOmegaPerImpulse1); // velocity due to rotation

  Pseudovec impulsiveTorquePerImpulse2 = relativePos[1].cross(normal),
            dOmegaPerImpulse2 = bodies[1]->getInvInertia() * impulsiveTorquePerImpulse2;
  Vec2 dvPerImpulse2 = -relativePos[1].cross(dOmegaPerImpulse2);

  // linear velocity change per impulse is just inverse mass
  // since J=mv
  float dvPerImpulse = dvPerImpulse1 * normal + bodies[0]->getInvMass() +
                       dvPerImpulse2 * normal + bodies[1]->getInvMass();

  float contactVelocity = closingVelocity * normal,
        dv = -(restitution + 1) * contactVelocity;

  std::cout << "normal impulse:\n"
            << "  dv " << dv << "\n"
            << "  per impulse " << dvPerImpulse << "\n"
            << "  impulse " << dv / dvPerImpulse << "\n";

  // impulse in world coords
  return dv / dvPerImpulse;
}

void ContactSolver::solveContacts(std::list<Contact> &contacts, float dt) const
{
  if (contacts.size() == 0)
    return;

  prepareContacts(contacts, dt);
  solvePenetration(contacts, dt);
  solveVelocity(contacts, dt);
}

void ContactSolver::prepareContacts(std::list<Contact> &contacts, float dt) const
{
  for (auto it = contacts.begin(); it != contacts.end(); ++it)
    it->calculateDerivedData();
}

void ContactSolver::solvePenetration(std::list<Contact> &contacts, float dt) const
{
  for (int i = 0; i < maxIterations; ++i)
  {
    auto max = contacts.begin();
    for (auto it = contacts.begin(); it != contacts.end(); ++it)
      if (it->penetration > max->penetration)
        max = it;

    if (max->penetration <= 0)
      return;

    max->calculateDerivedData(); // in case related objects have been modified
    max->solvePenetration();
  }
}

void ContactSolver::solveVelocity(std::list<Contact> &contacts, float dt) const
{
  for (int i = 0; i < maxIterations; ++i)
  {
    auto max = contacts.begin();
    for (auto it = contacts.begin(); it != contacts.end(); ++it)
      if (it->desiredDv > max->desiredDv)
        max = it;

    if (max->desiredDv <= 0)
      return;

    max->calculateDerivedData();
    max->solveVelocity();
  }
}
