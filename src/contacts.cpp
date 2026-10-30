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

  // velocity of each body at the contact point is v + w x r, with its own r;
  // we can't cross a pseudovector with a vector so instead we use axb=-bxa
  closingVelocity = (bodies[1]->getVel() - relativePos[1].cross(bodies[1]->getAngVel())) - (bodies[0]->getVel() - relativePos[0].cross(bodies[0]->getAngVel()));

  calculateDesiredDv();
}

void Contact::calculateDesiredDv()
{
  float normalClosingVel = closingVelocity * normal;

  // velocity that built up from acceleration over the last step doesn't bounce, so resting
  // contacts settle instead of jittering
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

void Contact::solvePenetration(Vec2 moved[2])
{
  moved[0] = moved[1] = Vec2(0, 0);
  if (penetration <= 0)
    return;

  float sumInvMass = bodies[0]->getInvMass() + bodies[1]->getInvMass();
  if (sumInvMass <= 0)
    return;

  Vec2 displacementPerInvMass = penetration / sumInvMass * normal;

  moved[0] = -displacementPerInvMass * bodies[0]->getInvMass();
  moved[1] = displacementPerInvMass * bodies[1]->getInvMass();
  bodies[0]->displace(moved[0]);
  bodies[1]->displace(moved[1]);

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

  // both bodies immovable, or an earlier impulse (from another contact between the same bodies)
  // already has them separating here: a negative impulse would pull them into each other
  if (dvPerImpulse <= 0 || desiredDv <= 0)
    return 0;

  // the same desiredDv the solver picks contacts by
  // impulse in world coords
  return desiredDv / dvPerImpulse;
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
  {
    // in the same sense as closingVelocity: bodies[1] relative to bodies[0], along the normal
    it->velFromAcc = (it->bodies[1]->getAcc() - it->bodies[0]->getAcc()) * it->normal * dt;
    it->calculateDerivedData();
  }
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
    Vec2 moved[2];
    max->solvePenetration(moved);

    // moving the bodies changes the penetration of every contact they're in, this one included;
    // without this the same contact is picked and pushed apart again on every iteration
    for (auto it = contacts.begin(); it != contacts.end(); ++it)
      for (int b = 0; b < 2; ++b)
      {
        // a contact closes as its bodies[0] moves along its normal, and opens as bodies[1] does
        if (it->bodies[0] == max->bodies[b])
          it->penetration += moved[b] * it->normal;
        if (it->bodies[1] == max->bodies[b])
          it->penetration -= moved[b] * it->normal;
      }
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
