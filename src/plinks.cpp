#include "ephys/math.h"
#include "ephys/plinks.h"
#include "ephys/particle.h"
#include "ephys/pcontact.h"

#include <list>

using namespace ephys;

float ParticleLink::currentLength() const
{
  return (particles[0]->getPos() - particles[1]->getPos()).norm();
}

float ParticleCable::currentLength() const
{
  return (particles[0]->getPos() - particles[1]->getPos()).norm();
}

std::list<ParticleContact> &ParticleCable::generateContacts() const
{
  std::list<ParticleContact> *contacts = new std::list<ParticleContact>();

  float currLength = currentLength();

  if (length > currLength) // still in range
    return *contacts;

  ParticleContact contact;
  contact.particles[0] = particles[0];
  contact.particles[1] = particles[1];

  // flip normal so instead of pushing the two objects apart they are pulled back together
  Vec2 normal = particles[0]->getPos() - particles[1]->getPos();
  normal.normalize();
  contact.normal = normal;

  contact.penetration = currLength - length;
  contact.restitution = restitution;

  contacts->push_back(contact);
  return *contacts;
}

float ParticleRod::currentLength() const
{
  return (particles[0]->getPos() - particles[1]->getPos()).norm();
}

std::list<ParticleContact> &ParticleRod::generateContacts() const
{
  std::list<ParticleContact> *contacts = new std::list<ParticleContact>;

  float currLength = currentLength();

  if (length == currLength)
    return *contacts;

  ParticleContact contact = *(new ParticleContact());
  contact.particles[0] = particles[0];
  contact.particles[1] = particles[1];

  Vec2 normal = particles[1]->getPos() - particles[0]->getPos();
  normal.normalize();

  if (currLength > length)
  {
    contact.normal = -normal;
    contact.penetration = currLength - length;
  }
  else
  {
    contact.normal = normal;
    contact.penetration = length - currLength;
  }
  contact.restitution = 0; // rod shouldn't be bouncy

  contacts->push_back(contact);
  return *contacts;
}
