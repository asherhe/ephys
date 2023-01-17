#include "ephys/math.h"
#include "ephys/pworld.h"

#include <list>

using namespace ephys;

std::list<ParticleContact> &ParticleWorld::generateContacts() const
{
  std::list<ParticleContact> *contacts = new std::list<ParticleContact>;

  for (auto it = pcGenerators.begin(); it != pcGenerators.end(); ++it)
    contacts->splice(contacts->end(), (*it)->generateContacts());

  return *contacts;
}

void ParticleWorld::step(float dt)
{
  pfReg.step(dt);

  for (auto it = particles.begin(); it != particles.end(); ++it)
  {
    (*it)->step(dt);
  }

  std::list<ParticleContact> contacts = generateContacts();
  if (iterations == CALCULATE_SOLVER_ITERATIONS)
    pcSolver.setIterations(contacts.size() * 2);
  pcSolver.solveContacts(contacts, dt);
}
