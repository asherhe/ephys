#include "ephys/world.h"

#include "ephys/contacts.h"
#include "ephys/collision.h"

#include <list>
#include <map>

#include <iostream>

using namespace ephys;

std::list<Contact> &World::generateContacts()
{
  auto contacts = new std::list<Contact>;

  for (auto it = cGenerators.begin(); it != cGenerators.end(); ++it)
    contacts->splice(contacts->begin(), (*it)->generateContacts());

  return *contacts;
}

void World::step(float dt)
{
  std::cout << "========tick========\n";

  freg.step(dt);

  for (auto it = bodies.begin(); it != bodies.end(); ++it)
    (*it)->step(dt);

  std::list<Contact> &contacts = generateContacts();
  if (iterations == 0)
    solver.setIterations(contacts.size() * 2);
  solver.solveContacts(contacts, dt);
  delete &contacts;
}
