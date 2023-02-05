#ifndef EPHYS_WORLD_H
#define EPHYS_WORLD_H

#include "ephys/rigidbody.h"
#include "ephys/forcegen.h"
#include "ephys/contacts.h"
#include "ephys/collision.h"

#include <list>
#include <map>

namespace ephys
{
  class World
  {
  protected:
    std::map<Rigidbody *, Collider *> bodies;

    ForceRegistry freg;

    std::list<ContactGenerator *> cGenerators;
    ContactSolver solver;

    size_t iterations;

  public:
    World(size_t solverIterations = 0) : iterations(solverIterations), solver(solverIterations)
    {
      cGenerators.push_back(new CollisionContactGenerator(bodies));
    }

    inline size_t getSolverIterations() { return iterations; }
    inline void setSolverIterations()
    {
      iterations = 0;
      solver.setIterations(iterations);
    }

    inline void addBody(Rigidbody &body, Collider &collider) { bodies[&body] = &collider; }
    inline void removeBody(Rigidbody &body, Collider &collider) { bodies.erase(&body); }

    inline void addFGen(ForceGenerator &fgen, Rigidbody &body) { freg.add(fgen, body); }
    inline void removeFGen(ForceGenerator &fgen, Rigidbody &body) { freg.remove(fgen, body); }

    inline void addContactGen(ContactGenerator &cgen) { cGenerators.push_back(&cgen); }
    inline void removeContactGen(ContactGenerator &cgen) { cGenerators.remove(&cgen); }

    void step(float dt);

  protected:
    std::list<Contact> &generateContacts();
  };
}

#endif // EPHYS_WORLD_H
