#ifndef _EPHYS_PWORLD_H
#define _EPHYS_PWORLD_H

#include "ephys/particle.h"
#include "ephys/pforcegen.h"
#include "ephys/pcontacts.h"

#include <list>

#define CALCULATE_SOLVER_ITERATIONS 0

namespace ephys
{
  class ParticleWorld
  {
  protected:
    std::list<Particle *> particles;

    ParticleForceRegistry pfReg;

    std::list<ParticleContactGenerator *> pcGenerators;

    ParticleContactSolver pcSolver;
    unsigned iterations;

  public:
    ParticleWorld(unsigned iterations = CALCULATE_SOLVER_ITERATIONS) : pcSolver(iterations), iterations(iterations) {}

    inline ParticleForceRegistry &getPFReg() { return pfReg; }

    void addParticle(Particle &particle);
    void removeParticle(Particle &particle);

    void addPContactGenerator(ParticleContactGenerator &pcg);
    void removePContactGenerator(ParticleContactGenerator &pcg);

    // invokes contact generators and generates a list of contacts
    std::list<ParticleContact> &generateContacts() const;

    // steps the simulation by time step dt
    void step(float dt);
  };
}

#endif // _EPHYS_PWORLD_H