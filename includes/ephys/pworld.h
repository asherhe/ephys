#ifndef EPHYS_PWORLD_H
#define EPHYS_PWORLD_H

#include "ephys/particle.h"
#include "ephys/pforcegen.h"
#include "ephys/pcontact.h"

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

    inline void addParticle(Particle &particle)
    {
      particles.push_back(&particle);
    }
    inline void removeParticle(Particle &particle)
    {
      particles.remove(&particle);
    }

    inline void addPFGen(Particle &particle, ParticleForceGenerator &fgen)
    {
      pfReg.add(particle, fgen);
    }
    inline void removePFGen(Particle &particle, ParticleForceGenerator &fgen)
    {
      pfReg.remove(particle, fgen);
    }

    inline void addPContactGenerator(ParticleContactGenerator &pcg)
    {
      pcGenerators.push_back(&pcg);
    }
    inline void removePContactGenerator(ParticleContactGenerator &pcg)
    {
      pcGenerators.remove(&pcg);
    }

    // invokes contact generators and generates a list of contacts
    std::list<ParticleContact> &generateContacts() const;

    // steps the simulation by time step dt
    void step(float dt);
  };
}

#endif // EPHYS_PWORLD_H
