#ifndef _EPHYS_PLINKS_H
#define _EPHYS_PLINKS_H

#include "ephys/particle.h"
#include "ephys/pcontacts.h"

#include <list>

namespace ephys
{
  // links two particles and generates a contact if they stray out of the constraints of the link
  class ParticleLink : public ParticleContactGenerator
  {
  protected:
    Particle *particles[2];

  public:
    inline void setParticle(size_t index, Particle &particle)
    {
      particles[index] = &particle;
    }
    inline Particle &getParticle(size_t index)
    {
      return *(particles[index]);
    }

    virtual std::list<ParticleContact> &generateContacts() const = 0;

  protected:
    // the current length of the link
    float currentLength() const;
  };

  // holds particles within a maximum distance
  class ParticleCable : public ParticleLink
  {
  protected:
    Particle *particles[2];

  public:
    float length;
    float restitution;

    ParticleCable(float length, float restitution) : length(length), restitution(restitution), particles{nullptr, nullptr} {}

    float currentLength() const;

    inline void setParticle(size_t index, Particle &particle)
    {
      particles[index] = &particle;
    }
    inline Particle &getParticle(size_t index)
    {
      return *(particles[index]);
    }

    std::list<ParticleContact> &generateContacts() const;
  };

  // holds particles at an exact ditance
  class ParticleRod : public ParticleLink
  {
  protected:
    Particle *particles[2];

  public:
    float length;

    ParticleRod(float length) : length(length), particles{nullptr, nullptr} {}

    float currentLength() const;

    inline void setParticle(size_t index, Particle &particle)
    {
      particles[index] = &particle;
    }
    inline Particle &getParticle(size_t index)
    {
      return *(particles[index]);
    }

    std::list<ParticleContact> &generateContacts() const;
  };
}

#endif // _EPHYS_PLINKS_H
