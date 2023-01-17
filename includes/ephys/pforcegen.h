#ifndef _EPHYS_PFORCEGEN_H
#define _EPHYS_PFORCEGEN_H

#include "particle.h"

#include <list>

namespace ephys
{
  class ParticleForceGenerator
  {
  public:
    virtual void updateForce(Particle &particle, float dt) const = 0;
  };

  // catalogs all particle force generators
  class ParticleForceRegistry
  {
  protected:
    struct ParticleForcePair
    {
      Particle *particle;
      ParticleForceGenerator *pfgen;
    };

    std::list<ParticleForcePair> reg;

  public:
    ParticleForceRegistry() {}

    // register a new particle-force generator pair
    void add(Particle &particle, ParticleForceGenerator &fgen);
    void remove(Particle &particle, ParticleForceGenerator &fgen);
    void clear();

    void step(float dt);
  };

  // generates gravitational force on a particle
  class ParticleGravity : public ParticleForceGenerator
  {
  protected:
    Vec2 gravity;

  public:
    ParticleGravity(const Vec2 &gravity) : gravity(gravity) {}

    void updateForce(Particle &particle, float dt) const;
  };

  // applies a simple drag force to particles, using the model
  // F = -(k1 * v + k2 * v^2)
  // where k1 and k2 are constants specified by the user.
  // for smaller values of v, k1 is the dominant term, but as v becomes larger, k2 grows quadratically
  class ParticleDrag : public ParticleForceGenerator
  {
  public:
    float k1;
    float k2;

    ParticleDrag(float k1, float k2) : k1(k1), k2(k2) {}

    void updateForce(Particle &particle, float dt) const;
  };

  // applies force according to Hooke's law
  // F = -kx
  // where k is the stiffness of the spring and x is the displacement from the spring's rest length
  class ParticleSpring : public ParticleForceGenerator
  {
  protected:
    // the particle at the other end of the spring
    Particle &end;

  public:
    // spring constant (or stiffness)
    float k;
    // rest length
    float length;

    ParticleSpring(Particle &end, float k, float length) : end(end), k(k), length(length) {}

    inline void setEnd(const Particle &particle) { end = particle; }
    inline Particle getEnd() const { return end; }

    void updateForce(Particle &particle, float dt) const;
  };
}

#endif // _EPHYS_PFORCEGEN_H