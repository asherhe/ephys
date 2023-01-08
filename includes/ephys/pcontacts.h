#ifndef _EPHYS_PCONTACTS_H
#define _EPHYS_PCONTACTS_H

#include "ephys/math.h"
#include "ephys/particle.h"

#include <list>

namespace ephys
{
  class ParticleContactSolver;

  // represents a contact between two particles
  class ParticleContact
  {
    // allow access to protected methods
    friend class ParticleContactSolver;

  public:
    // the two particles that are in contact
    Particle *particles[2];

    // coefficient of restitution
    // the ratio of the separating velocity before and after the collision (but negated to keep it positive)
    // describes how "bouncy" the collision is
    float restitution;

    // from the first particle to the second
    Vec2 contactNormal;

    // how much the two particles are intersecting
    float penetration;

    // calculate the separating velocity of this contact
    float separatingVelocity() const;

    bool operator<(const ParticleContact &other) const;

  protected:
    void solve(float dt);

  private:
    void solveVelocity(float dt);
    void solvePenetration(float dt);
  };

  class ParticleContactSolver
  {
  protected:
    // max. number of iterations
    unsigned maxIterations;

  public:
    ParticleContactSolver(unsigned maxIterations) : maxIterations(maxIterations) {}
    inline unsigned getIterations() const { return maxIterations; }
    void setIterations(unsigned iterations) { this->maxIterations = iterations; }

    // solve a given list of contacts
    void solveContacts(std::list<ParticleContact> &contacts, float dt) const;
  };

  class ParticleContactGenerator
  {
  public:
    // generates a list of contacts
    virtual std::list<ParticleContact> &generateContacts() const = 0;
  };
}

#endif // _EPHYS_PCONTACTS_H