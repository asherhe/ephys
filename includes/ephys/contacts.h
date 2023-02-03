#ifndef EPHYS_CONTACT_H
#define EPHYS_CONTACT_H

#include "ephys/math.h"
#include "ephys/rigidbody.h"

#include <list>

namespace ephys
{
  class ContactSolver;

  struct Contact
  {
    friend class ContactSolver;

    Rigidbody *bodies[2];

    Vec2 contactPoint,
        normal;

    float penetration, restitution;

    Contact() : bodies{nullptr, nullptr} {}

  protected:
    // the offset of the contact point, in object space
    Vec2 relativePos[2];

    // separating velocity (at the contact point, not the objects)
    // in world cordinates
    Vec2 closingVelocity;

    float desiredDv;

    // recalculates derived data in case any original data is modified
    void calculateDerivedData();

    void calculateDesiredDv();

    void solveVelocity();
    void solvePenetration();

    // determines the impulse generated normal to the contact
    // i.e. no friction involved
    float normalImpulse();
  };

  class ContactSolver
  {
  public:
    ContactSolver(size_t maxIterations) : maxIterations(maxIterations) {}
    inline float getIterations() const { return maxIterations; };
    inline void setIterations(size_t iterations) { maxIterations = iterations; }

    void solveContacts(std::list<Contact> &contacts, float dt) const;

  protected:
    size_t maxIterations;

    void prepareContacts(std::list<Contact> &contacts, float dt) const;
    void solvePenetration(std::list<Contact> &contacts, float dt) const;
    void solveVelocity(std::list<Contact> &contacts, float dt) const;
  };
}

#endif // EPHYS_CONTACT_H
