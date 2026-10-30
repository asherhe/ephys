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

    Contact() : bodies{nullptr, nullptr}, velFromAcc(0) {}

  protected:
    // the offset of the contact point from each body's center, in world space
    Vec2 relativePos[2];

    // velocity of bodies[1] relative to bodies[0] (at the contact point, not the objects)
    // in world cordinates
    Vec2 closingVelocity;

    float desiredDv;

    // part of the closing velocity along the normal that the bodies' accelerations built up
    // over the last step, set by ContactSolver::prepareContacts
    float velFromAcc;

    // recalculates derived data in case any original data is modified
    void calculateDerivedData();

    void calculateDesiredDv();

    void solveVelocity();
    // moves the bodies apart, writing how far each one moved to moved[0] and moved[1]
    void solvePenetration(Vec2 moved[2]);

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

  class ContactGenerator
  {
  public:
    virtual std::list<Contact> &generateContacts() const = 0;
  };
}

#endif // EPHYS_CONTACT_H
