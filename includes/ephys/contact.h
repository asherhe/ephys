#ifndef EPHYS_CONTACT_H
#define EPHYS_CONTACT_H

#include "ephys/math.h"
#include "ephys/rigidbody.h"

namespace ephys
{
  struct Contact
  {
    Rigidbody *bodies[2];

    Vec2 contactPoint,
        normal;

    float penetration, restitution;

    Contact() : bodies{nullptr, nullptr} {}
  };
}

#endif // EPHYS_CONTACT_H
