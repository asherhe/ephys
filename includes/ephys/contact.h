#ifndef EPHYS_CONTACT_H
#define EPHYS_CONTACT_H

#include "ephys/math.h"
#include "ephys/rigidbody.h"

namespace ephys
{
  struct Contact
  {
    Rigidbody *body[2];

    Vec2 contactPoint,
        normal;

    float penetration, restitution;
  };
}

#endif // EPHYS_CONTACT_H