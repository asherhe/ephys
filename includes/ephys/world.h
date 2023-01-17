#ifndef _EPHYS_WORLD_H
#define _EPHYS_WORLD_H

#include "ephys/rigidbody.h"
#include "ephys/forcegen.h"

#include <list>

namespace ephys
{
  class World
  {
  protected:
    std::list<RigidBody *> bodies;

    ForceRegistry freg;

  public:
    World() {}

    inline void addBody(RigidBody &body)
    {
      bodies.push_back(&body);
    }
    inline void removeBody(RigidBody &body)
    {
      bodies.remove(&body);
    }

    void step(float dt);
  };
}

#endif // _EPHYS_WORLD_H
