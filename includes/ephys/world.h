#ifndef EPHYS_WORLD_H
#define EPHYS_WORLD_H

#include "ephys/rigidbody.h"
#include "ephys/forcegen.h"

#include <list>

namespace ephys
{
  class World
  {
  protected:
    std::list<Rigidbody *> bodies;

    ForceRegistry freg;

  public:
    World() {}

    inline void addBody(Rigidbody &body)
    {
      bodies.push_back(&body);
    }
    inline void removeBody(Rigidbody &body)
    {
      bodies.remove(&body);
    }

    void step(float dt);
  };
}

#endif // EPHYS_WORLD_H
