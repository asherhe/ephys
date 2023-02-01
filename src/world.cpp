#include "ephys/world.h"

#include <list>

using namespace ephys;

void World::step(float dt)
{
  freg.step(dt);

  for (auto it = bodies.begin(); it != bodies.end(); ++it)
  {
    (*it)->step(dt);
  }
}
