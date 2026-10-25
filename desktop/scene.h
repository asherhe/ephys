#ifndef EPHYS_DESKTOP_SCENE_H
#define EPHYS_DESKTOP_SCENE_H

#include "ephys/world.h"

#include <memory>
#include <vector>

struct Shape
{
  enum class Type
  {
    Circle,
    Box
  } type;
  float radius;           // circles only
  ephys::Vec2 halfSize;   // boxes only
  // heap-allocated so the raw pointers World keeps stay valid as the vector grows
  std::unique_ptr<ephys::Rigidbody> body;
  std::unique_ptr<ephys::Collider> collider;
};

// owns the world and every body in it (the JS demo's `bodies` array + `world`)
class Scene
{
  std::unique_ptr<ephys::World> world;
  std::vector<Shape> shapes;

public:
  Scene() { reset(); }

  // rebuilds the demo scene from demos/js/main.js
  void reset();

  ephys::Rigidbody &circle(float radius);
  ephys::Rigidbody &box(float width, float height);

  inline void step(float dt) { world->step(dt); }

  inline const std::vector<Shape> &getShapes() const { return shapes; }
};

#endif // EPHYS_DESKTOP_SCENE_H
