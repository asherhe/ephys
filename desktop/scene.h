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
  bool wall = false; // one of the static boundary walls
};

// owns the world and every body in it (the JS demo's `bodies` array + `world`)
class Scene
{
  std::unique_ptr<ephys::World> world;
  std::vector<Shape> shapes;
  // tightest box around the shapes' starting positions, set by reset()
  ephys::Vec2 startMin, startMax;

  // inner faces of the boundary walls, set once by setWalls and rebuilt by every reset()
  bool hasWalls = false;
  ephys::Vec2 wallMin, wallMax;
  void addWalls();
  void addWall(const ephys::Vec2 &center, const ephys::Vec2 &size);

  // virtual anchor that follows the mouse while dragging; static and never added to the world
  ephys::Rigidbody mouseBody;
  ephys::Vec2 prevMouse;
  // links the dragged shape to mouseBody, null when nothing is dragged
  std::unique_ptr<ephys::Spring> dragSpring;
  int dragged = -1;
  ephys::Vec2 dragAnchor; // grabbed point, in the dragged body's local space

  // sets the drag spring's stiffness and damping from dragSettleTime and the dragged mass
  void tuneDragSpring();

public:
  // seconds for a dragged body to settle within 2% of the mouse, whatever its mass;
  // the spring's stiffness and (critical) damping follow from it, applied every frame
  float dragSettleTime = 0.5f;

  Scene();

  // rebuilds the demo scene from web/js/main.js
  void reset();

  ephys::Rigidbody &circle(float radius);
  ephys::Rigidbody &box(float width, float height);

  // walls the region between two world-space corners in with static boxes, outside it;
  // they stay put from then on, through resets too
  void setWalls(const ephys::Vec2 &min, const ephys::Vec2 &max);

  void step(float dt);

  // index of the topmost shape whose collider contains a world-space point, or -1
  int pick(const ephys::Vec2 &point) const;

  // starts dragging the shape under a world-space point, returning its index or -1
  int startDrag(const ephys::Vec2 &point);
  // moves the drag anchor to a world-space point; dt is the simulated time this frame
  void moveDrag(const ephys::Vec2 &point, float dt);
  void endDrag();
  // world-space ends of the drag spring, false when nothing is dragged
  bool getDragLine(ephys::Vec2 &from, ephys::Vec2 &to) const;
  inline bool isDragging() const { return dragSpring != nullptr; }
  // the drag spring's current parameters, 0 when nothing is dragged
  inline float getDragStiffness() const { return dragSpring ? dragSpring->k : 0; }
  inline float getDragDamping() const { return dragSpring ? dragSpring->damping : 0; }

  // corners of the box around the shapes' starting positions, for framing the camera
  inline void getStartBounds(ephys::Vec2 &min, ephys::Vec2 &max) const
  {
    min = startMin;
    max = startMax;
  }

  inline std::vector<Shape> &getShapes() { return shapes; }
  inline const std::vector<Shape> &getShapes() const { return shapes; }
};

#endif // EPHYS_DESKTOP_SCENE_H
