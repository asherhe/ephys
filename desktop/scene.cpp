#include "scene.h"

#include <cmath>

using namespace ephys;

// a critically damped spring decays as (1 + wt) e^-wt, which is within 2% of rest at wt = 5.83
constexpr float SETTLE_RATIO = 5.83f;

Scene::Scene()
{
  mouseBody.setStatic(true);
  // the constructor leaves these uninitialized, and the spring's damping reads angVel
  mouseBody.setAngle(0);
  mouseBody.setAngVel(0);
  reset();
}

void Scene::reset()
{
  endDrag();
  shapes.clear();
  world = std::make_unique<World>();

  for (int i = 0; i < 4; i++)
  {
    float y = 2 * (i - 1.5f);

    Rigidbody &b1 = i < 2 ? box(1, 1) : circle(0.5);
    b1.setPos(Vec2(-5, y));
    b1.setVel(Vec2(7.5, 0));
    b1.setAngVel(-0.5);
    Rigidbody &b2 = i % 2 == 0 ? box(1, 1) : circle(0.5);
    b2.setPos(Vec2(5, y));
    b2.setVel(Vec2(-7.5, 0));
    b2.setAngVel(0.5);
  }

  startMin = startMax = shapes.empty() ? Vec2(0, 0) : shapes.front().body->getPos();
  for (const Shape &shape : shapes)
  {
    Vec2 p = shape.body->getPos();
    startMin = Vec2(fminf(startMin.x, p.x), fminf(startMin.y, p.y));
    startMax = Vec2(fmaxf(startMax.x, p.x), fmaxf(startMax.y, p.y));
  }
}

void Scene::step(float dt)
{
  world->step(dt);
  // the spring pushes back on mouseBody, which is never stepped, so clear what it accumulated
  mouseBody.clearAccums();
}

int Scene::startDrag(const Vec2 &point)
{
  endDrag();
  int index = pick(point);
  if (index < 0)
    return -1;

  Rigidbody &body = *shapes[index].body;
  mouseBody.setPos(point);
  mouseBody.setVel(Vec2(0, 0));
  prevMouse = point;
  dragAnchor = body.world2Local(point);
  // zero rest length: the grabbed point is pulled onto the mouse
  dragSpring = std::make_unique<Spring>(mouseBody, dragAnchor, Vec2(0, 0), 0, 0, 0);
  world->addFGen(*dragSpring, body);
  dragged = index;
  tuneDragSpring();
  return index;
}

void Scene::tuneDragSpring()
{
  const Rigidbody &body = *shapes[dragged].body;
  // static bodies don't move, and their infinite mass would make the force NaN
  if (body.getStatic())
  {
    dragSpring->k = dragSpring->damping = 0;
    return;
  }
  // fixing w = sqrt(k / m) fixes the settle time for every mass: k = m w^2, and critical damping
  // c = 2 sqrt(k m) = 2 m w (m is the whole reduced mass, since the mouse anchor is static)
  float w = SETTLE_RATIO / dragSettleTime, m = body.getMass();
  dragSpring->k = m * w * w;
  dragSpring->damping = 2 * m * w;
}

void Scene::moveDrag(const Vec2 &point, float dt)
{
  if (!dragSpring)
    return;
  // the anchor moves with the mouse, so damping only resists motion relative to the mouse
  mouseBody.setVel(dt > 0 ? (point - prevMouse) / dt : Vec2(0, 0));
  // tracked while paused too, so unpausing doesn't see one big jump
  prevMouse = point;
  mouseBody.setPos(point);
  // recomputed every frame, so settle time or mass edits mid-drag take effect
  tuneDragSpring();
}

void Scene::endDrag()
{
  if (dragSpring)
    world->removeFGen(*dragSpring, *shapes[dragged].body);
  dragSpring.reset();
  dragged = -1;
}

bool Scene::getDragLine(Vec2 &from, Vec2 &to) const
{
  if (!dragSpring)
    return false;
  from = shapes[dragged].body->local2World(dragAnchor);
  to = mouseBody.getPos();
  return true;
}

int Scene::pick(const Vec2 &point) const
{
  // shapes are drawn in order, so search backwards to find the topmost one
  for (int i = (int)shapes.size() - 1; i >= 0; i--)
  {
    const Shape &shape = shapes[i];
    Vec2 local = shape.collider->object2Collider(shape.body->world2Local(point));
    bool hit = shape.type == Shape::Type::Circle
                   ? local.norm() <= shape.radius
                   : fabsf(local.x) <= shape.halfSize.x && fabsf(local.y) <= shape.halfSize.y;
    if (hit)
      return i;
  }
  return -1;
}

Rigidbody &Scene::circle(float radius)
{
  Shape shape;
  shape.type = Shape::Type::Circle;
  shape.radius = radius;
  shape.body = std::make_unique<Rigidbody>();
  shape.collider = std::make_unique<CircleCollider>(radius, *shape.body);
  shape.body->setCollider(*shape.collider);
  world->addBody(*shape.body);

  shapes.push_back(std::move(shape));
  return *shapes.back().body;
}

Rigidbody &Scene::box(float width, float height)
{
  Shape shape;
  shape.type = Shape::Type::Box;
  shape.halfSize = Vec2(width / 2, height / 2);
  shape.body = std::make_unique<Rigidbody>();
  shape.collider = std::make_unique<BoxCollider>(shape.halfSize, *shape.body);
  shape.body->setCollider(*shape.collider);
  world->addBody(*shape.body);

  shapes.push_back(std::move(shape));
  return *shapes.back().body;
}
