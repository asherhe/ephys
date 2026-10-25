#include "scene.h"

using namespace ephys;

void Scene::reset()
{
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
