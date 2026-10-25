#include "render.h"

#include <cmath>

using namespace ephys;

namespace
{
  constexpr float TAU = 6.28318530718f;
  constexpr int CIRCLE_SEGMENTS = 32;
  // most points any outline has; circles have the most
  constexpr int MAX_POINTS = CIRCLE_SEGMENTS;
  // velocity vectors show the distance travelled in this many seconds
  constexpr float VELOCITY_SCALE = 0.25f;

  // palette from demos/js/main.js
  const SDL_FColor palette[] = {
      {0xf8 / 255.f, 0x58 / 255.f, 0x3e / 255.f, 1},
      {0x5e / 255.f, 0x9e / 255.f, 0xea / 255.f, 1},
      {0xf6 / 255.f, 0xc4 / 255.f, 0x0e / 255.f, 1},
  };
  constexpr size_t PALETTE_SIZE = sizeof(palette) / sizeof(palette[0]);

  const SDL_FColor SELECTED_COLOR = {1, 1, 1, 1};
  const SDL_FColor COLLIDER_COLOR = {0.4f, 1, 0.4f, 1};
  const SDL_FColor VELOCITY_COLOR = {0.8f, 0.8f, 0.8f, 1};

  inline SDL_Vertex vertex(const SDL_FPoint &p, const SDL_FColor &color)
  {
    return {p, color, {0, 0}};
  }

  inline void setColor(SDL_Renderer *renderer, const SDL_FColor &color)
  {
    SDL_SetRenderDrawColorFloat(renderer, color.r, color.g, color.b, color.a);
  }

  void circlePoints(const Vec2 &center, float radius, Vec2 out[CIRCLE_SEGMENTS])
  {
    for (int i = 0; i < CIRCLE_SEGMENTS; i++)
    {
      float theta = TAU * i / CIRCLE_SEGMENTS;
      out[i] = Vec2(center.x + radius * cosf(theta), center.y + radius * sinf(theta));
    }
  }

  // corners of a box rotated counterclockwise by angle in world space,
  // equivalent to ctx.rotate(-angle) on the y-down canvas in the web demo
  void boxCorners(const Vec2 &center, float angle, const Vec2 &halfSize, Vec2 out[4])
  {
    float c = cosf(angle), s = sinf(angle);
    const Vec2 corners[4] = {
        Vec2(-halfSize.x, -halfSize.y),
        Vec2(halfSize.x, -halfSize.y),
        Vec2(halfSize.x, halfSize.y),
        Vec2(-halfSize.x, halfSize.y),
    };
    for (int i = 0; i < 4; i++)
      out[i] = Vec2(center.x + c * corners[i].x - s * corners[i].y,
                    center.y + s * corners[i].x + c * corners[i].y);
  }

  // the shape as the web demo draws it; returns the number of points written
  int shapePoints(const Shape &shape, Vec2 out[MAX_POINTS])
  {
    Vec2 pos = shape.body->getPos();
    if (shape.type == Shape::Type::Circle)
    {
      circlePoints(pos, shape.radius, out);
      return CIRCLE_SEGMENTS;
    }
    boxCorners(pos, shape.body->getAngle(), shape.halfSize, out);
    return 4;
  }

  // collider space -> world space, the same way the collision code does it
  inline Vec2 colliderToWorld(const Shape &shape, const Vec2 &p)
  {
    return shape.body->local2World(shape.collider->collider2Object(p));
  }

  // the collider as the engine sees it; returns the number of points written
  int colliderPoints(const Shape &shape, Vec2 out[MAX_POINTS])
  {
    int count;
    if (shape.type == Shape::Type::Circle)
    {
      circlePoints(Vec2(0, 0), shape.radius, out);
      count = CIRCLE_SEGMENTS;
    }
    else
    {
      boxCorners(Vec2(0, 0), 0, shape.halfSize, out);
      count = 4;
    }
    for (int i = 0; i < count; i++)
      out[i] = colliderToWorld(shape, out[i]);
    return count;
  }

  // fills a convex polygon as a triangle fan around its first point
  void fillPolygon(SDL_Renderer *renderer, const Camera &camera, const Vec2 *points, int count, const SDL_FColor &color)
  {
    SDL_Vertex verts[MAX_POINTS];
    int indices[(MAX_POINTS - 2) * 3];
    for (int i = 0; i < count; i++)
      verts[i] = vertex(camera.toScreen(points[i]), color);
    for (int i = 0; i < count - 2; i++)
    {
      indices[i * 3] = 0;
      indices[i * 3 + 1] = i + 1;
      indices[i * 3 + 2] = i + 2;
    }

    SDL_RenderGeometry(renderer, nullptr, verts, count, indices, (count - 2) * 3);
  }

  // outlines a closed polygon
  void drawLoop(SDL_Renderer *renderer, const Camera &camera, const Vec2 *points, int count, const SDL_FColor &color)
  {
    SDL_FPoint screen[MAX_POINTS + 1];
    for (int i = 0; i < count; i++)
      screen[i] = camera.toScreen(points[i]);
    screen[count] = screen[0];

    setColor(renderer, color);
    SDL_RenderLines(renderer, screen, count + 1);
  }

  void drawLine(SDL_Renderer *renderer, const Camera &camera, const Vec2 &a, const Vec2 &b, const SDL_FColor &color)
  {
    SDL_FPoint sa = camera.toScreen(a), sb = camera.toScreen(b);
    setColor(renderer, color);
    SDL_RenderLine(renderer, sa.x, sa.y, sb.x, sb.y);
  }
}

SDL_FColor shapeColor(size_t index)
{
  return palette[index % PALETTE_SIZE];
}

void drawScene(SDL_Renderer *renderer, const Scene &scene, const Camera &camera, const RenderOptions &options)
{
  const std::vector<Shape> &shapes = scene.getShapes();
  Vec2 points[MAX_POINTS];

  for (size_t i = 0; i < shapes.size(); i++)
  {
    int count = shapePoints(shapes[i], points);
    fillPolygon(renderer, camera, points, count, shapeColor(i));
  }

  // overlays go on top of every fill
  for (const Shape &shape : shapes)
  {
    if (options.colliders)
    {
      int count = colliderPoints(shape, points);
      drawLoop(renderer, camera, points, count, COLLIDER_COLOR);
      // line along the collider's +x axis, so its rotation is visible on circles too
      Vec2 axis = shape.type == Shape::Type::Circle ? Vec2(shape.radius, 0) : Vec2(shape.halfSize.x, 0);
      drawLine(renderer, camera, colliderToWorld(shape, Vec2(0, 0)), colliderToWorld(shape, axis), COLLIDER_COLOR);
    }
    if (options.velocities)
    {
      Vec2 pos = shape.body->getPos();
      drawLine(renderer, camera, pos, pos + shape.body->getVel() * VELOCITY_SCALE, VELOCITY_COLOR);
    }
  }

  if (options.selected >= 0 && options.selected < (int)shapes.size())
  {
    int count = shapePoints(shapes[options.selected], points);
    drawLoop(renderer, camera, points, count, SELECTED_COLOR);
  }
}
