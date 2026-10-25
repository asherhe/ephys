#include "render.h"

#include <cmath>

using namespace ephys;

namespace
{
  constexpr float TAU = 6.28318530718f;
  constexpr int CIRCLE_SEGMENTS = 32;

  // palette from demos/js/main.js
  const SDL_FColor palette[] = {
      {0xf8 / 255.f, 0x58 / 255.f, 0x3e / 255.f, 1},
      {0x5e / 255.f, 0x9e / 255.f, 0xea / 255.f, 1},
      {0xf6 / 255.f, 0xc4 / 255.f, 0x0e / 255.f, 1},
  };
  constexpr size_t PALETTE_SIZE = sizeof(palette) / sizeof(palette[0]);

  inline SDL_Vertex vertex(const SDL_FPoint &p, const SDL_FColor &color)
  {
    return {p, color, {0, 0}};
  }

  void drawCircle(SDL_Renderer *renderer, const Camera &camera, const Vec2 &center, float radius, const SDL_FColor &color)
  {
    // triangle fan: vertex 0 is the center, 1..N are on the rim
    SDL_Vertex verts[CIRCLE_SEGMENTS + 1];
    int indices[CIRCLE_SEGMENTS * 3];

    SDL_FPoint c = camera.toScreen(center);
    float r = radius * camera.size;
    verts[0] = vertex(c, color);
    for (int i = 0; i < CIRCLE_SEGMENTS; i++)
    {
      float theta = TAU * i / CIRCLE_SEGMENTS;
      verts[i + 1] = vertex({c.x + r * cosf(theta), c.y + r * sinf(theta)}, color);

      indices[i * 3] = 0;
      indices[i * 3 + 1] = i + 1;
      indices[i * 3 + 2] = (i + 1) % CIRCLE_SEGMENTS + 1;
    }

    SDL_RenderGeometry(renderer, nullptr, verts, CIRCLE_SEGMENTS + 1, indices, CIRCLE_SEGMENTS * 3);
  }

  void drawBox(SDL_Renderer *renderer, const Camera &camera, const Vec2 &center, float angle, const Vec2 &halfSize, const SDL_FColor &color)
  {
    // rotate corners counterclockwise in world space; equivalent to ctx.rotate(-angle) on the y-down canvas
    float c = cosf(angle), s = sinf(angle);
    const Vec2 corners[4] = {
        Vec2(-halfSize.x, -halfSize.y),
        Vec2(halfSize.x, -halfSize.y),
        Vec2(halfSize.x, halfSize.y),
        Vec2(-halfSize.x, halfSize.y),
    };

    SDL_Vertex verts[4];
    for (int i = 0; i < 4; i++)
    {
      Vec2 world(center.x + c * corners[i].x - s * corners[i].y,
                 center.y + s * corners[i].x + c * corners[i].y);
      verts[i] = vertex(camera.toScreen(world), color);
    }
    const int indices[6] = {0, 1, 2, 0, 2, 3};

    SDL_RenderGeometry(renderer, nullptr, verts, 4, indices, 6);
  }
}

void drawScene(SDL_Renderer *renderer, const Scene &scene, const Camera &camera)
{
  size_t i = 0;
  for (const Shape &shape : scene.getShapes())
  {
    const SDL_FColor &color = palette[i++ % PALETTE_SIZE];
    Vec2 pos = shape.body->getPos();

    if (shape.type == Shape::Type::Circle)
      drawCircle(renderer, camera, pos, shape.radius, color);
    else
      drawBox(renderer, camera, pos, shape.body->getAngle(), shape.halfSize, color);
  }
}
