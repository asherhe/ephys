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
  // width of the antialiasing fade across every edge, in render pixels
  constexpr float AA_SIZE = 1;
  // width of outlines and vectors, in unscaled pixels
  constexpr float LINE_WIDTH = 1;

  // palette from web/js/main.js
  const SDL_FColor palette[] = {
      {0xf8 / 255.f, 0x58 / 255.f, 0x3e / 255.f, 1},
      {0x5e / 255.f, 0x9e / 255.f, 0xea / 255.f, 1},
      {0xf6 / 255.f, 0xc4 / 255.f, 0x0e / 255.f, 1},
  };
  constexpr size_t PALETTE_SIZE = sizeof(palette) / sizeof(palette[0]);

  const SDL_FColor SELECTED_COLOR = {1, 1, 1, 1};
  const SDL_FColor COLLIDER_COLOR = {0.4f, 1, 0.4f, 1};
  const SDL_FColor VELOCITY_COLOR = {0.8f, 0.8f, 0.8f, 1};
  const SDL_FColor DRAG_COLOR = {1, 1, 1, 1};
  // drag lines shorter than this, in unscaled pixels, fade out towards zero length
  constexpr float DRAG_FADE_LENGTH = 20;

  inline SDL_Vertex vertex(const SDL_FPoint &p, const SDL_FColor &color)
  {
    return {p, color, {0, 0}};
  }

  inline SDL_FColor transparent(const SDL_FColor &color)
  {
    return {color.r, color.g, color.b, 0};
  }

  inline SDL_FPoint offset(const SDL_FPoint &p, const SDL_FPoint &dir, float distance)
  {
    return {p.x + dir.x * distance, p.y + dir.y * distance};
  }

  // two triangles over the quad a, b, c, d
  inline void addQuad(int *indices, int &n, int a, int b, int c, int d)
  {
    const int quad[6] = {a, b, c, a, c, d};
    for (int i : quad)
      indices[n++] = i;
  }

  // unit normal of each edge from points[i] to the next point;
  // for a closed loop they all point outward, whichever way the loop winds
  void edgeNormals(const SDL_FPoint *points, int count, bool closed, SDL_FPoint *out)
  {
    float area = 0;
    if (closed)
      for (int i = 0; i < count; i++)
      {
        const SDL_FPoint &a = points[i], &b = points[(i + 1) % count];
        area += a.x * b.y - b.x * a.y;
      }
    float side = area < 0 ? -1.f : 1.f;

    int edges = closed ? count : count - 1;
    for (int i = 0; i < edges; i++)
    {
      const SDL_FPoint &a = points[i], &b = points[(i + 1) % count];
      float dx = b.x - a.x, dy = b.y - a.y;
      float len = SDL_sqrtf(dx * dx + dy * dy);
      out[i] = len > 0 ? SDL_FPoint{side * dy / len, -side * dx / len} : SDL_FPoint{0, 0};
    }
  }

  // offset direction at a corner between two edges, scaled so both edges move by one unit
  // (the miter); clamped so nearly parallel edges don't spike, as imgui does
  inline SDL_FPoint miter(const SDL_FPoint &a, const SDL_FPoint &b)
  {
    float x = (a.x + b.x) / 2, y = (a.y + b.y) / 2;
    float d2 = x * x + y * y;
    if (d2 > 1e-6f)
    {
      float inv = SDL_min(1 / d2, 100.f);
      x *= inv;
      y *= inv;
    }
    return {x, y};
  }

  // a polyline width render pixels wide, fading out over AA_SIZE on each side
  void strokePolyline(SDL_Renderer *renderer, const SDL_FPoint *points, int count, bool closed, float width, const SDL_FColor &color)
  {
    SDL_FPoint normals[MAX_POINTS];
    edgeNormals(points, count, closed, normals);
    int edges = closed ? count : count - 1;

    // 50% coverage lands at width / 2 from the center; a 1px line is a single fade peak
    float inner = SDL_max(width - AA_SIZE, 0.f) / 2;
    float outer = inner + AA_SIZE;
    SDL_FColor clear = transparent(color);

    // per point: outer fade, solid edge, solid edge, outer fade, across the line
    SDL_Vertex verts[MAX_POINTS * 4];
    for (int i = 0; i < count; i++)
    {
      SDL_FPoint m;
      if (closed)
        m = miter(normals[(i + count - 1) % count], normals[i]);
      else if (i == 0)
        m = normals[0];
      else if (i == count - 1)
        m = normals[edges - 1];
      else
        m = miter(normals[i - 1], normals[i]);
      verts[i * 4] = vertex(offset(points[i], m, outer), clear);
      verts[i * 4 + 1] = vertex(offset(points[i], m, inner), color);
      verts[i * 4 + 2] = vertex(offset(points[i], m, -inner), color);
      verts[i * 4 + 3] = vertex(offset(points[i], m, -outer), clear);
    }

    int indices[MAX_POINTS * 18];
    int n = 0;
    for (int e = 0; e < edges; e++)
    {
      int a = e * 4, b = (e + 1) % count * 4;
      for (int k = 0; k < 3; k++)
        addQuad(indices, n, a + k, a + k + 1, b + k + 1, b + k);
    }

    SDL_RenderGeometry(renderer, nullptr, verts, count * 4, indices, n);
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

  // fills a convex polygon, fading its edge out over AA_SIZE so that 50% coverage lands on the
  // true outline: a solid fan over the outline inset by half the fade, then a fringe around it
  void fillPolygon(SDL_Renderer *renderer, const Camera &camera, const Vec2 *points, int count, const SDL_FColor &color)
  {
    SDL_FPoint screen[MAX_POINTS], normals[MAX_POINTS];
    for (int i = 0; i < count; i++)
      screen[i] = camera.toScreen(points[i]);
    edgeNormals(screen, count, true, normals);

    // per point: solid inset, transparent outset
    SDL_Vertex verts[MAX_POINTS * 2];
    SDL_FColor clear = transparent(color);
    for (int i = 0; i < count; i++)
    {
      SDL_FPoint m = miter(normals[(i + count - 1) % count], normals[i]);
      verts[i * 2] = vertex(offset(screen[i], m, -AA_SIZE / 2), color);
      verts[i * 2 + 1] = vertex(offset(screen[i], m, AA_SIZE / 2), clear);
    }

    int indices[(MAX_POINTS - 2) * 3 + MAX_POINTS * 6];
    int n = 0;
    for (int i = 1; i < count - 1; i++)
    {
      indices[n++] = 0;
      indices[n++] = i * 2;
      indices[n++] = (i + 1) * 2;
    }
    for (int i = 0; i < count; i++)
    {
      int j = (i + 1) % count;
      addQuad(indices, n, i * 2, i * 2 + 1, j * 2 + 1, j * 2);
    }

    SDL_RenderGeometry(renderer, nullptr, verts, count * 2, indices, n);
  }

  // outlines a closed polygon
  void drawLoop(SDL_Renderer *renderer, const Camera &camera, const Vec2 *points, int count, const SDL_FColor &color)
  {
    SDL_FPoint screen[MAX_POINTS];
    for (int i = 0; i < count; i++)
      screen[i] = camera.toScreen(points[i]);
    strokePolyline(renderer, screen, count, true, LINE_WIDTH * camera.scale, color);
  }

  void drawLine(SDL_Renderer *renderer, const Camera &camera, const Vec2 &a, const Vec2 &b, const SDL_FColor &color)
  {
    const SDL_FPoint screen[2] = {camera.toScreen(a), camera.toScreen(b)};
    strokePolyline(renderer, screen, 2, false, LINE_WIDTH * camera.scale, color);
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

  // untextured geometry uses the draw blend mode, and the antialiased edges need blending
  SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

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

  Vec2 from, to;
  if (scene.getDragLine(from, to))
  {
    // fully opaque at DRAG_FADE_LENGTH and longer, fully transparent at zero length
    SDL_FPoint a = camera.toScreen(from), b = camera.toScreen(to);
    float length = SDL_sqrtf((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
    SDL_FColor color = DRAG_COLOR;
    color.a *= SDL_clamp(length / (DRAG_FADE_LENGTH * camera.scale), 0.f, 1.f);
    drawLine(renderer, camera, from, to, color);
  }
}
