#ifndef EPHYS_DESKTOP_CAMERA_H
#define EPHYS_DESKTOP_CAMERA_H

#include "ephys/math.h"

#include <SDL3/SDL_rect.h>

// same mapping as Camera in web/js/main.js: y-up world, y-down screen
struct Camera
{
  float x = 0, y = 0;
  float size = 40; // pixels per unit

  // viewport size in pixels, updated every frame
  float width = 0, height = 0;

  inline SDL_FPoint toScreen(const ephys::Vec2 &p) const
  {
    return {(p.x - x) * size + width / 2, (y - p.y) * size + height / 2};
  }
  inline ephys::Vec2 toWorld(const SDL_FPoint &s) const
  {
    return ephys::Vec2((s.x - width / 2) / size + x, y - (s.y - height / 2) / size);
  }
};

#endif // EPHYS_DESKTOP_CAMERA_H
