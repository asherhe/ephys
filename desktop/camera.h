#ifndef EPHYS_DESKTOP_CAMERA_H
#define EPHYS_DESKTOP_CAMERA_H

#include "ephys/math.h"

#include <SDL3/SDL_rect.h>

// same mapping as Camera in web/js/main.js: y-up world, y-down screen
struct Camera
{
  float x = 0, y = 0;
  float size = 40; // unscaled pixels per unit

  // viewport size in render pixels and render pixels per unscaled pixel
  // (SDL_GetWindowDisplayScale), updated every frame
  float width = 0, height = 0;
  float scale = 1;

  inline SDL_FPoint toScreen(const ephys::Vec2 &p) const
  {
    float ppu = size * scale;
    return {(p.x - x) * ppu + width / 2, (y - p.y) * ppu + height / 2};
  }
  inline ephys::Vec2 toWorld(const SDL_FPoint &s) const
  {
    float ppu = size * scale;
    return ephys::Vec2((s.x - width / 2) / ppu + x, y - (s.y - height / 2) / ppu);
  }
};

#endif // EPHYS_DESKTOP_CAMERA_H
