#ifndef EPHYS_DESKTOP_RENDER_H
#define EPHYS_DESKTOP_RENDER_H

#include "camera.h"
#include "scene.h"

#include <SDL3/SDL_render.h>

struct RenderOptions
{
  int selected = -1;       // index into Scene::getShapes(), -1 for none
  bool colliders = false;  // outline colliders through the engine's own transforms
  bool velocities = false; // draw velocity vectors
};

// fill color of the shape at an index, cycling through the demo palette
SDL_FColor shapeColor(size_t index);

// draws every shape in the scene, plus any debug overlays enabled in options
void drawScene(SDL_Renderer *renderer, const Scene &scene, const Camera &camera, const RenderOptions &options);

#endif // EPHYS_DESKTOP_RENDER_H
