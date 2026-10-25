#ifndef EPHYS_DESKTOP_RENDER_H
#define EPHYS_DESKTOP_RENDER_H

#include "camera.h"
#include "scene.h"

#include <SDL3/SDL_render.h>

// draws every shape in the scene, cycling through the demo palette
void drawScene(SDL_Renderer *renderer, const Scene &scene, const Camera &camera);

#endif // EPHYS_DESKTOP_RENDER_H
