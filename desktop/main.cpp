// SDL owns the main loop (required on the web, where the browser drives frames)
#define SDL_MAIN_USE_CALLBACKS
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_sdlrenderer3.h"

#include "camera.h"
#include "render.h"
#include "scene.h"

#include <cmath>

using namespace ephys;

// longest frame the sim will advance by, so stopping at a breakpoint doesn't blow up the next step
constexpr float MAX_FRAME_TIME = 0.05f;
// the sim always advances in steps of this size, whatever the frame rate; also what a single
// step advances it by while paused
constexpr float PHYSICS_STEP = 1 / 120.f;
// most frames rendered per second; fewer when frames run long
constexpr int MAX_FPS = 120;

constexpr float DEFAULT_ZOOM = 40, MIN_ZOOM = 4, MAX_ZOOM = 400;
constexpr float WHEEL_ZOOM_FACTOR = 1.1f;

struct App
{
  Scene scene;
  Camera camera;
  RenderOptions render;

  bool paused = false;
  bool stepOnce = false; // advance by PHYSICS_STEP on the next frame while paused
  float pendingTime = 0; // simulated time not yet stepped, carried over to the next frame
  float timeScale = 1;
  float stepMs = 0; // smoothed time spent in World::step per frame
  bool panning = false;
  SDL_FPoint mouse = {0, 0}; // last cursor position, in render pixels

  void reset()
  {
    scene.reset();
    render.selected = -1;
  }

  void resetCamera()
  {
    camera.x = camera.y = 0;
    camera.size = DEFAULT_ZOOM;
  }
};

// keys handleEvent acts on, without ctrl/alt/gui so browser and OS shortcuts like ctrl+r keep working
static bool isHotkey(const SDL_KeyboardEvent &key)
{
  if (key.mod & (SDL_KMOD_CTRL | SDL_KMOD_ALT | SDL_KMOD_GUI))
    return false;
  return key.key == SDLK_SPACE || key.key == SDLK_RIGHT || key.key == SDLK_R;
}

#ifdef SDL_PLATFORM_EMSCRIPTEN
// keys imgui's text fields act on as keys, with or without ctrl/shift (word jumps, select all,
// undo); typed characters don't need their key events, they arrive as SDL_EVENT_TEXT_INPUT
static bool isTextEditKey(SDL_Keycode key)
{
  switch (key)
  {
  case SDLK_LEFT: case SDLK_RIGHT: case SDLK_UP: case SDLK_DOWN:
  case SDLK_HOME: case SDLK_END: case SDLK_BACKSPACE: case SDLK_DELETE:
  case SDLK_RETURN: case SDLK_KP_ENTER: case SDLK_ESCAPE: case SDLK_TAB:
  case SDLK_A: case SDLK_C: case SDLK_V: case SDLK_X: case SDLK_Y: case SDLK_Z:
    return true;
  default:
    return false;
  }
}

// on the web SDL calls preventDefault on every key event it posts, which swallows browser
// shortcuts (ctrl+r, F5, F12...); so only post keys the app listens for, and leave every
// other key to the browser by dropping it here
static bool SDLCALL filterKeys(void *, SDL_Event *event)
{
  if (event->type != SDL_EVENT_KEY_DOWN)
    return true; // key ups don't trigger shortcuts, and imgui needs them to release held keys
  if (isHotkey(event->key))
    return true;
  return ImGui::GetCurrentContext() && ImGui::GetIO().WantTextInput && isTextEditKey(event->key.key);
}
#endif

// event coordinates must already be in render coordinates
static void handleEvent(App &app, const SDL_Event &event)
{
  const ImGuiIO &io = ImGui::GetIO();
  switch (event.type)
  {
  case SDL_EVENT_KEY_DOWN:
    if (io.WantCaptureKeyboard || !isHotkey(event.key))
      break;
    if (event.key.key == SDLK_SPACE && !event.key.repeat)
      app.paused = !app.paused;
    else if (event.key.key == SDLK_RIGHT && app.paused)
      app.stepOnce = true;
    else if (event.key.key == SDLK_R && !event.key.repeat)
      app.reset();
    break;

  case SDL_EVENT_MOUSE_BUTTON_DOWN:
    app.mouse = {event.button.x, event.button.y};
    if (io.WantCaptureMouse)
      break;
    // left click selects and drags (or deselects on empty space), right/middle drag pans
    if (event.button.button == SDL_BUTTON_LEFT)
      app.render.selected = app.scene.startDrag(app.camera.toWorld(app.mouse));
    else
      app.panning = true;
    break;

  case SDL_EVENT_MOUSE_BUTTON_UP:
    app.mouse = {event.button.x, event.button.y};
    // not gated on imgui, so releasing over a window still ends the drag
    if (event.button.button == SDL_BUTTON_LEFT)
      app.scene.endDrag();
    else
      app.panning = false;
    break;

  case SDL_EVENT_MOUSE_MOTION:
    app.mouse = {event.motion.x, event.motion.y};
    if (app.panning)
    {
      // screen is y-down, world is y-up; motion is in render pixels
      app.camera.x -= event.motion.xrel / (app.camera.size * app.camera.scale);
      app.camera.y += event.motion.yrel / (app.camera.size * app.camera.scale);
    }
    break;

  case SDL_EVENT_MOUSE_WHEEL:
  {
    if (io.WantCaptureMouse)
      break;
    // zoom around the cursor: keep the world point under it fixed
    SDL_FPoint mouse = {event.wheel.mouse_x, event.wheel.mouse_y};
    Vec2 before = app.camera.toWorld(mouse);
    app.camera.size = SDL_clamp(app.camera.size * powf(WHEEL_ZOOM_FACTOR, event.wheel.y), MIN_ZOOM, MAX_ZOOM);
    Vec2 after = app.camera.toWorld(mouse);
    app.camera.x += before.x - after.x;
    app.camera.y += before.y - after.y;
    break;
  }
  }
}

static bool dragVec2(const char *label, Vec2 &v, float speed)
{
  float data[2] = {v.x, v.y};
  if (!ImGui::DragFloat2(label, data, speed))
    return false;
  v = Vec2(data[0], data[1]);
  return true;
}

// edits a body through the engine's own getters/setters
static void drawInspector(Shape &shape)
{
  Rigidbody &body = *shape.body;
  if (shape.type == Shape::Type::Circle)
    ImGui::Text("circle %d, radius %.2f", (int)body.id, shape.radius);
  else
    ImGui::Text("box %d, %.2f x %.2f", (int)body.id, shape.halfSize.x * 2, shape.halfSize.y * 2);

  Vec2 v = body.getPos();
  if (dragVec2("position", v, 0.05f))
    body.setPos(v);
  v = body.getVel();
  if (dragVec2("velocity", v, 0.05f))
    body.setVel(v);
  v = body.getAcc();
  if (dragVec2("acceleration", v, 0.05f))
    body.setAcc(v);

  float f = body.getAngle();
  if (ImGui::DragFloat("angle", &f, 0.01f, 0, 0, "%.3f rad"))
    body.setAngle(f);
  f = body.getAngVel();
  if (ImGui::DragFloat("angular vel", &f, 0.01f, 0, 0, "%.3f rad/s"))
    body.setAngVel(f);

  bool isStatic = body.getStatic();
  if (ImGui::Checkbox("static", &isStatic))
  {
    if (isStatic)
      body.setStatic(true);
    else
      body.setMass(1);
  }
  ImGui::BeginDisabled(isStatic);
  f = body.getMass();
  if (ImGui::DragFloat("mass", &f, 0.05f, 0.01f, 1000, "%.2f", ImGuiSliderFlags_AlwaysClamp))
    body.setMass(f);
  ImGui::EndDisabled();
  f = body.getInertia();
  if (ImGui::DragFloat("inertia", &f, 0.05f, 0.01f, 1000, "%.2f", ImGuiSliderFlags_AlwaysClamp))
    body.setInertia(f);

  f = body.getRestitution();
  if (ImGui::SliderFloat("restitution", &f, 0, 1))
    body.setRestitution(f);
  f = body.getLinearDamping();
  if (ImGui::SliderFloat("linear damping", &f, 0, 1))
    body.setLinearDamping(f);
  f = body.getAngularDamping();
  if (ImGui::SliderFloat("angular damping", &f, 0, 1))
    body.setAngularDamping(f);
}

static void drawUI(App &app)
{
  // fixed sizes below are in unscaled pixels
  const float scale = ImGui::GetStyle().FontScaleDpi;
  ImGui::SetNextWindowPos(ImVec2(8 * scale, 8 * scale), ImGuiCond_FirstUseEver);
  ImGui::Begin("ephys", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::PushItemWidth(160 * scale);

  if (ImGui::CollapsingHeader("simulation", ImGuiTreeNodeFlags_DefaultOpen))
  {
    ImGui::Checkbox("paused", &app.paused);
    ImGui::SameLine();
    ImGui::BeginDisabled(!app.paused);
    if (ImGui::Button("step"))
      app.stepOnce = true;
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("reset"))
      app.reset();
    ImGui::SliderFloat("time scale", &app.timeScale, 0.05f, 4, "%.2fx", ImGuiSliderFlags_Logarithmic);
    ImGui::Text("%.1f fps, step %.2f ms", ImGui::GetIO().Framerate, app.stepMs);
    ImGui::TextDisabled("space: pause, right: step, r: reset");
  }

  if (ImGui::CollapsingHeader("view", ImGuiTreeNodeFlags_DefaultOpen))
  {
    ImGui::Checkbox("colliders", &app.render.colliders);
    ImGui::SameLine();
    ImGui::Checkbox("velocities", &app.render.velocities);
    ImGui::SliderFloat("zoom", &app.camera.size, MIN_ZOOM, MAX_ZOOM, "%.0f px/unit", ImGuiSliderFlags_Logarithmic);
    if (ImGui::Button("reset camera"))
      app.resetCamera();
    ImGui::TextDisabled("right drag: pan, wheel: zoom");
  }

  if (ImGui::CollapsingHeader("dragging", ImGuiTreeNodeFlags_DefaultOpen))
  {
    // shorter times stiffen the spring, which the explicit integrator can't keep stable
    ImGui::SliderFloat("settle time", &app.scene.dragSettleTime, 0.2f, 5, "%.2f s", ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_AlwaysClamp);
    if (app.scene.isDragging())
      ImGui::Text("stiffness %.1f, damping %.2f", app.scene.getDragStiffness(), app.scene.getDragDamping());
    else
      ImGui::TextDisabled("stiffness and damping scale with mass");
  }

  std::vector<Shape> &shapes = app.scene.getShapes();
  if (ImGui::CollapsingHeader("bodies", ImGuiTreeNodeFlags_DefaultOpen))
  {
    ImGui::Text("%d bodies", (int)shapes.size());
    ImGui::TextDisabled("click a body to select it, drag to pull it");

    float swatch = ImGui::GetTextLineHeight();
    ImGui::BeginChild("body list", ImVec2(260 * scale, 8 * ImGui::GetTextLineHeightWithSpacing()), ImGuiChildFlags_Borders);
    for (int i = 0; i < (int)shapes.size(); i++)
    {
      ImGui::PushID(i);
      SDL_FColor c = shapeColor(i);
      ImGui::ColorButton("##color", ImVec4(c.r, c.g, c.b, c.a), ImGuiColorEditFlags_NoTooltip, ImVec2(swatch, swatch));
      ImGui::SameLine();
      const char *type = shapes[i].type == Shape::Type::Circle ? "circle" : "box";
      char label[32];
      SDL_snprintf(label, sizeof(label), "%s %d", type, (int)shapes[i].body->id);
      if (ImGui::Selectable(label, app.render.selected == i))
        app.render.selected = app.render.selected == i ? -1 : i;
      ImGui::PopID();
    }
    ImGui::EndChild();

    if (app.render.selected >= 0 && app.render.selected < (int)shapes.size())
    {
      ImGui::SeparatorText("selected");
      drawInspector(shapes[app.render.selected]);
    }
  }

  ImGui::PopItemWidth();
  ImGui::End();
}

// everything the SDL callbacks share, passed around as appstate
struct State
{
  SDL_Window *window = nullptr;
  SDL_Renderer *renderer = nullptr;
  App app;
  Uint64 prevTicks = 0;
#ifdef SDL_PLATFORM_EMSCRIPTEN
  Uint64 nextFrame = 0; // earliest time the next frame renders, see SDL_AppIterate
#endif
  ImFont *pixelFont = nullptr;  // ProggyClean, only sharp at whole multiples of its 13px size
  ImFont *vectorFont = nullptr; // scalable, for fractional display scales
};

// camera needs the viewport size before mouse events are mapped to world space
static void updateViewport(State &state)
{
  int width, height;
  SDL_GetCurrentRenderOutputSize(state.renderer, &width, &height);
  state.app.camera.width = (float)width;
  state.app.camera.height = (float)height;
  // render pixels per unscaled pixel: the browser's devicePixelRatio on the web,
  // the OS scale setting on Windows, the backing scale on macOS
  state.app.camera.scale = SDL_GetWindowDisplayScale(state.window);
}

// imgui lays out and draws in render pixels (see SDL_AppIterate), so its style and font take
// the whole display scale; rebuilt from scratch since ScaleAllSizes compounds
static void updateUiScale(State &state)
{
  float scale = SDL_GetWindowDisplayScale(state.window);
  ImGuiStyle &style = ImGui::GetStyle();
  style = ImGuiStyle();
  ImGui::StyleColorsDark(&style);
  style.ScaleAllSizes(scale);
  style.FontScaleDpi = scale;
  ImGui::GetIO().FontDefault = scale == SDL_floorf(scale) ? state.pixelFont : state.vectorFont;
}

SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[])
{
  if (!SDL_Init(SDL_INIT_VIDEO))
  {
    SDL_Log("SDL_Init failed: %s", SDL_GetError());
    return SDL_APP_FAILURE;
  }

  State *state = new State;
  *appstate = state;
  // window sizes are in OS units, which are pixels on Windows, so scale the default size too
  // (ignored on the web, where css sizes the canvas)
  float scale = SDL_GetDisplayContentScale(SDL_GetPrimaryDisplay());
  if (scale <= 0)
    scale = 1;
  SDL_WindowFlags flags = SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY;
#ifdef SDL_PLATFORM_EMSCRIPTEN
  // create the window ready for the GLES2 (WebGL) renderer; otherwise the renderer recreates it,
  // and tearing down a HiDPI window leaves an inline 1px css size on the canvas, so the new
  // window no longer sees the stylesheet's size and falls back to 1280x720
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
  flags |= SDL_WINDOW_OPENGL;
#endif
  if (!SDL_CreateWindowAndRenderer("ephys", (int)(1280 * scale), (int)(720 * scale), flags, &state->window, &state->renderer))
  {
    SDL_Log("SDL_CreateWindowAndRenderer failed: %s", SDL_GetError());
    return SDL_APP_FAILURE;
  }
  SDL_SetRenderVSync(state->renderer, 1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.IniFilename = nullptr; // don't write imgui.ini next to the exe
  state->pixelFont = io.Fonts->AddFontDefaultBitmap();
  state->vectorFont = io.Fonts->AddFontDefaultVector();
  updateUiScale(*state);
  ImGui_ImplSDL3_InitForSDLRenderer(state->window, state->renderer);
  ImGui_ImplSDLRenderer3_Init(state->renderer);

#ifdef SDL_PLATFORM_EMSCRIPTEN
  SDL_SetEventFilter(filterKeys, nullptr);
#else
  // SDL sleeps between iterations to hold this rate; vsync can still lower it
  char rate[16];
  SDL_snprintf(rate, sizeof(rate), "%d", MAX_FPS);
  SDL_SetHint(SDL_HINT_MAIN_CALLBACK_RATE, rate);
#endif

  updateViewport(*state);
  state->prevTicks = SDL_GetTicksNS();
  return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event)
{
  State &state = *(State *)appstate;

  // imgui and the camera both work in render pixels
  SDL_ConvertEventToRenderCoordinates(state.renderer, event);
  ImGui_ImplSDL3_ProcessEvent(event);
  if (event->type == SDL_EVENT_QUIT)
    return SDL_APP_SUCCESS;
  if (event->type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && event->window.windowID == SDL_GetWindowID(state.window))
    return SDL_APP_SUCCESS;

  // moved to another monitor, OS scale changed, or browser zoom / devicePixelRatio changed
  if (event->type == SDL_EVENT_WINDOW_DISPLAY_SCALE_CHANGED)
    updateUiScale(state);

  updateViewport(state);
  handleEvent(state.app, *event);
  return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void *appstate)
{
  State &state = *(State *)appstate;
  App &app = state.app;

  Uint64 ticks = SDL_GetTicksNS();
#ifdef SDL_PLATFORM_EMSCRIPTEN
  // the browser calls this from requestAnimationFrame at the display's refresh rate (the rate
  // hint would swap that for imprecise timers), so cap the rate by skipping frames instead;
  // the deadline falls at most one frame behind, so a 144 Hz display still averages MAX_FPS,
  // and the tolerance keeps timing jitter from halving the rate on a MAX_FPS display
  constexpr Uint64 FRAME_NS = SDL_NS_PER_SECOND / MAX_FPS, FRAME_TOLERANCE_NS = SDL_NS_PER_MS;
  if (ticks + FRAME_TOLERANCE_NS < state.nextFrame)
    return SDL_APP_CONTINUE;
  if (state.nextFrame + FRAME_NS < ticks)
    state.nextFrame = ticks - FRAME_NS;
  state.nextFrame += FRAME_NS;
#endif
  float elapsed = SDL_min((ticks - state.prevTicks) / 1e9f, MAX_FRAME_TIME);
  state.prevTicks = ticks;

  updateViewport(state);

  // simulated time this frame
  float dt = app.paused ? (app.stepOnce ? PHYSICS_STEP : 0) : elapsed * app.timeScale;
  app.stepOnce = false;

  // every frame, so the anchor stays under the cursor while panning or zooming
  app.scene.moveDrag(app.camera.toWorld(app.mouse), dt);

  // fixed steps, so the sim behaves the same at any frame rate or time scale;
  // whatever is left under a step carries over to the next frame
  app.pendingTime += dt;
  if (app.pendingTime >= PHYSICS_STEP)
  {
    Uint64 start = SDL_GetPerformanceCounter();
    for (; app.pendingTime >= PHYSICS_STEP; app.pendingTime -= PHYSICS_STEP)
      app.scene.step(PHYSICS_STEP);
    float ms = (SDL_GetPerformanceCounter() - start) * 1000.f / SDL_GetPerformanceFrequency();
    app.stepMs = app.stepMs == 0 ? ms : app.stepMs * 0.95f + ms * 0.05f;
  }

  ImGui_ImplSDLRenderer3_NewFrame();
  ImGui_ImplSDL3_NewFrame();
  // run imgui in render pixels instead of window units, so it snaps layout and text to physical
  // pixels; scaling window units up by a fractional pixel density lands edges between pixels
  ImGuiIO &io = ImGui::GetIO();
  if (io.DisplaySize.x > 0 && io.DisplaySize.y > 0) // zero while minimized
    io.DisplaySize = ImVec2(app.camera.width, app.camera.height);
  io.DisplayFramebufferScale = ImVec2(1, 1);
  ImGui::NewFrame();
  drawUI(app);
  ImGui::Render();

  // background matches web/css/styles.css (#222)
  SDL_SetRenderDrawColor(state.renderer, 0x22, 0x22, 0x22, 0xff);
  SDL_RenderClear(state.renderer);
  drawScene(state.renderer, app.scene, app.camera, app.render);
  ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), state.renderer);
  SDL_RenderPresent(state.renderer);
  return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result)
{
  State *state = (State *)appstate;
  if (ImGui::GetCurrentContext())
  {
    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
  }
  if (!state)
    return; // SDL_Init failed before state was allocated
  if (state->renderer)
    SDL_DestroyRenderer(state->renderer);
  if (state->window)
    SDL_DestroyWindow(state->window);
  delete state; // SDL_Quit is called for us after this returns
}
