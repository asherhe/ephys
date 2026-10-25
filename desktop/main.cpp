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
// how far a single step advances the sim while paused
constexpr float STEP_TIME = 1 / 60.f;

constexpr float DEFAULT_ZOOM = 40, MIN_ZOOM = 4, MAX_ZOOM = 400;
constexpr float WHEEL_ZOOM_FACTOR = 1.1f;

struct App
{
  Scene scene;
  Camera camera;
  RenderOptions render;

  bool paused = false;
  bool stepOnce = false; // advance by STEP_TIME on the next frame while paused
  float timeScale = 1;
  float stepMs = 0; // smoothed time spent in World::step per frame
  bool panning = false;

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

// event coordinates must already be in render coordinates
static void handleEvent(App &app, const SDL_Event &event)
{
  const ImGuiIO &io = ImGui::GetIO();
  switch (event.type)
  {
  case SDL_EVENT_KEY_DOWN:
    if (io.WantCaptureKeyboard)
      break;
    if (event.key.key == SDLK_SPACE && !event.key.repeat)
      app.paused = !app.paused;
    else if (event.key.key == SDLK_RIGHT && app.paused)
      app.stepOnce = true;
    else if (event.key.key == SDLK_R && !event.key.repeat)
      app.reset();
    break;

  case SDL_EVENT_MOUSE_BUTTON_DOWN:
    if (io.WantCaptureMouse)
      break;
    // left click selects (or deselects on empty space), right/middle drag pans
    if (event.button.button == SDL_BUTTON_LEFT)
      app.render.selected = app.scene.pick(app.camera.toWorld({event.button.x, event.button.y}));
    else
      app.panning = true;
    break;

  case SDL_EVENT_MOUSE_BUTTON_UP:
    if (event.button.button != SDL_BUTTON_LEFT)
      app.panning = false;
    break;

  case SDL_EVENT_MOUSE_MOTION:
    if (app.panning)
    {
      // screen is y-down, world is y-up
      app.camera.x -= event.motion.xrel / app.camera.size;
      app.camera.y += event.motion.yrel / app.camera.size;
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
  ImGui::SetNextWindowPos(ImVec2(8, 8), ImGuiCond_FirstUseEver);
  ImGui::Begin("ephys", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::PushItemWidth(160);

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

  std::vector<Shape> &shapes = app.scene.getShapes();
  if (ImGui::CollapsingHeader("bodies", ImGuiTreeNodeFlags_DefaultOpen))
  {
    ImGui::Text("%d bodies", (int)shapes.size());
    ImGui::TextDisabled("click a body to select it");

    float swatch = ImGui::GetTextLineHeight();
    ImGui::BeginChild("body list", ImVec2(260, 8 * ImGui::GetTextLineHeightWithSpacing()), ImGuiChildFlags_Borders);
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

int main(int argc, char *argv[])
{
  if (!SDL_Init(SDL_INIT_VIDEO))
  {
    SDL_Log("SDL_Init failed: %s", SDL_GetError());
    return 1;
  }

  SDL_Window *window;
  SDL_Renderer *renderer;
  if (!SDL_CreateWindowAndRenderer("ephys", 1280, 720, SDL_WINDOW_RESIZABLE, &window, &renderer))
  {
    SDL_Log("SDL_CreateWindowAndRenderer failed: %s", SDL_GetError());
    SDL_Quit();
    return 1;
  }
  SDL_SetRenderVSync(renderer, 1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::GetIO().IniFilename = nullptr; // don't write imgui.ini next to the exe
  ImGui::StyleColorsDark();
  ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
  ImGui_ImplSDLRenderer3_Init(renderer);

  App app;

  Uint64 prevTicks = SDL_GetTicksNS();
  bool running = true;
  while (running)
  {
    Uint64 ticks = SDL_GetTicksNS();
    float elapsed = SDL_min((ticks - prevTicks) / 1e9f, MAX_FRAME_TIME);
    prevTicks = ticks;

    // camera needs the viewport size before mouse events are mapped to world space
    int width, height;
    SDL_GetCurrentRenderOutputSize(renderer, &width, &height);
    app.camera.width = (float)width;
    app.camera.height = (float)height;

    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
      // imgui wants window coordinates, the camera wants render coordinates
      ImGui_ImplSDL3_ProcessEvent(&event);
      SDL_ConvertEventToRenderCoordinates(renderer, &event);
      if (event.type == SDL_EVENT_QUIT)
        running = false;
      else if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && event.window.windowID == SDL_GetWindowID(window))
        running = false;
      else
        handleEvent(app, event);
    }

    float dt = app.paused ? (app.stepOnce ? STEP_TIME : 0) : elapsed;
    dt *= app.timeScale;
    app.stepOnce = false;

    // physics runs at twice the render rate, like the web demo
    // Rigidbody::step asserts dt > 0, so skip frames where no time has passed
    if (dt > 0)
    {
      Uint64 start = SDL_GetPerformanceCounter();
      app.scene.step(dt / 2);
      app.scene.step(dt / 2);
      float ms = (SDL_GetPerformanceCounter() - start) * 1000.f / SDL_GetPerformanceFrequency();
      app.stepMs = app.stepMs == 0 ? ms : app.stepMs * 0.95f + ms * 0.05f;
    }

    ImGui_ImplSDLRenderer3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();
    drawUI(app);
    ImGui::Render();

    // background matches web/css/styles.css (#222)
    SDL_SetRenderDrawColor(renderer, 0x22, 0x22, 0x22, 0xff);
    SDL_RenderClear(renderer);
    drawScene(renderer, app.scene, app.camera, app.render);
    ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
    SDL_RenderPresent(renderer);
  }

  ImGui_ImplSDLRenderer3_Shutdown();
  ImGui_ImplSDL3_Shutdown();
  ImGui::DestroyContext();
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
