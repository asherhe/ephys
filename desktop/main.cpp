#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_sdlrenderer3.h"

#include "camera.h"
#include "render.h"
#include "scene.h"

// longest frame the sim will advance by, so stopping at a breakpoint doesn't blow up the next step
constexpr float MAX_FRAME_TIME = 0.05f;

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

  Scene scene;
  Camera camera;

  Uint64 prevTicks = SDL_GetTicksNS();
  bool running = true;
  while (running)
  {
    Uint64 ticks = SDL_GetTicksNS();
    float elapsed = SDL_min((ticks - prevTicks) / 1e9f, MAX_FRAME_TIME);
    prevTicks = ticks;

    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
      ImGui_ImplSDL3_ProcessEvent(&event);
      if (event.type == SDL_EVENT_QUIT)
        running = false;
      else if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && event.window.windowID == SDL_GetWindowID(window))
        running = false;
    }

    // physics runs at twice the render rate, like the web demo
    // Rigidbody::step asserts dt > 0, so skip frames where no time has passed
    if (elapsed > 0)
    {
      scene.step(elapsed / 2);
      scene.step(elapsed / 2);
    }

    ImGui_ImplSDLRenderer3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(8, 8), ImGuiCond_FirstUseEver);
    ImGui::Begin("ephys");
    ImGui::Text("%.1f fps", ImGui::GetIO().Framerate);
    ImGui::End();

    ImGui::Render();

    // background matches demos/css/styles.css (#222)
    SDL_SetRenderDrawColor(renderer, 0x22, 0x22, 0x22, 0xff);
    SDL_RenderClear(renderer);

    int width, height;
    SDL_GetCurrentRenderOutputSize(renderer, &width, &height);
    camera.width = (float)width;
    camera.height = (float)height;
    drawScene(renderer, scene, camera);

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
