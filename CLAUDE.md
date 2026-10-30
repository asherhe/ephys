# CLAUDE.md

ephys: a 2D rigidbody physics engine in C++ (modeled on cyclone physics), with one SDL3 + Dear ImGui app in `desktop/` that builds both natively (fast local builds and debugging) and to WebAssembly with Emscripten for the browser demo in `web/`.

## Commands

- Builds use CMake presets (`CMakePresets.json`, Ninja). Each preset has its own dir, `build/<preset>/`. `cmake --workflow --preset <name>` configures if needed, then builds incrementally. The first configure of each preset fetches SDL3 and ImGui via `FetchContent`.
- **Web:** `cmake --workflow --preset web-debug` (or `web-release` for `-O3`). Requires the `EMSDK` env var (the preset loads Emscripten's toolchain file from it). Output goes to `web/dist/`.
- **Run web demo:** build, then serve `web/dist/` with any static HTTP server (e.g. `python -m http.server -d web/dist`) and open `index.html` (wasm needs HTTP, not `file://`).
- **Desktop:** `cmake --workflow --preset desktop-debug` (or `desktop-release`).
- **Run desktop:** `build/desktop-debug/desktop/ephys_app.exe`. Space or `k` pauses, `.` steps, `r` resets, `j`/`l` change speed by 0.25x; left click selects, right drag pans, wheel zooms. Same controls on the web.
- No test suite or linter exists.

## Architecture

- `includes/ephys/*.h` + `src/*.cpp`: engine, all in namespace `ephys`, built as the `ephys_core` static lib. Two parallel engines:
  - Rigidbody: `rigidbody`, `collider`, `collision` (BVH broadphase), `contacts`, `forcegen`, `world`.
  - Particle (files prefixed `p`): `particle`, `pcontact(s)`, `pforcegen`, `plinks`, `pworld`.
- `desktop/`: the app (target `ephys_app`), built for every toolchain. Uses SDL3's main callbacks (`SDL_MAIN_USE_CALLBACKS`: `SDL_AppInit/Event/Iterate/Quit` in `main.cpp`) so the browser can drive frames; rendering is SDL_Renderer (WebGL via `ogl_es2` on the web).
- `web/`: added by the root `CMakeLists.txt` only under Emscripten. `web/CMakeLists.txt` sets `ephys_app`'s wasm link options and output, and target `ephys_site` copies the static site.
- Web output goes to `web/dist/js/ephys/ephys.{js,wasm}` (MODULARIZE, export name `ephys`); `ephys_site` copies `web/css`, `web/img`, `web/js` and `web/index.html` into `web/dist/`. `web/js/main.js` loads the module with `ephys({ canvas })` and handles the loader fade with jQuery; the app draws into `#canvas`.
- Canvas sizing: `#canvas` is sized full window by CSS (`web/css/styles.css`). The SDL window is `SDL_WINDOW_RESIZABLE`, so on each browser resize SDL reads the canvas's CSS size, resizes the backing store and sends `SDL_EVENT_WINDOW_RESIZED`. Don't set `canvas.width/height` from JS, and don't use `SDL_WINDOW_FILL_DOCUMENT` (it hides the rest of the page).
- `web/ephys.idl` and `web/ephys-headers.h` are the old WebIDL JS bindings, kept for reference but not built.

## Key Invariants

- New engine `.cpp` files must be added to `EPHYS_SOURCES` in the root `CMakeLists.txt`.
- If the WebIDL bindings are revived, engine API changes JS uses must also be mirrored in `web/ephys-headers.h` and `web/ephys.idl` (conventions at the top of the IDL).
- `web/dist/` is gitignored build output: edit the site sources in `web/`, never `web/dist/`. CI (`.github/workflows/pages.yml`) runs only on pushes to `main`: it builds `web-release`, uploads `web/dist/` as a Pages artifact and deploys it. Nothing is committed back to the repo.
