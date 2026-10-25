# ephys

A simple 2d rigidbody physics engine inspired by [cyclone physics](https://github.com/idmillington/cyclone-physics/) ([Game Physics Engine Development](https://www.amazon.com/Game-Physics-Engine-Development-Commercial-Grade/dp/0123819768)).

[live demo](https://asherhe.github.io/ephys/)

## Building

ephys has two CMake targets, both built from the engine sources in `src/` and `includes/`:

- `ephys`: the engine compiled to WebAssembly with Emscripten, plus the browser demo in `web/`
- `ephys_desktop`: a native demo app using [SDL3](https://www.libsdl.org/) and [Dear ImGui](https://github.com/ocornut/imgui), in `desktop/`

Each target needs its own toolchain, so builds are set up with [CMake presets](https://cmake.org/cmake/help/latest/manual/cmake-presets.7.html) in `CMakePresets.json`. Every preset gets its own directory under `build/`:

| Preset | Target | Build directory |
| --- | --- | --- |
| `web-debug` / `web-release` | `ephys` | `build/web-debug/`, `build/web-release/` |
| `desktop-debug` / `desktop-release` | `ephys_desktop` | `build/desktop-debug/`, `build/desktop-release/` |

`cmake --workflow --preset <name>` configures (if needed) and builds in one step, and rebuilds are incremental. `cmake --list-presets` lists them. Editors with CMake preset support (VS Code's CMake Tools, Visual Studio, CLion) pick them up automatically.

### Common dependencies

- [CMake](https://cmake.org/) 3.25 or newer (for workflow presets)
- [Ninja](https://ninja-build.org/), which every preset uses as its generator
- [Git](https://git-scm.com/), to clone emsdk and for CMake to fetch the desktop dependencies

### Web

#### Dependencies

- [Emscripten SDK (emsdk)](https://emscripten.org/docs/getting_started/downloads.html): provides `em++`, `emcmake`, and the WebIDL binder used to generate the JS bindings
- [Python 3](https://www.python.org/), available on your `PATH` as `python` (the build runs Emscripten's `webidl_binder.py` with it)

#### Set up Emscripten

```sh
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk
./emsdk install latest
./emsdk activate latest
```

Then load the environment in every new shell you build from:

```sh
source ./emsdk_env.sh   # Linux/macOS
emsdk_env.bat           # Windows (cmd)
.\emsdk_env.ps1         # Windows (PowerShell)
```

This puts `em++` and `emcmake` on your `PATH` and sets the `EMSDK` environment variable. The web presets only need `EMSDK`: they load Emscripten's toolchain file from it, and the toolchain finds `em++` and the WebIDL binder on its own.

#### Build

From the repository root:

```sh
cmake --workflow --preset web-debug
```

Use `web-release` for an optimized (`-O3`) build.

The build compiles `web/dist/js/ephys/ephys.js` and `web/dist/js/ephys/ephys.wasm`, then copies the demo page's files (`index.html`, `css/`, `img/` and `js/`) from `web/` into `web/dist/` alongside them. `web/dist/` is regenerated on every build and isn't tracked by git, so edit the files in `web/` instead.

Editing anything in `src/`, `includes/` or `web/` rebuilds the wasm module on the next build.

The wasm build's files (`ephys.idl`, `ephys-headers.h` and the target's `CMakeLists.txt`) also live in `web/`.

#### Run the demo

Build first, then serve `web/dist/`. The demo has to be served over HTTP, because browsers won't load `.wasm` files from `file://`. Any static file server works:

```sh
python -m http.server 8000 -d web/dist
```

Then open <http://localhost:8000>.

#### Deployment

Every push to `main` runs `.github/workflows/pages.yml`, which does a Release build and deploys `web/dist/` to GitHub Pages. Build output is never committed to the repository.

### Desktop

#### Dependencies

- A C++17 compiler: GCC, Clang, MSVC, or [MinGW-w64](https://www.mingw-w64.org/) on Windows
- On Linux, the development packages SDL needs for windowing (X11 and/or Wayland). See SDL's [`docs/README-linux.md`](https://github.com/libsdl-org/SDL/blob/main/docs/README-linux.md).

SDL3 (`release-3.4.18`) and Dear ImGui (`v1.92.9b`) don't need to be installed. CMake downloads them with `FetchContent` the first time you configure, so that first configure needs a network connection and takes a while. SDL is built as a static library.

#### Build

From the repository root:

```sh
cmake --workflow --preset desktop-debug
```

Use `desktop-release` for an optimized build.

The presets use whichever C++ compiler CMake finds first. To pick a specific one, such as MinGW's `g++` on Windows, add a `CMakeUserPresets.json` (gitignored) that inherits from a preset and sets `CMAKE_CXX_COMPILER`. MinGW builds are linked statically, so the executable doesn't need the MinGW runtime DLLs on `PATH`.

#### Run

```sh
./build/desktop-debug/desktop/ephys_desktop
```

On Windows the executable is `ephys_desktop.exe`.

Space pauses, the right arrow steps once while paused, and `R` resets the scene. Left click selects a body, right drag pans, and the scroll wheel zooms.
