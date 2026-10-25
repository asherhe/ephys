# ephys

A simple 2d rigidbody physics engine inspired by [cyclone physics](https://github.com/idmillington/cyclone-physics/) ([Game Physics Engine Development](https://www.amazon.com/Game-Physics-Engine-Development-Commercial-Grade/dp/0123819768)).

[live demo](https://asherhe.github.io/ephys/)

## Building

ephys is compiled to WebAssembly with [Emscripten](https://emscripten.org/). The build produces `web/dist/js/ephys/ephys.js` and `web/dist/js/ephys/ephys.wasm`, then copies the demo page's files from `web/` into `web/dist/` alongside them.

### Dependencies

- [Emscripten SDK (emsdk)](https://emscripten.org/docs/getting_started/downloads.html): provides `emcc`, `emcmake`, and the WebIDL binder used to generate the JS bindings
- [CMake](https://cmake.org/) 3.10 or newer
- [Python 3](https://www.python.org/), available on your `PATH` as `python` (the build runs Emscripten's `webidl_binder.py` with it)
- A CMake build tool: Make on Linux/macOS, MinGW Make (`mingw32-make`) on Windows, or [Ninja](https://ninja-build.org/) anywhere
- [Git](https://git-scm.com/), to clone emsdk

### Set up Emscripten

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

This puts `emcc` and `emcmake` on your `PATH` and sets the `EMSDK` environment variable, which the build needs to find the WebIDL binder.

### Build

From the repository root:

```sh
emcmake cmake -S . -B build -D CMAKE_BUILD_TYPE=Debug
cmake --build build -t ephys
```

Use `-D CMAKE_BUILD_TYPE=Release` for an optimized (`-O3`) build. To use Ninja, add `-G Ninja` to the first command.

Rebuilds are incremental: editing anything in `src/`, `includes/` or `web/` rebuilds the wasm module. On Windows, `build.bat` runs the build command.

The wasm build's files (`ephys.idl`, `ephys-headers.h` and the target's `CMakeLists.txt`) live in `web/`, along with the demo page's source (`index.html`, `css/`, `img/` and `js/`). `web/dist/` is generated on every build, so edit the files in `web/` instead.

### Run the demo

Build first, then serve `web/dist/`. The demo has to be served over HTTP, because browsers won't load `.wasm` files from `file://`. Any static file server works:

```sh
python -m http.server 8000 -d web/dist
```

Then open <http://localhost:8000>.
