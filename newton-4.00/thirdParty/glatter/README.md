# glatter - OpenGL‑family loader & tracer


A practical loader and tracer for GL‑family APIs (GL, GLX, WGL, EGL, GLES, optional GLU). It resolves symbols on first use, allows logging calls or just errors, and warns on cross‑thread usage. It works in C and C++ with sensible defaults.

---


## Platform Support

| Platform    | Status | Notes |
| :---        | :--- | :--- |
| **Windows** | [![Windows build status](https://github.com/imakris/glatter/actions/workflows/ci-windows.yml/badge.svg?branch=master)](https://github.com/imakris/glatter/actions/workflows/ci-windows.yml) | Tested with WGL via GitHub Actions. |
| **Linux**   | [![Linux build status](https://github.com/imakris/glatter/actions/workflows/ci-linux.yml/badge.svg?branch=master)](https://github.com/imakris/glatter/actions/workflows/ci-linux.yml) | Tested with GLX via GitHub Actions. |
| **FreeBSD** | [![FreeBSD build status](https://github.com/imakris/glatter/actions/workflows/ci-freebsd.yml/badge.svg?branch=master)](https://github.com/imakris/glatter/actions/workflows/ci-freebsd.yml) | Tested with GLX via GitHub Actions (QEMU VM). |
| **macOS**   | [![macOS build status](https://github.com/imakris/glatter/actions/workflows/ci-macos.yml/badge.svg?branch=master)](https://github.com/imakris/glatter/actions/workflows/ci-macos.yml) | Tested with GLX via GitHub Actions, but requires XQuartz for GLX compatibility. Not a native (CGL) build. |

---

## Technical summary

* **Integration:** header‑only (C++) or compiled translation unit (C/C++).
* **WSI detection:** auto‑detects WGL/GLX/EGL with optional runtime override.
* **On‑demand symbols:** function pointers are resolved on first use to minimize startup work.
* **Diagnostics:** opt‑in call/error logging; in debug builds (when `NDEBUG` is not defined) errors are logged by default. Messages go to stdout/stderr; a custom handler can be installed for redirection.
* **Utilities:** generated extension flags (e.g., `glatter_GL_ARB_vertex_array_object`) and `enum_to_string_*()` helpers.


---


## Choosing an integration mode

Glatter supports two primary integration modes:

### 1) Header‑only (C++ only)

This mode provides the simplest C++ integration.
```cpp
#include <glatter/glatter_solo.h>

void setup_scene() {
    // ...
    // Window and OpenGL context creation happens here...
    // ...

    // glatter loads the function pointer on the first call.
    // This would fail to link without a loader.
    GLuint vao = 0;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    // ...
    // The main rendering loop would follow...
    // ...
}
```
For header-only usage, include `<glatter/glatter_solo.h>`. This tiny wrapper defines
`GLATTER_HEADER_ONLY` and includes the main header.
Note: On MinGW/Windows, header-only mode requires C++17 (inline variables) to
avoid multiple-definition linker errors.

### 2) Compiled C translation unit (C or C++)

```c
#include <glatter/glatter.h>
/* The build must include src/glatter/glatter.c */

void setup_scene() {
    // ...
    // Window and OpenGL context creation happens here...
    // ...

    // The usage is identical to header-only mode.
    // This would fail to link without a loader.
    GLuint vao = 0;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    // ...
    // The main rendering loop would follow...
    // ...
}
```

**Note:** System GL headers (e.g., `GL/gl.h`, `EGL/egl.h`) should not be included directly. Glatter chooses the right ones.

---

## Quick start

1. The `include/` directory must be present in the compiler’s include paths.
2. Select either **header‑only** (C++) or **compiled TU** (C/C++), as shown above.
3. Link platform libraries (see Integration notes).
4. Optional: install a custom log sink to redirect messages away from stdout/stderr.

### CMake smoke tests for CI/CD

A companion CMake configuration replicates the build smoke-tests previously implemented in `tests/test_build.py`. It compiles representative consumers for both the compiled C translation unit and the header-only C++ modes, checks the static-library workflow, and exercises the internal EGL context-key helper using stubbed entry points.

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

The default build also verifies that the WGL headers continue to compile when paired with the lightweight Windows stubs in `tests/include/`. These targets have no runtime dependencies and are meant purely as CI/CD guardrails.

---

## Window System Interface (WSI) selection (auto and overrides)

Glatter auto‑selects the appropriate Window System Interface (WSI) for the target platform.

Optional WSI override (function call or env var):

```c
/* Optional WSI override (defaults are auto‑detected).
   Alternatively set: GLATTER_WSI=egl|glx|wgl */

glatter_set_wsi(GLATTER_WSI_EGL); /* or WGL, GLX, AUTO */
```

WSI is latched at **first successful resolution** in the process. Changes to the environment variable or calls to
`glatter_set_wsi()` after first use have no effect for the remainder of the process.

**Thread-safety & determinism:** In `AUTO` mode, WSI detection is fully thread-safe. A tiny atomic gate ensures the decision is made exactly once, proceeding in a fixed order (Windows: WGL→EGL; POSIX: GLX→EGL). Both header-only and compiled TU modes are functionally correct and thread-safe. The choice is one of project architecture:

*   **Header-only (C++):** State is managed per translation unit. All TUs will deterministically converge to the same WSI.
*   **Compiled TU (C/C++):** State is centralized in a single, shared object, which can reduce code size.


## Typical single‑context app

```cpp
#include <glatter/glatter_solo.h> // header‑only C++ header
// for C/C++ compiled TU use <glatter/glatter.h> and compile src/glatter/glatter.c

int main() {
    // Create and make a GL context current here
    // ...

    // Gate once on a requirement (example: VAOs)
    if (!glatter_GL_ARB_vertex_array_object) {
        // Extension not available → bail out or use a fallback
        return 1;
    }

    // Safe to use the corresponding functions
    GLuint vao = 0;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glClear(GL_COLOR_BUFFER_BIT);
    return 0;
}
```

## API summary (essentials)

* **WSI override/inspect**: `glatter_set_wsi(GLATTER_WSI_*)`, `glatter_get_wsi()` (APIs use the term "Window System Interface (WSI)").
* **Extension flags**: test generated flags like `glatter_GL_ARB_vertex_array_object` once the context is current.
* **Enum names**: `enum_to_string_*()` for readable GL/GLX/WGL/EGL/GLU enums.

Notes: Diagnostics and multi‑context thread checks are covered under **Tracing & diagnostics**. Low‑level entry‑point helpers are documented under **Advanced** and are rarely needed.

## Tracing & error logging

By default, messages go to stdout/stderr. A custom log handler should only be installed to redirect or integrate with a different logging system.

Two independent, opt‑in layers:

* `GLATTER_LOG_CALLS` Log every wrapped call (name, args, return).
* `GLATTER_LOG_ERRORS` Log only API errors.

If neither is defined, **debug builds default to error‑only logging** (i.e., when `NDEBUG` is not defined).

To redirect output to a sink:

```c
void my_log_sink(const char* msg) { /* route to a logger */ }
int main() {
    glatter_set_log_handler(my_log_sink);
}
```

> Note: ARB/KHR debug output still needs a debug context; glatter’s error checks work independently.

For WGL wrappers, glatter sets `SetLastError(0)` immediately before the call so the subsequent
`GetLastError()` check reflects that call. This avoids reporting a stale error from unrelated
WinAPI calls; any `GetLastError()` observed right after the wrapper reflects that specific WGL call.

---

## Thread ownership checks

Glatter tracks an "owner thread" and warns when calls come from a different thread.

* **Header‑only C++:** first touching thread becomes owner. Explicit control is available by calling `glatter_bind_owner_to_current_thread()` early. Defining `GLATTER_REQUIRE_EXPLICIT_OWNER_BIND` requires an explicit bind; otherwise the library aborts on first use without binding.
* **Compiled C/C++:** the owner is captured on first use; later calls from other threads are reported.

These checks are diagnostic only. Glatter does not serialize or block.

---

## Extension requirements

Use the generated boolean flags `glatter_<EXTENSION_NAME>` after a context is current. Example (VAOs):

```c
if (!glatter_GL_ARB_vertex_array_object) {
    /* Report and exit early or use a fallback */
    return 1;
}

GLuint vao = 0;
glGenVertexArrays(1, &vao);
glBindVertexArray(vao);
```

---

## GLX Xlib error handler

When using the GLX WSI, glatter installs a small X error handler the first time GLX is touched, to surface common GLX/Xlib issues. Opting out and installing a custom handler (for example after `XInitThreads()`) requires defining:

```c
#define GLATTER_DO_NOT_INSTALL_X_ERROR_HANDLER
```

**Xlib threading:** Applications that use Xlib from multiple threads must call `XInitThreads()` **before any Xlib call**.
Glatter does not call it. In multi-threaded Xlib apps, invoking `XInitThreads()` early (e.g., at program start) and
defining `GLATTER_DO_NOT_INSTALL_X_ERROR_HANDLER` enables installation of a custom handler under a specific threading model.

---

## Building & Integration

Glatter is a simple, dependency-light library designed for easy integration using CMake.

### Using Glatter in a Project (Consumer)

Projects that consume `glatter` via CMake typically build and install it before linking. Examples follow:

**C++ app (header‑only):**
```cmake
# Add the include directory
target_include_directories(my_app PRIVATE path/to/glatter/include)

# Link platform libraries
if(WIN32)
  target_link_libraries(my_app PRIVATE opengl32)
else()
  # On POSIX, link against GL, Threads, X11, and dl
  find_package(Threads REQUIRED)
  target_link_libraries(my_app PRIVATE GL Threads::Threads X11::X11 dl)
endif()
```

**C/C++ app (compiled TU):**
```cmake
# Find the installed glatter package
find_package(glatter REQUIRED)

# Link it to the application
target_link_libraries(my_app PRIVATE glatter::glatter)
```

### Building Glatter from Source

Building the compiled library is straightforward. Standard build tools (CMake, C/C++ compiler) and the development libraries for OpenGL on the system are required.

```sh
# Configure the project
cmake -B build -S .

# Build the library
cmake --build build --config Release
```

---

## Advanced configuration

### Windows character encoding

Handles UNICODE and MBCS builds. The generator assumes UNICODE by default; on non-UNICODE builds, the `GLATTER_WINDOWS_MBCS` switch is automatically set unless defined otherwise. This keeps TCHAR handling correct without extra setup.

### Regenerating headers (optional)

Two headers are meant for power users:

* `glatter_config.h` Feature/platform switches. Define `GLATTER_USER_CONFIGURED` and set custom `GLATTER_*` macros to opt out of the defaults.
* `glatter_platform_headers.h` The list of API headers glatter should use. If this is edited, re‑run the generator.

1. Place target API headers under `include/glatter/headers/`.
2. Add the bundle to `glatter_platform_headers.h` (one `#include` per line, no trailing comments).
3. Run either:
   * `python include/glatter/glatter.py`
   * `cd include/glatter && python glatter.py`

If `GLATTER_HAS_EGL_GENERATED_HEADERS` is off for a target, EGL/GLES helpers are unavailable until the headers are generated; the library will still build.

---

## Troubleshooting

* **Undefined references to X11 symbols (e.g., `XGetErrorText`)** Link against the platform libraries listed above for POSIX targets (`GL`, `Threads`, `X11`, `dl`). When consuming the installed CMake target (`glatter::glatter`), these usage requirements propagate automatically; manual build systems must add `-lX11` (and friends) explicitly.
* **“Failed to resolve …”** A current context is required, and GL/EGL/GLES libraries must be visible on the platform.
* **Cross‑thread warnings** `glatter_bind_owner_to_current_thread()` on the render thread provides explicit ownership, and strict binding is available through `GLATTER_REQUIRE_EXPLICIT_OWNER_BIND`.
* **GLX error spam** Defining `GLATTER_DO_NOT_INSTALL_X_ERROR_HANDLER` and installing a custom handler after X threading initialization suppresses the default handler.
* **Missing EGL/GLES generated headers** Builds still succeed; EGL/GLES helpers are unavailable until the headers are generated.
* **Platform family mismatch** Enabling a platform wrapper that is unavailable on the target (e.g., WGL on Linux,
  GLX on Windows) leads to failures. Enable only the families that exist on the current platform.
* **WSI override not taking effect** The WSI is latched at first use. Set `GLATTER_WSI` or call `glatter_set_wsi()` **before**
  the first GL/WSI call.
---

## License

BSD‑2‑Clause (Simplified). See header prologs for the full text.
