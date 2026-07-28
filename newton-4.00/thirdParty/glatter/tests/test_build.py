"""Build smoke tests for the Glatter library.

These tests ensure that both the compiled C library variant and the
header-only C++ variant continue to build successfully.  They compile small
programs on the fly so that regressions in the public headers or the
implementation are caught early without requiring a rendering environment.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import textwrap
from dataclasses import dataclass
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]


def _khronos_static_flags() -> list[str]:
    if os.name == "nt":
        return ["-DKHRONOS_STATIC=1"]
    return []


def _thread_flags() -> list[str]:
    if os.name == "nt":
        return []
    return ["-pthread"]


def _dl_flags() -> list[str]:
    if os.name == "nt" or sys.platform == "darwin":
        return []
    return ["-ldl"]


def _opengl_libs() -> list[str]:
    if os.name == "nt":
        return ["-lopengl32"]
    return []


def _require_tool(executable: str) -> str:
    """Return the path to *executable* or skip the test if it is missing."""

    env_var_map = {"cc": "CC", "c++": "CXX"}
    lookup = env_var_map.get(executable, "")
    candidate = os.environ.get(lookup, executable) if lookup else executable

    path = shutil.which(candidate)
    if path is None:
        pytest.skip(f"required build tool '{candidate}' is not available")
    return path


def _run_command(command: list[str | Path], *, cwd: Path | None = None) -> None:
    """Run *command* and fail the test when it exits with a non-zero status."""

    result = subprocess.run(
        [str(arg) for arg in command],
        cwd=cwd,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        message = "\n".join(
            [
                "command failed: " + " ".join(str(arg) for arg in command),
                result.stdout,
                result.stderr,
            ]
        )
        pytest.fail(message)


def _write_egl_stub(directory: Path) -> Path:
    """Create a minimal EGL shim used to satisfy dynamic loader symbols."""

    stub_path = directory / "egl_stubs.c"
    stub_path.write_text(
        textwrap.dedent(
            """
            #include <stddef.h>
            #define KHRONOS_STATIC 1
            #include <EGL/egl.h>

            __eglMustCastToProperFunctionPointerType eglGetProcAddress(const char* name)
            {
                (void)name;
                return NULL;
            }

            EGLint eglGetError(void)
            {
                return EGL_SUCCESS;
            }

            EGLDisplay eglGetCurrentDisplay(void)
            {
                return EGL_NO_DISPLAY;
            }

            EGLContext eglGetCurrentContext(void)
            {
                return EGL_NO_CONTEXT;
            }
            """
        ).strip()
        + "\n"
    )
    return stub_path


@dataclass(frozen=True)
class ExampleProgram:
    """Description of an example program that should compile."""

    name: str
    source: Path
    defines: tuple[str, ...]
    platform: str | None = None


EXAMPLE_PROGRAMS: tuple[ExampleProgram, ...] = (
    ExampleProgram(
        name="glxgears",
        source=Path("example/glatter/glxgears.c"),
        defines=(
            "-D_DEFAULT_SOURCE",
            "-DGLATTER_CONFIG_H_DEFINED",
            "-DGLATTER_GL=1",
            "-DGLATTER_GLX=1",
            "-DGLATTER_MESA_GLX_GL=1",
        ),
        platform="linux",
    ),
    ExampleProgram(
        name="eglgears",
        source=Path("example/glatter/eglgears.c"),
        defines=(
            "-DGLATTER_CONFIG_H_DEFINED",
            "-DGLATTER_GL=1",
            "-DGLATTER_EGL=1",
            "-DGLATTER_MESA_EGL_GLES=1",
            "-DGLATTER_EGL_GLES_1_1=1",
        ),
        platform="win32",
    ),
    ExampleProgram(
        name="wglgears",
        source=Path("example/glatter/wglgears.c"),
        defines=(
            "-DGLATTER_CONFIG_H_DEFINED",
            "-DGLATTER_GL=1",
            "-DGLATTER_WGL=1",
            "-DGLATTER_WINDOWS_WGL_GL=1",
        ),
        platform="win32",
    ),
)


def test_c_program_compiles_with_glatter_c(tmp_path: Path) -> None:
    """Verify that a C program builds when linking against glatter.c."""

    cc = _require_tool("cc")

    c_source = tmp_path / "compile_test.c"
    c_source.write_text(
        textwrap.dedent(
            """
            #include <stddef.h>
            #include <glatter/glatter.h>

            static void noop_logger(const char* message) {
                (void)message;
            }

            int main(void) {
                glatter_set_log_handler(noop_logger);
                glatter_set_log_handler(NULL);
                return 0;
            }
            """
        ).strip()
        + "\n"
    )

    output_binary = tmp_path / "c_program"
    egl_stub = _write_egl_stub(tmp_path)
    config_flags = [
        "-DGLATTER_CONFIG_H_DEFINED",
        "-DGLATTER_EGL_GLES2_2_0",
        "-DGLATTER_EGL",
        *_khronos_static_flags(),
    ]
    _run_command(
        [
            cc,
            "-std=c11",
            *config_flags,
            "-I",
            str(REPO_ROOT / "include"),
            "-I",
            str(REPO_ROOT / "tests" / "include"),
            *_thread_flags(),
            str(REPO_ROOT / "src" / "glatter" / "glatter.c"),
            str(egl_stub),
            str(c_source),
            *_dl_flags(),
            *_opengl_libs(),
            "-o",
            str(output_binary),
        ]
    )


def test_header_only_cpp_compiles_across_translation_units(tmp_path: Path) -> None:
    """Ensure the header-only configuration builds in multiple C++ units."""

    cxx = _require_tool("c++")

    sources = {
        "main.cpp": textwrap.dedent(
            """
            #include <glatter/glatter.h>

            int helper();

            static void noop_logger(const char*) {}

            int main() {
                glatter_set_log_handler(noop_logger);
                return helper();
            }
            """
        ).strip()
        + "\n",
        "helper.cpp": textwrap.dedent(
            """
            #include <glatter/glatter.h>

            int helper() {
                return glatter_get_proc_address("glGetString") != nullptr;
            }
            """
        ).strip()
        + "\n",
    }

    for name, content in sources.items():
        (tmp_path / name).write_text(content)

    config_flags = [
        "-DGLATTER_CONFIG_H_DEFINED",
        "-DGLATTER_HEADER_ONLY",
        "-DGLATTER_EGL_GLES2_2_0",
        "-DGLATTER_EGL",
        *_khronos_static_flags(),
    ]

    cc = _require_tool("cc")
    egl_stub = _write_egl_stub(tmp_path)
    stub_object = tmp_path / "egl_stubs.o"
    _run_command(
        [
            cc,
            "-std=c11",
            "-I",
            str(REPO_ROOT / "include"),
            "-I",
            str(REPO_ROOT / "tests" / "include"),
            *_khronos_static_flags(),
            "-c",
            str(egl_stub),
            "-o",
            str(stub_object),
        ]
    )

    compile_args = [
        cxx,
        "-std=c++17",
        *config_flags,
        "-I",
        str(REPO_ROOT / "include"),
        "-I",
        str(REPO_ROOT / "tests" / "include"),
        *_thread_flags(),
    ]

    objects: list[Path] = []
    for source_name in sources:
        object_path = tmp_path / (Path(source_name).stem + ".o")
        _run_command(
            compile_args
            + ["-c", str(tmp_path / source_name), "-o", str(object_path)],
        )
        objects.append(object_path)

    _run_command(
        [
            cxx,
            *_thread_flags(),
            *_dl_flags(),
            *map(str, objects),
            str(stub_object),
            *_opengl_libs(),
            "-o",
            str(tmp_path / "header_only"),
        ]
    )


def test_header_only_cpp_compiles_via_glatter_solo(tmp_path: Path) -> None:
    """Verify that header-only mode works out of the box via glatter_solo.h."""

    cxx = _require_tool("c++")

    sources = {
        "main.cpp": textwrap.dedent(
            """
            #include <glatter/glatter_solo.h>

            int helper();

            static void noop_logger(const char*) {}

            int main() {
                glatter_set_log_handler(noop_logger);
                glatter_set_log_handler(nullptr);
                return helper();
            }
            """
        ).strip()
        + "\n",
        "helper.cpp": textwrap.dedent(
            """
            #include <glatter/glatter_solo.h>

            int helper() {
                return glatter_get_proc_address("glGetString") != nullptr;
            }
            """
        ).strip()
        + "\n",
    }

    for name, content in sources.items():
        (tmp_path / name).write_text(content)

    compile_args = [
        cxx,
        "-std=c++17",
        "-I",
        str(REPO_ROOT / "include"),
        "-I",
        str(REPO_ROOT / "tests" / "include"),
        *_thread_flags(),
    ]

    objects: list[Path] = []
    for source_name in sources:
        object_path = tmp_path / (Path(source_name).stem + ".o")
        _run_command(
            compile_args
            + ["-c", str(tmp_path / source_name), "-o", str(object_path)],
        )
        objects.append(object_path)

    _run_command(
        [
            cxx,
            *_thread_flags(),
            *_dl_flags(),
            *map(str, objects),
            *_opengl_libs(),
            "-o",
            str(tmp_path / "header_only_zeroconfig"),
        ]
    )


def test_header_only_wsi_state_shared_across_tus(tmp_path: Path) -> None:
    """Ensure glatter_set_wsi/glatter_get_wsi share state across TUs."""

    cxx = _require_tool("c++")
    cc = _require_tool("cc")

    sources = {
        "main.cpp": textwrap.dedent(
            """
            #include <glatter/glatter.h>

            int read_wsi();

            int main()
            {
                glatter_set_wsi(GLATTER_WSI_EGL);
                int helper_value = read_wsi();
                int local_value = glatter_get_wsi();
                return helper_value == GLATTER_WSI_EGL &&
                       local_value == GLATTER_WSI_EGL ? 0 : 1;
            }
            """
        ).strip()
        + "\n",
        "helper.cpp": textwrap.dedent(
            """
            #include <glatter/glatter.h>

            int read_wsi()
            {
                return glatter_get_wsi();
            }
            """
        ).strip()
        + "\n",
    }

    for name, content in sources.items():
        (tmp_path / name).write_text(content)

    config_flags = [
        "-DGLATTER_CONFIG_H_DEFINED",
        "-DGLATTER_HEADER_ONLY",
        "-DGLATTER_EGL_GLES2_2_0",
        "-DGLATTER_EGL",
        *_khronos_static_flags(),
    ]

    egl_stub = _write_egl_stub(tmp_path)
    stub_object = tmp_path / "egl_stubs.o"
    _run_command(
        [
            cc,
            "-std=c11",
            "-I",
            str(REPO_ROOT / "include"),
            "-I",
            str(REPO_ROOT / "tests" / "include"),
            *_khronos_static_flags(),
            "-c",
            str(egl_stub),
            "-o",
            str(stub_object),
        ]
    )

    compile_args = [
        cxx,
        "-std=c++17",
        *config_flags,
        "-I",
        str(REPO_ROOT / "include"),
        "-I",
        str(REPO_ROOT / "tests" / "include"),
        *_thread_flags(),
    ]

    objects: list[Path] = []
    for source_name in sources:
        object_path = tmp_path / (Path(source_name).stem + ".o")
        _run_command(
            compile_args + ["-c", str(tmp_path / source_name), "-o", str(object_path)]
        )
        objects.append(object_path)

    binary_path = tmp_path / "shared_wsi"
    _run_command(
        [
            cxx,
            *_thread_flags(),
            *_dl_flags(),
            *map(str, objects),
            str(stub_object),
            *_opengl_libs(),
            "-o",
            str(binary_path),
        ]
    )

    _run_command([binary_path])


def test_cpp_program_links_against_static_library(tmp_path: Path) -> None:
    """Ensure linking succeeds when a consumer uses the compiled C library."""

    cc = _require_tool("cc")
    cxx = _require_tool("c++")
    ar = _require_tool("ar")

    config_flags = [
        "-DGLATTER_CONFIG_H_DEFINED",
        "-DGLATTER_EGL_GLES2_2_0",
        "-DGLATTER_EGL",
        *_khronos_static_flags(),
    ]

    glatter_object = tmp_path / "glatter.o"
    _run_command(
        [
            cc,
            "-std=c11",
            *config_flags,
            "-I",
            str(REPO_ROOT / "include"),
            "-I",
            str(REPO_ROOT / "tests" / "include"),
            "-c",
            str(REPO_ROOT / "src" / "glatter" / "glatter.c"),
            "-o",
            str(glatter_object),
        ]
    )

    static_lib = tmp_path / "libglattertest.a"
    _run_command([ar, "rcs", str(static_lib), str(glatter_object)])

    stub_source = _write_egl_stub(tmp_path)
    stub_object = tmp_path / "egl_stubs.o"
    _run_command(
        [
            cc,
            "-std=c11",
            "-I",
            str(REPO_ROOT / "include"),
            "-I",
            str(REPO_ROOT / "tests" / "include"),
            *_khronos_static_flags(),
            "-c",
            str(stub_source),
            "-o",
            str(stub_object),
        ]
    )

    sources = {
        "main.cpp": textwrap.dedent(
            """
            #include <glatter/glatter.h>

            int helper();

            int main() {
                (void)glatter_get_wsi();
                return helper();
            }
            """
        ).strip()
        + "\n",
        "helper.cpp": textwrap.dedent(
            """
            #include <glatter/glatter.h>

            int helper() {
                return glatter_get_proc_address("glGetString") != nullptr;
            }
            """
        ).strip()
        + "\n",
    }

    object_files: list[Path] = []
    for name, content in sources.items():
        source_path = tmp_path / name
        source_path.write_text(content)
        object_path = tmp_path / (Path(name).stem + ".o")
        _run_command(
            [
                cxx,
                "-std=c++17",
                *config_flags,
                "-I",
                str(REPO_ROOT / "include"),
                "-I",
                str(REPO_ROOT / "tests" / "include"),
                *_thread_flags(),
                "-c",
                str(source_path),
                "-o",
                str(object_path),
            ]
        )
        object_files.append(object_path)

    _run_command(
        [
            cxx,
            *_thread_flags(),
            *_dl_flags(),
            *map(str, object_files),
            str(stub_object),
            str(static_lib),
            *_opengl_libs(),
            "-o",
            str(tmp_path / "linked_consumer"),
        ]
    )


def test_context_key_mixer_behaves_with_stubbed_egl(tmp_path: Path) -> None:
    """Exercise glatter_current_gl_context_key_() under controlled EGL stubs."""

    if os.name == "nt":
        pytest.skip("glatter_current_gl_context_key_ uses WGL on Windows")

    cc = _require_tool("cc")

    source = tmp_path / "context_key_test.c"
    source.write_text(
        textwrap.dedent(
            """
            #include <inttypes.h>
            #include <stdint.h>
            #include <stdio.h>
            #include <EGL/egl.h>
            #include <glatter/glatter.h>

            #undef eglGetCurrentContext
            #undef eglGetCurrentDisplay
            #undef eglGetError
            #undef glGetError

            extern uintptr_t glatter_current_gl_context_key_(void);

            static uintptr_t g_fake_context = (uintptr_t)0;
            static uintptr_t g_fake_display = (uintptr_t)0;

            EGLAPI EGLContext EGLAPIENTRY eglGetCurrentContext(void)
            {
                return (EGLContext)(uintptr_t)g_fake_context;
            }

            EGLAPI EGLDisplay EGLAPIENTRY eglGetCurrentDisplay(void)
            {
                return (EGLDisplay)(uintptr_t)g_fake_display;
            }

            EGLAPI EGLint EGLAPIENTRY eglGetError(void)
            {
                return EGL_SUCCESS;
            }

            unsigned int glGetError(void)
            {
                return 0u;
            }

            int main(void)
            {
                const uintptr_t ctx_value = (uintptr_t)0x13572468u;
                const uintptr_t display_value = (uintptr_t)0xFDB97531u;

                g_fake_context = (uintptr_t)0;
                g_fake_display = (uintptr_t)0;

                if (glatter_current_gl_context_key_() != (uintptr_t)0) {
                    fprintf(stderr, "expected zero key when no context\\n");
                    return 1;
                }

                g_fake_context = ctx_value;
                g_fake_display = display_value;

                const unsigned half = (unsigned)(sizeof(uintptr_t) * 4u);
                const unsigned bits = (unsigned)(sizeof(uintptr_t) * 8u);
                const uintptr_t expected_rotation =
                    (uintptr_t)((display_value << half) | (display_value >> (bits - half)));
                const uintptr_t expected = ctx_value ^ expected_rotation;

                const uintptr_t observed = glatter_current_gl_context_key_();
                if (observed != expected) {
                    fprintf(
                        stderr,
                        "expected key %#" PRIxPTR " but got %#" PRIxPTR "\\n",
                        expected,
                        observed);
                    return 2;
                }

                return 0;
            }
            """
        ).strip()
        + "\n"
    )

    config_flags = [
        "-DGLATTER_CONFIG_H_DEFINED",
        "-DGLATTER_GL=0",
        "-DGLATTER_EGL=1",
        "-DGLATTER_EGL_GLES2_2_0=1",
        *_khronos_static_flags(),
    ]

    output = tmp_path / "context_key_test"
    _run_command(
        [
            cc,
            "-std=c11",
            *config_flags,
            "-I",
            str(REPO_ROOT / "include"),
            "-I",
            str(REPO_ROOT / "tests" / "include"),
            *_thread_flags(),
            str(REPO_ROOT / "src" / "glatter" / "glatter.c"),
            str(source),
            *_dl_flags(),
            *_opengl_libs(),
            "-o",
            str(output),
        ]
    )

    _run_command([output])


def test_wgl_headers_compile_with_stubs(tmp_path: Path) -> None:
    """Verify WGL-enabled builds compile when using stubbed Windows headers."""

    cc = _require_tool("cc")

    source = tmp_path / "wgl_headers.c"
    source.write_text(
        textwrap.dedent(
            """
            #include <stddef.h>
            #include <glatter/glatter.h>

            static void noop(const char* message) {
                (void)message;
            }

            int main(void) {
                glatter_set_log_handler(noop);
                glatter_set_log_handler(NULL);
                return glatter_get_wsi();
            }
            """
        ).strip()
        + "\n"
    )

    config_flags = [
        "-D_WIN32",
        "-DGLATTER_CONFIG_H_DEFINED",
        "-DGLATTER_GL=1",
        "-DGLATTER_WGL=1",
        "-DGLATTER_WINDOWS_WGL_GL=1",
        "-D__STDC_NO_ATOMICS__=1",
    ]

    _run_command(
        [
            cc,
            "-std=c11",
            *config_flags,
            "-I",
            str(REPO_ROOT / "include"),
            "-I",
            str(REPO_ROOT / "tests" / "include"),
            "-c",
            str(source),
            "-o",
            str(tmp_path / "wgl_headers.o"),
        ]
    )


@pytest.mark.parametrize("example", EXAMPLE_PROGRAMS, ids=lambda example: example.name)
def test_examples_compile(example: ExampleProgram, tmp_path: Path) -> None:
    """Compile shipped example programs to ensure they stay buildable."""

    if example.platform is not None:
        if example.platform == "linux" and not sys.platform.startswith("linux"):
            pytest.skip("GLX example only builds on Linux")
        if example.platform == "win32" and os.name != "nt":
            pytest.skip("WGL example only builds on Windows")

    cc = _require_tool("cc")

    output = tmp_path / f"{example.name}.o"
    command = [
        cc,
        "-std=c11",
        *example.defines,
        *_khronos_static_flags(),
        "-I",
        str(REPO_ROOT / "include"),
        "-I",
        str(REPO_ROOT / "tests" / "include"),
        "-c",
        str(REPO_ROOT / example.source),
        "-o",
        str(output),
    ]

    _run_command(command)


def test_windows_egl_gl_compiles_with_glatter_c(tmp_path: Path) -> None:
    """Verify that the Windows EGL+GL platform compiles end-to-end."""

    if os.name != "nt":
        pytest.skip("Windows EGL+GL compile test only runs on Windows")

    cc = _require_tool("cc")

    c_source = tmp_path / "egl_gl_compile.c"
    c_source.write_text(
        textwrap.dedent(
            """
            #include <stddef.h>
            #include <glatter/glatter.h>

            static void noop_logger(const char* message) {
                (void)message;
            }

            int main(void) {
                glatter_set_log_handler(noop_logger);
                glatter_set_log_handler(NULL);
                return 0;
            }
            """
        ).strip()
        + "\n"
    )

    egl_stub = _write_egl_stub(tmp_path)
    config_flags = [
        "-DGLATTER_CONFIG_H_DEFINED",
        "-DGLATTER_GL=1",
        "-DGLATTER_EGL=1",
        "-DGLATTER_WINDOWS_EGL_GL=1",
        *_khronos_static_flags(),
    ]

    output_binary = tmp_path / "egl_gl_program"
    _run_command(
        [
            cc,
            "-std=c11",
            *config_flags,
            "-I",
            str(REPO_ROOT / "include"),
            "-I",
            str(REPO_ROOT / "tests" / "include"),
            str(REPO_ROOT / "src" / "glatter" / "glatter.c"),
            str(egl_stub),
            str(c_source),
            *_opengl_libs(),
            "-o",
            str(output_binary),
        ]
    )
