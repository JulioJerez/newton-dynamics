set(GLATTER_TEST_ROOT "${PROJECT_SOURCE_DIR}")
set(GLATTER_TEST_INCLUDES
    "${GLATTER_TEST_ROOT}/include"
    "${GLATTER_TEST_ROOT}/tests/include"
)

find_package(Threads REQUIRED)

add_library(glatter_egl_stubs STATIC egl_stubs.c)
target_include_directories(glatter_egl_stubs PUBLIC ${GLATTER_TEST_INCLUDES})

add_executable(glatter_c_smoketest c_program.c "${GLATTER_TEST_ROOT}/src/glatter/glatter.c")
target_include_directories(glatter_c_smoketest PRIVATE ${GLATTER_TEST_INCLUDES})
target_compile_definitions(glatter_c_smoketest PRIVATE
    GLATTER_CONFIG_H_DEFINED
    GLATTER_EGL_GLES2_2_0
    GLATTER_EGL
)
target_link_libraries(glatter_c_smoketest PRIVATE
    glatter_egl_stubs
    Threads::Threads
    ${CMAKE_DL_LIBS}
)
add_test(NAME glatter.c_smoketest COMMAND glatter_c_smoketest)

add_executable(glatter_header_only
    header_only_main.cpp
    header_only_helper.cpp
)
target_include_directories(glatter_header_only PRIVATE ${GLATTER_TEST_INCLUDES})
target_compile_definitions(glatter_header_only PRIVATE
    GLATTER_CONFIG_H_DEFINED
    GLATTER_HEADER_ONLY
    GLATTER_EGL_GLES2_2_0
    GLATTER_EGL
)
target_link_libraries(glatter_header_only PRIVATE
    glatter_egl_stubs
    Threads::Threads
    ${CMAKE_DL_LIBS}
)
add_test(NAME glatter.header_only COMMAND glatter_header_only)

add_executable(glatter_header_only_solo
    header_only_solo_main.cpp
    header_only_solo_helper.cpp
)
target_include_directories(glatter_header_only_solo PRIVATE ${GLATTER_TEST_INCLUDES})
target_link_libraries(glatter_header_only_solo PRIVATE
    Threads::Threads
    ${CMAKE_DL_LIBS}
)
add_test(NAME glatter.header_only_solo COMMAND glatter_header_only_solo)

add_library(glatter_test_static STATIC "${GLATTER_TEST_ROOT}/src/glatter/glatter.c")
target_include_directories(glatter_test_static PUBLIC ${GLATTER_TEST_INCLUDES})
target_compile_definitions(glatter_test_static PUBLIC
    GLATTER_CONFIG_H_DEFINED
    GLATTER_EGL_GLES2_2_0
    GLATTER_EGL
)
target_link_libraries(glatter_test_static PUBLIC
    Threads::Threads
    ${CMAKE_DL_LIBS}
)

add_executable(glatter_static_link
    static_link_main.cpp
    static_link_helper.cpp
)
target_include_directories(glatter_static_link PRIVATE ${GLATTER_TEST_INCLUDES})
target_compile_definitions(glatter_static_link PRIVATE
    GLATTER_CONFIG_H_DEFINED
    GLATTER_EGL_GLES2_2_0
    GLATTER_EGL
)
target_link_libraries(glatter_static_link PRIVATE
    glatter_test_static
    glatter_egl_stubs
)
add_test(NAME glatter.static_link COMMAND glatter_static_link)

add_executable(glatter_context_key
    context_key_test.c
    "${GLATTER_TEST_ROOT}/src/glatter/glatter.c"
)
target_include_directories(glatter_context_key PRIVATE ${GLATTER_TEST_INCLUDES})
target_compile_definitions(glatter_context_key PRIVATE
    GLATTER_CONFIG_H_DEFINED
    GLATTER_GL=0
    GLATTER_EGL=1
    GLATTER_EGL_GLES2_2_0=1
)
target_link_libraries(glatter_context_key PRIVATE
    Threads::Threads
    ${CMAKE_DL_LIBS}
)
add_test(NAME glatter.context_key COMMAND glatter_context_key)

add_library(glatter_wgl_headers OBJECT wgl_headers.c)
target_include_directories(glatter_wgl_headers PRIVATE ${GLATTER_TEST_INCLUDES})
target_compile_definitions(glatter_wgl_headers PRIVATE
    _WIN32
    GLATTER_CONFIG_H_DEFINED
    GLATTER_GL=1
    GLATTER_WGL=1
    GLATTER_WINDOWS_WGL_GL=1
    __STDC_NO_ATOMICS__=1
)
add_custom_target(glatter.build_wgl_headers ALL DEPENDS glatter_wgl_headers)
