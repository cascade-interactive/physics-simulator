set(UAVIEW_SHADER_OUTPUT_DIR "${CMAKE_BINARY_DIR}/shaders")
file(MAKE_DIRECTORY "${UAVIEW_SHADER_OUTPUT_DIR}")

set(UAVIEW_SHADER_SOURCES
    "${CMAKE_SOURCE_DIR}/shaders/ground.vert"
    "${CMAKE_SOURCE_DIR}/shaders/ground.frag"
    "${CMAKE_SOURCE_DIR}/shaders/skybox.vert"
    "${CMAKE_SOURCE_DIR}/shaders/skybox.frag"
    "${CMAKE_SOURCE_DIR}/shaders/shadow.vert"
    "${CMAKE_SOURCE_DIR}/shaders/debug_lines.vert"
    "${CMAKE_SOURCE_DIR}/shaders/debug_lines.frag"
)

set(UAVIEW_SHADER_BINARIES)
foreach(shader IN LISTS UAVIEW_SHADER_SOURCES)
    get_filename_component(shader_name "${shader}" NAME)
    set(shader_output "${UAVIEW_SHADER_OUTPUT_DIR}/${shader_name}.spv")
    add_custom_command(
        OUTPUT "${shader_output}"
        COMMAND Vulkan::glslc
            --target-env=vulkan1.3
            -Werror
            -o "${shader_output}"
            "${shader}"
        DEPENDS "${shader}"
        COMMENT "Compiling ${shader_name} to SPIR-V"
        VERBATIM
    )
    list(APPEND UAVIEW_SHADER_BINARIES "${shader_output}")
endforeach()

add_custom_target(uaview_shaders ALL DEPENDS ${UAVIEW_SHADER_BINARIES})
