cmake_minimum_required(VERSION 3.24)

project(
    UAViewStudio
    VERSION 0.1.0
    DESCRIPTION "UAView Studio - Vulkan visualization and simulation editor"
    LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

option(UAVIEW_BUILD_STUDIO "Build the Vulkan desktop editor" ON)
option(UAVIEW_ENABLE_VALIDATION "Enable Vulkan validation in supported builds" ON)
option(UAVIEW_BUILD_TESTS "Build UAView Studio tests" ON)

add_library(uaview_physics STATIC
    src/physics/Drone.cpp
    src/physics/Environment.cpp
    src/physics/RigidBody.cpp
    src/physics/World.cpp
)
target_include_directories(uaview_physics
    PUBLIC
        "${CMAKE_SOURCE_DIR}/include"
)
target_compile_features(uaview_physics PUBLIC cxx_std_17)

if(UAVIEW_BUILD_STUDIO)
file(
    STRINGS
    "${CMAKE_SOURCE_DIR}/config/vulkan-sdk-version.txt"
    UAVIEW_VULKAN_SDK_VERSION
    LIMIT_COUNT 1
)
string(STRIP "${UAVIEW_VULKAN_SDK_VERSION}" UAVIEW_VULKAN_SDK_VERSION)
set(
    UAVIEW_VULKAN_SDK
    "${CMAKE_SOURCE_DIR}/.tools/VulkanSDK/${UAVIEW_VULKAN_SDK_VERSION}"
    CACHE PATH
    "Path to a Vulkan SDK installation"
)

if(WIN32 AND EXISTS "${UAVIEW_VULKAN_SDK}/Include/vulkan/vulkan.h")
    file(TO_NATIVE_PATH "${UAVIEW_VULKAN_SDK}" UAVIEW_VULKAN_SDK_NATIVE)
    set(ENV{VULKAN_SDK} "${UAVIEW_VULKAN_SDK_NATIVE}")
endif()

find_package(Vulkan 1.3 REQUIRED COMPONENTS glslc)

include(cmake/Dependencies.cmake)
include(cmake/CompileShaders.cmake)

add_library(uaview_studio_math STATIC
    src/render/Camera.cpp
)
target_include_directories(uaview_studio_math
    PUBLIC
        "${CMAKE_SOURCE_DIR}/include"
)
target_link_libraries(uaview_studio_math
    PUBLIC
        glm::glm
)
target_compile_features(uaview_studio_math PUBLIC cxx_std_17)

add_library(uaview_renderer STATIC
    src/render/VulkanRenderer.cpp
    src/render/StbImage.cpp
)
target_include_directories(uaview_renderer
    PUBLIC
        "${CMAKE_SOURCE_DIR}/include"
)
target_link_libraries(uaview_renderer
    PUBLIC
        uaview_studio_math
    PRIVATE
        uaview_imgui
        Vulkan::Vulkan
        vk-bootstrap::vk-bootstrap
        GPUOpen::VulkanMemoryAllocator
        glfw
        glm::glm
    PRIVATE
        uaview_stb
)
target_compile_definitions(uaview_renderer
    PRIVATE
        UAVIEW_VULKAN_SDK_DIR="${UAVIEW_VULKAN_SDK}"
        $<$<BOOL:${UAVIEW_ENABLE_VALIDATION}>:UAVIEW_ENABLE_VALIDATION=1>
)
target_compile_features(uaview_renderer PUBLIC cxx_std_17)

add_library(uaview_editor STATIC
    src/app/StudioApplication.cpp
)
target_include_directories(uaview_editor
    PUBLIC
        "${CMAKE_SOURCE_DIR}/include"
)
target_link_libraries(uaview_editor
    PRIVATE
        uaview_physics
        uaview_renderer
        uaview_imgui
        glfw
)
target_compile_features(uaview_editor PUBLIC cxx_std_17)

add_executable(UAViewStudio WIN32
    src/main.cpp
)
target_link_libraries(UAViewStudio PRIVATE uaview_editor)
set_target_properties(UAViewStudio PROPERTIES
    OUTPUT_NAME "UAView Studio"
    VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:UAViewStudio>"
)

if(MSVC)
    foreach(target uaview_studio_math uaview_renderer uaview_editor UAViewStudio)
        target_compile_options(${target} PRIVATE /W4 /permissive- /Zc:__cplusplus)
    endforeach()
    target_compile_definitions(UAViewStudio PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
    foreach(target uaview_studio_math uaview_renderer uaview_editor UAViewStudio)
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endforeach()
endif()

set(UAVIEW_RUNTIME_ROOT "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$<CONFIG>")
set(UAVIEW_RUNTIME_ASSET_DIR "${UAVIEW_RUNTIME_ROOT}/assets")
set(UAVIEW_RUNTIME_SHADER_DIR "${UAVIEW_RUNTIME_ROOT}/shaders")
set(UAVIEW_RUNTIME_ASSET_SOURCES
    "${CMAKE_SOURCE_DIR}/assets/textures/concrete/concrete_pavement_diff_2k.jpg"
    "${CMAKE_SOURCE_DIR}/assets/textures/concrete/concrete_pavement_nor_gl_2k.jpg"
    "${CMAKE_SOURCE_DIR}/assets/textures/concrete/concrete_pavement_rough_2k.jpg"
    "${CMAKE_SOURCE_DIR}/assets/textures/concrete/concrete_pavement_ao_2k.jpg"
)
set(UAVIEW_RUNTIME_FONT_SOURCES
    "${CMAKE_SOURCE_DIR}/assets/fonts/InterVariable.ttf"
    "${CMAKE_SOURCE_DIR}/assets/fonts/Inter-OFL.txt"
)
set(UAVIEW_RUNTIME_ENVIRONMENT_SOURCES
    "${CMAKE_SOURCE_DIR}/assets/textures/environment/kloofendal_48d_partly_cloudy_puresky_4k.hdr"
)
add_custom_target(uaview_runtime_files ALL
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${UAVIEW_RUNTIME_ASSET_DIR}/textures/concrete"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${UAVIEW_RUNTIME_ASSET_DIR}/textures/environment"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${UAVIEW_RUNTIME_ASSET_DIR}/fonts"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${UAVIEW_RUNTIME_SHADER_DIR}"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        ${UAVIEW_RUNTIME_ASSET_SOURCES}
        "${UAVIEW_RUNTIME_ASSET_DIR}/textures/concrete"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        ${UAVIEW_RUNTIME_FONT_SOURCES}
        "${UAVIEW_RUNTIME_ASSET_DIR}/fonts"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        ${UAVIEW_RUNTIME_ENVIRONMENT_SOURCES}
        "${UAVIEW_RUNTIME_ASSET_DIR}/textures/environment"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        ${UAVIEW_SHADER_BINARIES}
        "${UAVIEW_RUNTIME_SHADER_DIR}"
    DEPENDS
        ${UAVIEW_RUNTIME_ASSET_SOURCES}
        ${UAVIEW_RUNTIME_FONT_SOURCES}
        ${UAVIEW_RUNTIME_ENVIRONMENT_SOURCES}
        ${UAVIEW_SHADER_BINARIES}
    VERBATIM
)
add_dependencies(uaview_runtime_files uaview_shaders)
add_dependencies(UAViewStudio uaview_runtime_files)
endif()

if(UAVIEW_BUILD_TESTS)
    enable_testing()

    add_executable(uaview_physics_tests tests/PhysicsTests.cpp)
    target_link_libraries(uaview_physics_tests PRIVATE uaview_physics)
    target_compile_features(uaview_physics_tests PRIVATE cxx_std_17)
    add_test(NAME physics_core COMMAND uaview_physics_tests)

    add_executable(uaview_mass_properties_tests tests/MassPropertiesTests.cpp)
    target_link_libraries(uaview_mass_properties_tests PRIVATE uaview_physics)
    target_compile_features(uaview_mass_properties_tests PRIVATE cxx_std_17)
    add_test(NAME mass_properties COMMAND uaview_mass_properties_tests)

    add_executable(
        uaview_rotational_dynamics_tests
        tests/RotationalDynamicsTests.cpp
    )
    target_link_libraries(
        uaview_rotational_dynamics_tests
        PRIVATE uaview_physics
    )
    target_compile_features(
        uaview_rotational_dynamics_tests
        PRIVATE cxx_std_17
    )
    add_test(
        NAME rotational_dynamics
        COMMAND uaview_rotational_dynamics_tests
    )

    add_executable(
        uaview_gyroscopic_rotor_tests
        tests/GyroscopicRotorTests.cpp
    )
    target_link_libraries(
        uaview_gyroscopic_rotor_tests
        PRIVATE uaview_physics
    )
    target_compile_features(
        uaview_gyroscopic_rotor_tests
        PRIVATE cxx_std_17
    )
    add_test(
        NAME gyroscopic_rotor
        COMMAND uaview_gyroscopic_rotor_tests
    )

    add_executable(
        uaview_corner_impact_tests
        tests/CornerImpactTests.cpp
    )
    target_link_libraries(
        uaview_corner_impact_tests
        PRIVATE uaview_physics
    )
    target_compile_features(
        uaview_corner_impact_tests
        PRIVATE cxx_std_17
    )
    add_test(
        NAME corner_impact
        COMMAND uaview_corner_impact_tests
    )

    add_executable(uaview_drone_dynamics_tests tests/DroneDynamicsTests.cpp)
    target_link_libraries(uaview_drone_dynamics_tests PRIVATE uaview_physics)
    target_compile_features(uaview_drone_dynamics_tests PRIVATE cxx_std_17)
    add_test(NAME drone_dynamics COMMAND uaview_drone_dynamics_tests)

    if(UAVIEW_BUILD_STUDIO)
        add_executable(uaview_camera_tests tests/CameraTests.cpp)
        target_link_libraries(uaview_camera_tests PRIVATE uaview_studio_math)
        target_compile_features(uaview_camera_tests PRIVATE cxx_std_17)
        add_test(NAME camera_math COMMAND uaview_camera_tests)
    endif()
endif()

if(MSVC)
    target_compile_options(
        uaview_physics
        PRIVATE /W4 /permissive- /Zc:__cplusplus /EHsc
    )
    if(TARGET uaview_physics_tests)
        target_compile_options(
            uaview_physics_tests
            PRIVATE /W4 /permissive- /Zc:__cplusplus /EHsc
        )
    endif()
    if(TARGET uaview_mass_properties_tests)
        target_compile_options(
            uaview_mass_properties_tests
            PRIVATE /W4 /permissive- /Zc:__cplusplus /EHsc
        )
    endif()
    if(TARGET uaview_rotational_dynamics_tests)
        target_compile_options(
            uaview_rotational_dynamics_tests
            PRIVATE /W4 /permissive- /Zc:__cplusplus /EHsc
        )
    endif()
    if(TARGET uaview_gyroscopic_rotor_tests)
        target_compile_options(
            uaview_gyroscopic_rotor_tests
            PRIVATE /W4 /permissive- /Zc:__cplusplus /EHsc
        )
    endif()
    if(TARGET uaview_corner_impact_tests)
        target_compile_options(
            uaview_corner_impact_tests
            PRIVATE /W4 /permissive- /Zc:__cplusplus /EHsc
        )
    endif()
    if(TARGET uaview_drone_dynamics_tests)
        target_compile_options(
            uaview_drone_dynamics_tests
            PRIVATE /W4 /permissive- /Zc:__cplusplus /EHsc
        )
    endif()
    if(TARGET uaview_camera_tests)
        target_compile_options(
            uaview_camera_tests
            PRIVATE /W4 /permissive- /Zc:__cplusplus
        )
    endif()
else()
    target_compile_options(uaview_physics PRIVATE -Wall -Wextra -Wpedantic)
    if(TARGET uaview_physics_tests)
        target_compile_options(
            uaview_physics_tests
            PRIVATE -Wall -Wextra -Wpedantic
        )
    endif()
    if(TARGET uaview_mass_properties_tests)
        target_compile_options(
            uaview_mass_properties_tests
            PRIVATE -Wall -Wextra -Wpedantic
        )
    endif()
    if(TARGET uaview_rotational_dynamics_tests)
        target_compile_options(
            uaview_rotational_dynamics_tests
            PRIVATE -Wall -Wextra -Wpedantic
        )
    endif()
    if(TARGET uaview_gyroscopic_rotor_tests)
        target_compile_options(
            uaview_gyroscopic_rotor_tests
            PRIVATE -Wall -Wextra -Wpedantic
        )
    endif()
    if(TARGET uaview_corner_impact_tests)
        target_compile_options(
            uaview_corner_impact_tests
            PRIVATE -Wall -Wextra -Wpedantic
        )
    endif()
    if(TARGET uaview_drone_dynamics_tests)
        target_compile_options(
            uaview_drone_dynamics_tests
            PRIVATE -Wall -Wextra -Wpedantic
        )
    endif()
    if(TARGET uaview_camera_tests)
        target_compile_options(
            uaview_camera_tests
            PRIVATE -Wall -Wextra -Wpedantic
        )
    endif()
endif()
