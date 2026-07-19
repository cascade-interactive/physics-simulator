# UAView Studio repository guidance

Read `docs/DRONE_PHYSICS_ENGINE_SPEC.md` before changing architecture or adding a feature.

The current approved scope is **Milestone 2: Headless Physics & Environment**:

- C++17 desktop application named **UAView Studio**.
- Native Vulkan renderer using GLFW, vk-bootstrap, Vulkan Memory Allocator, and Dear ImGui docking.
- Physically based rendering of the supplied concrete ground material.
- A renderer-independent rigid-body engine with Velocity Verlet integration,
  impulse contacts, static/dynamic friction, restitution, aerodynamic drag,
  angular drag, gravity, sleeping, and fixed-step/substep controls.
- A constant atmosphere backed by a subtle deterministic 3D wind vector field;
  keep the atmosphere interface ready for altitude-dependent models later.
- A real 1 m cube, environment/contact diagnostics, and Ctrl + LMB spring-force
  manipulation through a renderer/editor adapter.
- No drone dynamics, PID/controller, sensor emulation, HIL bridge, CSM, rover,
  or soft-body implementation until the user approves a later milestone.

Keep the headless simulation library independent of Vulkan, GLFW, ImGui,
desktop operating-system APIs, and editor utilities so it can later target ARM
Cortex-M4/M33 hardware.

For every stage:

1. Keep dependencies pinned and driven by CMake.
2. Compile GLSL to SPIR-V as part of the build.
3. Build and run the focused tests before expanding scope.
4. Treat validation-layer errors as failures.
