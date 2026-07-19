# Codex Sol 5.6 Ultra: Cooperative Agent Directives & Technical Specification

**Execution Environment:** OpenAI Codex Sol 5.6 Ultra
**Workflow Mode:** Cooperative Subagent Architecture (Ultra Mode)

---

## 1. Project Objective & Architecture

The objective is to architect, implement, and validate a highly portable, highly advanced 3D physics engine and companion editor.

- **Core Engine:** Must be purely headless, hardware-agnostic, written in C++17, and capable of running natively on both microcontrollers (specifically targeting ARM Cortex-M4/M33, e.g., STM32G474) and desktop environments.
- **Visualizer:** A powerful desktop companion editor built around the headless core, utilizing Vulkan and Dear ImGui.

**Agent Deployment Protocol:** Codex Sol must deploy specialized subagents (Physics Architect, Vulkan Rendering Engineer, Control Systems Expert, and Integration Lead) to autonomously collaborate, generate, and mathematically validate each phase of this specification.

---

## 2. Phase 1: Core Physics Engine (Headless & Portable)

**Assigned Subagent:** Core Physics & Aerodynamics Specialist

1. **Integration Scheme:** Implement a strict **Velocity Verlet** integration scheme for all rigid body dynamics to maintain stability over time.
2. **Physical Properties:** Accurately model static/kinetic friction, Center of Mass (CoM), Center of Pressure (CoP), gravity, and precise collision resolution using impulse-based solving.
3. **Advanced Aerodynamics (The Leaf Effect):**
   - Wind must be modeled as a dynamic 3D vector field (e.g., using curl noise).
   - Area of resistance must be calculated dynamically based on the object's 3D bounding box relative to its velocity vector and the wind field.
   - **Crucial Behavior:** A flat plane falling face-down must experience high drag and turbulence, fluttering like a leaf. If it falls edge-first, it must slice through the air and drop straight down. Edge-catching turbulence should organically generate lift.
4. **Entities:** Natively support drones (quadcopters), rovers, and standard primitives (blocks, planes, spheres).

---

## 3. Phase 2: Drone Dynamics, Control Systems & HIL

**Assigned Subagent:** Aerospace, Embedded Systems, & Control Engineer

1. **Flight Dynamics:** Model standard quadcopter physics (thrust, torque, drag, inertia tensor).
2. **Internal Controller:** Implement a built-in PID controller for basic stabilization, controllable via `WASD` (pitch/roll) and dedicated throttle/yaw keys. PID values must be exposed for runtime editing.
3. **Hardware-In-the-Loop (HIL) Bridge:**
   - Implement a distinct HIL mode that ignores keyboard input and internal PID stabilization.
   - **API & Protocol:** The HIL API must perfectly mirror a UDP-serial pipeline. It should expect a custom binary protocol with CRC-32 checksums to interface seamlessly with external microcontrollers.
   - **Sensor Emulation:** The engine must emulate an ICM-42688 IMU, outputting data at a configurable rate (target 1 kHz). Crucially, the API must allow the injection of realistic sensor noise, bias, and drift to validate the robustness of external error-state EKF estimators.

---

## 4. Phase 3: Desktop Rendering Engine

**Assigned Subagent:** Graphics & Vulkan Engineer

1. **Renderer:** Build the visualizer natively in **Vulkan**.
2. **Lighting & Shadows:** Implement highly optimized Cascaded Shadow Maps (CSM) to ensure crisp shadows for flight simulation at varying altitudes.
3. **Decoupling:** The renderer must remain strictly decoupled from the physics engine to preserve the C++17 embedded portability of the core.

---

## 5. Phase 4: User Interface & Editor

**Assigned Subagent:** UI/UX & Tooling Engineer

1. **Framework:** Utilize **Dear ImGui**.
2. **UI Elements:**
   - **Menu Bar:** Standard controls (File, Edit, Engine Settings, Wind Vector Field toggles).
   - **Spawner Menu:** Seamless spawning of drones, rovers, and primitives.
   - **Properties Inspector:** Real-time editing of mass, dimensions, PID values, and friction coefficients for the selected object.

---

## 6. Phase 5: Interactivity & Camera Controls

**Assigned Subagent:** Interaction & Input Developer

1. **Free Camera:** Holding `RMB` + `WASD` moves the scene camera smoothly through 3D space.
2. **BeamNG-Style Vertex Manipulation:**
   - Holding `Ctrl` must illuminate/highlight every vertex within a spherical radius (sphere of influence) around the cursor/camera in 3D space.
   - Holding `Ctrl` + `LMB` must allow the user to grab and pull these vertices.
   - This action must apply a localized spring-like force or soft-body pulling effect to the rigid body solver, allowing the user to physically drag drones or primitives dynamically.

---

## 7. Validation & Output Requirements

**Assigned Subagent:** Lead QA & Integration Manager

- **Execution Constraint:** Generate structural code, followed immediately by validation scripts or mathematical proofs for the physics model before proceeding to the next phase.
- **Deliverables:** Provide the final architecture, directory structure, core header files, the UDP-serial HIL bridge implementation, bounding-box aerodynamics logic, Vulkan CSM setup, and the soft-body `Ctrl`-pull interaction logic.
