# Tasks: Module 2 - The Digital Twin (Gazebo & Unity)

**Input**: Design documents from `/specs/002-module2-digital-twin/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md

**Tests**: Testing for this feature will be done via manual execution and visual verification of the simulation examples. Automated tests are not required.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create the directory structure for the new module's content and code.

- [x] T001 [P] Create documentation directory `docs/module-02-simulation/`
- [x] T002 [P] Create code examples directory `code-examples/module2-the-digital-twin/`
- [x] T003 [P] Add category metadata file `docs/module-02-simulation/_category_.json`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Create the baseline assets that all user stories will build upon.

- [x] T004 Create the base humanoid model file `code-examples/module2-the-digital-twin/shared/humanoid_base.urdf.xacro`
- [x] T005 Write the main module overview page `docs/module-02-simulation/index.mdx`

**Checkpoint**: Foundation ready - user story implementation can now begin.

---

## Phase 3: User Story 1 - Gazebo Physics Simulation (Priority: P1) 🎯 MVP

**Goal**: Create the content and code for Chapter 2.1, teaching students to simulate a humanoid in Gazebo.

**Independent Test**: A student can run the `spawn_humanoid.launch.py` file and see the humanoid model load and move in a Gazebo world.

### Implementation for User Story 1

- [x] T006 [US1] Write the full content for Chapter 2.1 in `docs/module-02-simulation/chapter2.1.mdx`
- [x] T007 [P] [US1] Create the simple Gazebo world file `code-examples/module2-the-digital-twin/chapter2.1/worlds/empty.sdf`
- [x] T008 [P] [US1] Create the specific URDF for this chapter `code-examples/module2-the-digital-twin/chapter2.1/urdf/humanoid_ch2.1.urdf.xacro` by including the base from T004.
- [x] T009 [US1] Create the ROS 2 launch file `code-examples/module2-the-digital-twin/chapter2.1/launch/spawn_humanoid.launch.py`
- [x] T010 [US1] Write the Python script to apply forces to joints `code-examples/module2-the-digital-twin/chapter2.1/scripts/joint_controller.py`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently.

---

## Phase 4: User Story 2 - Unity High-Fidelity Rendering (Priority: P2)

**Goal**: Create the content and code for Chapter 2.2, teaching students to control a humanoid in Unity from ROS 2.

**Independent Test**: A student can run the Unity scene, see the robot, and command its joints by running the `unity_joint_commander.py` script.

### Implementation for User Story 2

- [x] T011 [US2] Write the full content for Chapter 2.2 in `docs/module-02-simulation/chapter2.2.mdx`
- [x] T012 [P] [US2] Create the Unity Project and Scene structure under `code-examples/module2-the-digital-twin/chapter2.2/unity_project/` (Placeholder for Unity Editor work)
- [x] T013 [P] [US2] Implement the Unity C# scripts for ROS connection and robot control within the project from T012.
- [x] T014 [US2] Write the ROS 2 Python script to send joint commands to Unity `code-examples/module2-the-digital-twin/chapter2.2/ros2_scripts/unity_joint_commander.py`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently.

---

## Phase 5: User Story 3 - Sensor Simulation (Priority: P3)

**Goal**: Create the content and code for Chapter 2.3, teaching students to add and process simulated sensor data.

**Independent Test**: A student can launch the sensor-equipped robot, visualize the sensor data in RViz2, and see processed data logged by the `sensor_processor.py` script.

### Implementation for User Story 3

- [x] T015 [US3] Write the full content for Chapter 2.3 in `docs/module-02-simulation/chapter2.3.mdx`
- [x] T016 [P] [US3] Create the sensor-enabled URDF `code-examples/module2-the-digital-twin/chapter2.3/urdf/humanoid_sensors.urdf.xacro` by extending the base URDF.
- [x] T017 [US3] Create the ROS 2 launch file `code-examples/module2-the-digital-twin/chapter2.3/launch/spawn_humanoid_with_sensors.launch.py`
- [x] T018 [US3] Write the Python script to subscribe to and process sensor data `code-examples/module2-the-digital-twin/chapter2.3/scripts/sensor_processor.py`

**Checkpoint**: All user stories should now be independently functional.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Finalize the module with assessment materials and a final review.

- [x] T019 [P] Write the hands-on project description `docs/module-02-simulation/project.mdx`
- [x] T020 [P] Write the module assessment content `docs/module-02-simulation/assessment.mdx`
- [x] T021 [P] Create the assessment code challenge files in `code-examples/module2-the-digital-twin/assessment/`
- [x] T022 Review all content and code examples for clarity, accuracy, and consistency.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: Can start immediately.
- **Foundational (Phase 2)**: Depends on Setup completion.
- **User Stories (Phases 3-5)**: All depend on Foundational phase completion.
- **Polish (Phase 6)**: Depends on all User Story phases being complete.

### User Story Dependencies

- **User Story 1 (P1)**: Depends on Foundational (T004).
- **User Story 2 (P2)**: Depends on Foundational (T004). It is independent of US1.
- **User Story 3 (P3)**: Depends on Foundational (T004). It can be seen as an extension of US1 but can be worked on independently.

### Parallel Opportunities

- Once Phase 2 is complete, work on all three user stories (Phase 3, 4, 5) can begin in parallel.
- Within each story, content writing (e.g., T006) can happen in parallel with code creation (e.g., T007, T008).

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test the Gazebo simulation (T009) and ensure it's a functional, standalone learning experience.

### Incremental Delivery

1. Complete MVP (US1).
2. Add User Story 2 (Unity). Test independently.
3. Add User Story 3 (Sensors). Test independently.
4. Complete Phase 6 (Polish & Assessment).
5. Each chapter and its code examples should function as a standalone deliverable.
