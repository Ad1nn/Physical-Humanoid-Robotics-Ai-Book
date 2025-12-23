# Tasks: Module 1: The Robotic Nervous System (ROS 2)

**Input**: Design documents from `specs/001-module1-ros2-nervous-system/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: The specification implies tests through "Expected terminal output," "Testing and validation checklist," and "practical coding challenges" in the assessment. Therefore, test-related tasks will be included where appropriate.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story?] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- Docusaurus content: `docs/module1/`
- Code examples: `code-examples/module1-ros2-nervous-system/`
- Spec/Plan/Tasks: `specs/001-module1-ros2-nervous-system/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for the module's content and code examples.

- [x] T001 Create Docusaurus content directory for Module 1 at `docs/module1/`
- [x] T002 Create code examples root directory for Module 1 at `code-examples/module1-ros2-nervous-system/`
- [x] T003 [P] Configure initial `docusaurus.config.js` entry for `docs/module1/`
- [x] T004 [P] Create `requirements.txt` for common Python dependencies in `code-examples/module1-ros2-nervous-system/`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure and content that MUST be complete before ANY user story content can be implemented.

- [x] T005 Write content for Module Overview and Prerequisites in `docs/module1/index.mdx`
- [x] T006 [P] Outline installation guide for ROS 2 Humble Hawksbill and Python 3.10+ in `docs/module1/setup.mdx`
- [x] T007 [P] Prepare Docker setup for Ubuntu 22.04 LTS development environment in `code-examples/module1-ros2-nervous-system/Dockerfile`

---

## Phase 3: User Story 1 - Understand ROS 2 Fundamentals (Priority: P1) 🎯 MVP

**Goal**: Student understands ROS 2 basics and sets up development environment.

**Independent Test**: Student can run "Hello World" node and create a custom node.

### Implementation for User Story 1 (Chapter 1.1: ROS 2 Fundamentals)

- [x] T008 [US1] Write content for "What is ROS 2 and why it exists" in `docs/module1/chapter1.1.mdx`
- [x] T009 [US1] Write content for "ROS 2 vs ROS 1 key differences" in `docs/module1/chapter1.1.mdx`
- [x] T010 [US1] Write content for "DDS middleware architecture" in `docs/module1/chapter1.1.mdx`
- [x] T011 [US1] Write content for "Installation and workspace setup" in `docs/module1/chapter1.1.mdx` (referencing `docs/module1/setup.mdx`)
- [x] T012 [US1] Write content for "Your first 'Hello World' ROS 2 node" in `docs/module1/chapter1.1.mdx`
- [x] T013 [US1] Implement "Minimal Python publisher/subscriber" code example in `code-examples/module1-ros2-nervous-system/chapter1.1/minimal_pub_sub.py`
- [x] T014 [US1] Add expected outputs and debugging tips for T013 in `docs/module1/chapter1.1.mdx`
- [x] T015 [US1] Describe "Create custom hello node" exercise in `docs/module1/chapter1.1.mdx`

**Checkpoint**: At this point, Chapter 1.1 content and code examples are ready.

---

## Phase 4: User Story 2 - Implement ROS 2 Communication (Priority: P1)

**Goal**: Student implements inter-node communication using topics and services.

**Independent Test**: Student can build a multi-node system and verify data flow.

### Implementation for User Story 2 (Chapter 1.2: Nodes, Topics & Services)

- [x] T016 [US2] Write content for "Deep dive into ROS 2 communication patterns" in `docs/module1/chapter1.2.mdx`
- [x] T017 [US2] Write content for "Nodes: autonomous processes and lifecycle" in `docs/module1/chapter1.2.mdx`
- [x] T018 [US2] Write content for "Topics: publish/subscribe pattern" in `docs/module1/chapter1.2.mdx`
- [x] T019 [US2] Write content for "Services: request/response pattern" in `docs/module1/chapter1.2.mdx`
- [x] T020 [US2] Write content for "Quality of Service (QoS) settings" in `docs/module1/chapter1.2.mdx`
- [x] T021 [US2] Implement "Multi-node system with topics and services" code example in `code-examples/module1-ros2-nervous-system/chapter1.2/multi_node_system.py`
- [x] T022 [US2] Implement "Camera feed publisher, image processor subscriber" code example in `code-examples/module1-ros2-nervous-system/chapter1.2/camera_pipeline.py`
- [x] T023 [US2] Describe "Build sensor data pipeline" exercise in `docs/module1/chapter1.2.mdx`
- [x] T024 [US2] Add debugging tips with `ros2 CLI` tools in `docs/module1/chapter1.2.mdx`

**Checkpoint**: At this point, Chapter 1.2 content and code examples are ready.

---

## Phase 5: User Story 3 - Design Humanoid Robot Structures (Priority: P2)

**Goal**: Student designs humanoid robot structures using URDF/Xacro.

**Independent Test**: Student can build and visualize a complete humanoid URDF model.

### Implementation for User Story 3 (Chapter 1.3: URDF for Humanoids)

- [x] T025 [US3] Write content for "Introduction to URDF" in `docs/module1/chapter1.3.mdx`
- [x] T026 [US3] Write content for "XML structure: links, joints, properties" in `docs/module1/chapter1.3.mdx`
- [x] T027 [US3] Write content for "Visual vs collision vs inertial properties" in `docs/module1/chapter1.3.mdx`
- [x] T028 [US3] Write content for "Xacro: parameterized URDF with macros" in `docs/module1/chapter1.3.mdx`
- [x] T029 [US3] Write content for "Building a simple humanoid model" in `docs/module1/chapter1.3.mdx`
- [ ] T030 [US3] Write content for "Joint types: revolute, continuous, prismatic" in `docs/module1/chapter1.3.mdx`
- [x] T031 [US3] Implement "Complete humanoid URDF with 20+ joints" code example in `code-examples/module1-ros2-nervous-system/chapter1.3/humanoid.urdf.xacro`
- [x] T032 [US3] Add visualization in RViz2 instructions in `docs/module1/chapter1.3.mdx`
- [x] T033 [US3] Describe "Add gripper hands to humanoid model" exercise in `docs/module1/chapter1.3.mdx`
- [x] T034 [US3] Add "Common URDF errors and validation" content in `docs/module1/chapter1.3.mdx`

**Checkpoint**: At this point, Chapter 1.3 content and code examples are ready.

---

## Phase 6: User Story 4 - Control a Basic Simulated Humanoid (Priority: P1 - Hands-on Project)

**Goal**: Student creates a ROS 2 system to control a simulated humanoid's arm.

**Independent Test**: Simulated humanoid's arm movements respond to commands and presets.

### Implementation for User Story 4 (Hands-on Project)

- [x] T035 [US4] Write content for "Hands-on Project: Basic Humanoid Controller" goal and requirements in `docs/module1/project.mdx`
- [x] T036 [US4] Implement URDF model with 6 DOF arm in `code-examples/module1-ros2-nervous-system/project/arm_robot.urdf.xacro`
- [x] T037 [US4] Implement publisher node for joint commands in `code-examples/module1-ros2-nervous-system/project/joint_command_publisher.py`
- [x] T038 [US4] Implement subscriber node for sensor feedback in `code-examples/module1-ros2-nervous-system/project/sensor_feedback_subscriber.py`
- [x] T039 [US4] Implement service for arm pose presets in `code-examples/module1-ros2-nervous-system/project/arm_pose_service.py`
- [x] T040 [US4] Write step-by-step implementation guide in `docs/module1/project.mdx`
- [x] T041 [US4] Create testing and validation checklist for the project in `docs/module1/project.mdx`
- [x] T042 [US4] Add expected outcomes and success criteria for the project in `docs/module1/project.mdx`

**Checkpoint**: At this point, the Hands-on Project content and code are ready.

---

## Phase 7: User Story 5 - Assess Module Competency (Priority: P3)

**Goal**: Student's understanding and practical skills are assessed.

**Independent Test**: Completion of assessment and comparison against rubric.

### Implementation for User Story 5 (Module Assessment)

- [x] T043 [US5] Write content for "Module Assessment" overview in `docs/module1/assessment.mdx`
- [x] T044 [US5] Create 10 multiple choice questions covering ROS 2 concepts in `docs/module1/assessment.mdx`
- [x] T045 [US5] Design 3 practical coding challenges in `docs/module1/assessment.mdx` (e.g., fix broken node communication, add new joint to URDF, implement custom service)
- [x] T046 [US5] Develop assessment rubric and passing criteria in `docs/module1/assessment.mdx`

**Checkpoint**: At this point, the Module Assessment content is ready.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final review, documentation, and quality assurance for the entire module.

- [x] T047 [P] Review all Docusaurus content in `docs/module1/` for clarity, consistency, and adherence to Content Quality standards.
- [x] T048 [P] Review all code examples in `code-examples/module1-ros2-nervous-system/` for PEP 8, `rclpy` best practices, and runnable status.
- [x] T049 [P] Verify all `requirements.txt` dependencies are pinned and correct for all code examples.
- [x] T050 [P] Ensure all code examples include full file paths, import statements, executable code, expected output, and run commands.
- [x] T051 [P] Add summary recap and "Next Steps" preview to `docs/module1/chapter1.3.mdx`
- [x] T052 [P] Final check for Legal & Ethics compliance across all content and code.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately.
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories.
- **User Story Phases (3-7)**: All depend on Foundational phase completion.
  - User Story 1, 2, and 4 are P1 (High Priority). Story 1 and 2 are foundational knowledge. Story 4 is the culminating project. While content creation can be parallelized, it is highly recommended to complete US1 and US2 before US4. US3 (URDF) can be worked on in parallel with US1/US2. US5 (Assessment) is dependent on all other content being finalized.
- **Polish (Phase 8)**: Depends on all user story content being complete.

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2). No dependencies on other stories for its core content.
- **User Story 2 (P1)**: Can start after Foundational (Phase 2). Relies on concepts from US1.
- **User Story 3 (P2)**: Can start after Foundational (Phase 2). Can be parallel with US1/US2.
- **User Story 4 (P1)**: Depends on US1, US2, and US3 for understanding ROS 2 fundamentals, communication, and URDF modeling.
- **User Story 5 (P3)**: Depends on content from US1, US2, US3, and US4 being complete for assessment questions/challenges.

### Within Each User Story

- Content writing tasks should generally precede code example implementation, which then feeds back into documentation.
- Exercises should be described after core content and code examples are presented.

### Parallel Opportunities

- Many content writing tasks within chapters can be parallelized (`[P]`).
- Multiple code examples can be implemented in parallel within a chapter or story.
- Module Assessment (US5) can begin in parallel with Polish Phase (Phase 8) if content is stable.

---

## Implementation Strategy

### Incremental Delivery (Recommended)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational → Foundation ready (ROS 2 environment and Docusaurus module base)
3. Complete Phase 3: User Story 1 → Test independently (run Hello World) → Content ready for Chapter 1.1
4. Complete Phase 4: User Story 2 → Test independently (multi-node communication) → Content ready for Chapter 1.2
5. Complete Phase 5: User Story 3 → Test independently (URDF visualization) → Content ready for Chapter 1.3
6. Complete Phase 6: User Story 4 → Test independently (simulated arm control) → Hands-on Project ready
7. Complete Phase 7: User Story 5 → Assessment ready
8. Complete Phase 8: Polish → Final module ready for review

### Parallel Team Strategy

With multiple content developers/engineers:

1. Team completes Setup + Foundational together.
2. Once Foundational is done:
   - Developer A: User Story 1 & 2 (ROS 2 Fundamentals, Communication)
   - Developer B: User Story 3 (URDF for Humanoids)
   - Developer C: User Story 4 (Hands-on Project - once A & B provide necessary context)
   - Developer D: User Story 5 (Assessment - once content from A, B, C is mostly stable)
3. Polish Phase (Phase 8) can then be done collaboratively or by a dedicated QA/editor.
