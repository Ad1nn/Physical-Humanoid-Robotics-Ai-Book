# Tasks: Module 3 - The AI-Robot Brain (NVIDIA Isaac)

**Input**: Design documents from `/specs/003-module3-isaac-ai-brain/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

**Tests**: Testing for this feature will be done via manual execution and visual verification of Isaac Sim and ROS 2 output. Automated tests are not explicitly required but functional verification is key.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create the directory structure for the new module's content and code.

- [x] T001 [P] Create documentation directory `docs/module-03-isaac/`
- [x] T002 [P] Create code examples directory `code-examples/module3-isaac-ai-brain/`
- [x] T003 [P] Add category metadata file `docs/module-03-isaac/_category_.json`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Create essential setup guides and shared resources for the module.

- [x] T004 Write the main module overview page `docs/module-03-isaac/index.mdx`
- [x] T005 [P] Document Isaac Sim installation/setup in `code-examples/module3-isaac-ai-brain/chapter3.1/isaac_sim_setup/README.md`
- [x] T006 [P] Document Isaac ROS workspace setup in `code-examples/module3-isaac-ai-brain/chapter3.2/isaac_ros_ws/README.md`

**Checkpoint**: Foundation ready - user story implementation can now begin.

---

## Phase 3: User Story 1 - Set up Isaac Sim & Generate Synthetic Data (Priority: P1) 🎯 MVP

**Goal**: Create the content and code for Chapter 3.1, teaching students to set up Isaac Sim and generate synthetic data.

**Independent Test**: Student can launch Isaac Sim with a humanoid scene and generate synthetic RGB, depth, and LiDAR data programmatically.

### Implementation for User Story 1

- [x] T007 [US1] Write the full content for Chapter 3.1 in `docs/module-03-isaac/chapter3.1.mdx`
- [x] T008 [P] [US1] Create example Isaac Sim scene USD file `code-examples/module3-isaac-ai-brain/chapter3.1/isaac_sim_setup/simple_humanoid_scene.usd`
- [x] T009 [US1] Create synthetic data generation script `code-examples/module3-isaac-ai-brain/chapter3.1/synthetic_data_gen.py`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently.

---

## Phase 4: User Story 2 - Implement VSLAM for Humanoid Localization with Isaac ROS (Priority: P2)

**Goal**: Create the content and code for Chapter 3.2, teaching students to implement VSLAM using Isaac ROS.

**Independent Test**: Student can run the Isaac ROS VSLAM pipeline in Isaac Sim, and visualize continuous pose estimates and map building for the humanoid.

### Implementation for User Story 2

- [x] T010 [US2] Write the full content for Chapter 3.2 in `docs/module-03-isaac/chapter3.2.mdx`
- [x] T011 [P] [US2] Create example stereo camera configuration for USD scene `code-examples/module3-isaac-ai-brain/chapter3.2/isaac_sim_setup/stereo_camera_humanoid.usd`
- [x] T012 [US2] Create Isaac ROS VSLAM launch file `code-examples/module3-isaac-ai-brain/chapter3.2/isaac_ros_ws/launch/vslam_pipeline.launch.py`
- [x] T013 [US2] Write Python script to visualize/evaluate VSLAM output `code-examples/module3-isaac-ai-brain/chapter3.2/scripts/vslam_evaluator.py`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently.

---

## Phase 5: User Story 3 - Configure Nav2 for Autonomous Bipedal Navigation (Priority: P3)

**Goal**: Create the content and code for Chapter 3.3, teaching students to configure Nav2 for humanoid navigation.

**Independent Test**: Student can configure Nav2 in Isaac Sim, set a goal, and observe the humanoid autonomously navigating to it while avoiding obstacles.

### Implementation for User Story 3

- [x] T014 [US3] Write the full content for Chapter 3.3 in `docs/module-03-isaac/chapter3.3.mdx`
- [x] T015 [P] [US3] Create Nav2 costmap configuration file `code-examples/module3-isaac-ai-brain/chapter3.3/nav2_config/costmap_params.yaml`
- [x] T016 [P] [US3] Create Nav2 planner configuration file `code-examples/module3-isaac-ai-brain/chapter3.3/nav2_config/planner_params.yaml`
- [x] T017 [US3] Create Nav2 launch file for humanoid navigation `code-examples/module3-isaac-ai-brain/chapter3.3/launch/nav2_humanoid.launch.py`

**Checkpoint**: All user stories should now be independently functional.

---

## Phase 6: Project & Assessment

**Purpose**: Finalize the module with a hands-on project and assessment materials.

- [x] T018 [P] Write the hands-on project guide `docs/module-03-isaac/project.mdx`
- [x] T019 [P] Write the module assessment content `docs/module-03-isaac/assessment.mdx`
- [x] T020 [P] Create placeholder Isaac Sim scene for project `code-examples/module3-isaac-ai-brain/project/isaac_sim_scene/autonomous_env.usd`
- [x] T021 [P] Create placeholder ROS 2 workspace structure for project `code-examples/module3-isaac-ai-brain/project/ros2_ws/README.md`
- [x] T022 [P] Create placeholder script for goal publishing in project `code-examples/module3-isaac-ai-brain/project/scripts/goal_publisher.py`
- [x] T023 Review all content and code examples for clarity, accuracy, and consistency.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: Can start immediately.
- **Foundational (Phase 2)**: Depends on Setup completion.
- **User Stories (Phases 3-5)**: All depend on Foundational phase completion.
- **Project & Assessment (Phase 6)**: Depends on all User Story phases being complete.

### User Story Dependencies

- **User Story 1 (P1)**: Depends on Foundational.
- **User Story 2 (P2)**: Depends on Foundational. Integrates concepts from US1.
- **User Story 3 (P3)**: Depends on Foundational. Integrates concepts from US1 and US2.

### Parallel Opportunities

- Once Phase 2 is complete, work on all three user stories (Phase 3, 4, 5) can begin in parallel if code examples are strictly independent. However, due to logical progression (VSLAM building on Isaac Sim, Nav2 building on VSLAM), a sequential approach is recommended for content creation, but code structure can be parallelized.
- Within each story, content writing can happen in parallel with code creation.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test Isaac Sim setup and synthetic data generation.

### Incremental Delivery

1. Complete MVP (US1).
2. Add User Story 2 (VSLAM). Test independently.
3. Add User Story 3 (Nav2). Test independently.
4. Complete Phase 6 (Project & Assessment).
5. Each chapter and its code examples should function as a standalone deliverable, with increasing complexity building on previous chapters.
