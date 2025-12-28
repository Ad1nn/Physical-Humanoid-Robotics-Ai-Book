# Tasks: Docusaurus Site Setup for "Physical AI & Humanoid Robotics" Book

**Input**: Design documents from `specs/001-docusaurus-site-setup/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: The specification defines acceptance criteria that imply testing. Test-related tasks will be included where appropriate (e.g., verifying build, server).

**Organization**: Tasks are grouped by user story and phase to enable logical implementation and testing.

## Format: `[ID] [P?] [Story?] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- Main configs: `docusaurus.config.js`, `sidebars.js`, `package.json`
- Content root: `docs/`
- Static assets: `static/`
- Custom CSS: `src/css/custom.css`

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Initialize the Docusaurus project and establish the basic file structure.

- [x] T001 Initialize Docusaurus project using `npx create-docusaurus@latest . classic` in project root.
- [x] T002 Update `package.json` with correct project name and description.
- [x] T003 Create `static/img/` directory for static images.
- [x] T004 Create `static/code/` directory for static code snippets.
- [x] T005 [P] Create `src/css/custom.css` with placeholder content.

---

## Phase 2: Core Configuration (Site-wide Features)

**Purpose**: Configure global Docusaurus settings as per site requirements.

- [x] T006 [US1] Set site `title` to "Physical AI & Humanoid Robotics" in `docusaurus.config.js`.
- [x] T007 [US1] Set site `tagline` to "From Digital Brain to Physical Body" in `docusaurus.config.js`.
- [x] T008 [US3] Configure dark mode default with toggle in `docusaurus.config.js` (`themeConfig.colorMode`).
- [x] T009 [US2] Enable search functionality in `docusaurus.config.js` (e.g., `@docusaurus/preset-classic` options).
- [x] T010 [US2] Configure syntax highlighting for Python, YAML, XML, bash in `docusaurus.config.js` (`themeConfig.prism`).
- [x] T011 [US2] Enable code block copy button in `docusaurus.config.js` (`themeConfig.prism`).
- [x] T012 [US2] Enable line numbers for code blocks in `docusaurus.config.js` (`themeConfig.prism`).
- [x] T013 Configure `editUrl` to point to GitHub repository for documentation contributions in `docusaurus.config.js`.
- [x] T014 Configure Docusaurus `url` and `baseUrl` for deployment.

---

## Phase 3: Navigation & Content Structure

**Purpose**: Define the sidebar navigation and create the content folder structure for all modules.

- [x] T015 [US1] Create `sidebars.js` file for main navigation structure.
- [x] T016 [US1] Define "Introduction" and "Prerequisites & Setup" entries in `sidebars.js`.
- [x] T017 [US1] Create `docs/intro.md` for Introduction page content.
- [x] T018 [US1] Create `docs/prerequisites.md` for Prerequisites & Setup content.
- [x] T019 [US1] Create module folder `docs/module-01-ros2/`.
- [x] T020 [US1] Create `docs/module-01-ros2/_category_.json` using template.
- [x] T021 [US1] Create `docs/module-01-ros2/index.mdx` (Module Overview) and integrate existing Module 1 overview content from `specs/001-module1-ros2-nervous-system/spec.md`.
- [x] T022 [US1] Create `docs/module-01-ros2/chapter1.1.mdx` and integrate existing content from `docs/module1/chapter1.1.mdx`.
- [x] T023 [US1] Create `docs/module-01-ros2/chapter1.2.mdx` and integrate existing content from `docs/module1/chapter1.2.mdx`.
- [x] T024 [US1] Create `docs/module-01-ros2/chapter1.3.mdx` and integrate existing content from `docs/module1/chapter1.3.mdx`.
- [x] T025 [US1] Create `docs/module-01-ros2/project.mdx` and integrate existing content from `docs/module1/project.mdx`.
- [x] T026 [US1] Create `docs/module-01-ros2/assessment.mdx` and integrate existing content from `docs/module1/assessment.mdx`.
- [x] T027 [US1] Update `sidebars.js` to include "Module 1: The Robotic Nervous System (ROS 2)" structure.
- [x] T028 [US1] Create placeholder module folder `docs/module-02-simulation/`.
- [x] T029 [US1] Create `docs/module-02-simulation/_category_.json` using template.
- [x] T030 [US1] Create placeholder module folder `docs/module-03-isaac/`.
- [x] T031 [US1] Create `docs/module-03-isaac/_category_.json` using template.
- [x] T032 [US1] Create placeholder module folder `docs/module-04-vla/`.
- [x] T033 [US1] Create `docs/module-04-vla/_category_.json` using template.
- [x] T034 [US1] Update `sidebars.js` to include placeholder entries for Modules 2, 3, and 4.

---

## Phase 4: Features & Integrations

**Purpose**: Enable required Docusaurus features and ensure mobile responsiveness and deployment readiness.

- [x] T035 [US2] Ensure admonitions (`:::tip`, `:::warning`, `:::info`, `:::danger`) render correctly in documentation.
- [x] T036 [US1] Ensure mobile responsiveness by reviewing `src/css/custom.css` and Docusaurus theme settings.
- [x] T037 [US1] Configure GitHub Pages deployment settings in `docusaurus.config.js`.

---

## Phase 5: Deliverables & Verification

**Purpose**: Verify Docusaurus functionality and ensure all acceptance criteria are met.

- [ ] T038 [US1, US2, US3] Verify all modules are visible in sidebar navigation (`SC-001`).
- [ ] T039 [US2] Verify code blocks render with syntax highlighting for Python, YAML, XML, bash (`SC-002`).
- [ ] T040 [US3] Verify dark/light mode toggle works (`SC-003`).
- [ ] T041 [US1, US2, US3] Start Docusaurus dev server (`npm start`) and verify it runs without errors (`SC-006`).
- [ ] T042 [US1, US2, US3] Run Docusaurus build command (`npm run build`) and verify it completes without errors (`SC-004`).
- [ ] T043 [US1, US2, US3] Verify Module 1 content displays correctly (`SC-005`).
- [x] T044 [P] Document setup instructions in `README.md` for Docusaurus environment.
- [ ] T045 [P] Conduct performance testing for page load time (Deliverable: < 3s, `SC-007`).
- [ ] T046 [P] Conduct performance testing for build time (Deliverable: < 5 min, `SC-008`).

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately.
- **Core Configuration (Phase 2)**: Depends on Setup completion.
- **Navigation & Content Structure (Phase 3)**: Depends on Phase 1 & 2 completion.
- **Features & Integrations (Phase 4)**: Depends on Phase 1 & 2 completion.
- **Deliverables & Verification (Phase 5)**: Depends on all previous phases being mostly complete.

### User Story Dependencies

- **User Story 1 (P1)**: `Browse Book Content` - Primarily depends on Phase 1, 2, 3 and 4 (mobile responsive).
- **User Story 2 (P1)**: `View Code Examples and Admonitions` - Primarily depends on Phase 2 (syntax highlighting, copy button, line numbers) and Phase 4 (admonitions).
- **User Story 3 (P2)**: `Customize Site Viewing Experience` - Primarily depends on Phase 2 (dark mode toggle).

### Within Each Phase

- Tasks should generally be executed sequentially unless marked `[P]` for parallelization.

### Parallel Opportunities

- Tasks marked `[P]` (e.g., T005, T045, T046) can be executed in parallel.
- Phases 2, 3, and 4 have some interdependencies but can have internal parallelization.

---

## Implementation Strategy

### Incremental Delivery (Recommended)

1.  Complete Phase 1: Setup → Basic Docusaurus project initialized.
2.  Complete Phase 2: Core Configuration → Site-wide features configured.
3.  Complete Phase 3: Navigation & Content Structure → All module folders and sidebar are set up.
4.  Complete Phase 4: Features & Integrations → All specific Docusaurus features enabled.
5.  Complete Phase 5: Deliverables & Verification → Site fully tested and ready for deployment.

### Parallel Team Strategy

With multiple developers:

1.  Team completes Phase 1 (Setup) collaboratively.
2.  Once Phase 1 is done:
    *   Developer A: Focus on Phase 2 (Core Configuration).
    *   Developer B: Focus on Phase 3 (Navigation & Content Structure).
    *   Developer C: Focus on Phase 4 (Features & Integrations).
3.  Phase 5 (Deliverables & Verification) can be a collaborative effort or handled by a dedicated QA/release engineer.
