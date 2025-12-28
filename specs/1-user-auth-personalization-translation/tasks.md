---

description: "Task list for implementing User Authentication, Personalization & Translation"
---

# Tasks: User Authentication, Personalization & Translation

**Input**: Design documents from `/specs/1-user-auth-personalization-translation/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for both backend and frontend.

- [X] T001 Initialize Node.js project in `backend/` with `npm init -y`
- [X] T002 [P] Install backend dependencies: `npm install express knex pg body-parser cors` in `backend/`
- [X] T003 [P] Create backend directory structure: `backend/src/models`, `backend/src/services`, `backend/src/api`, `backend/tests`
- [X] T004 [P] Install frontend dependencies: `npm install axios` in `frontend/`
- [X] T005 [P] Create frontend directory structure: `frontend/src/components`, `frontend/src/pages`, `frontend/src/services`, `frontend/src/context`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete.

- [X] T006 Create database migration for `users`, `user_profiles`, and `user_preferences` tables in `backend/src/migrations/`
- [X] T007 Create Knex models for `User`, `UserProfile`, and `UserPreferences` in `backend/src/models/`
- [X] T008 Install and configure Better Auth.
- [X] T009 Create `AuthContext.js` in `frontend/src/context/` to manage authentication state.
- [X] T010 Wrap the Docusaurus application with Better Auth provider in `src/theme/Root.js`.

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel.

---

## Phase 3: User Story 1 - New User Signup and Personalization (Priority: P1) 🎯 MVP

**Goal**: A new user signs up, provides their background information, and experiences personalized content for the first time.

**Independent Test**: A new user can successfully create an account, complete the background questionnaire, and see personalized content on a chapter page.

### Implementation for User Story 1

- [X] T011 [US1] Create Signup page component in `frontend/src/pages/Signup.js`.
- [X] T012 [US1] Implement signup form with validation in `frontend/src/pages/Signup.js`.
- [X] T013 [US1] Create background questionnaire page in `frontend/src/pages/Questionnaire.js`.
- [X] T014 [US1] Create API endpoint in `backend/src/api/users.js` to create a new user and save their profile information.
- [X] T015 [US1] Create `PersonalizedContent.js` component in `frontend/src/components/` to show/hide content based on user profile.
- [X] T016 [US1] Integrate `PersonalizedContent` component into Docusaurus chapter pages.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently.

---

## Phase 4: User Story 2 - Existing User Login and Translation (Priority: P2)

**Goal**: An existing user logs in and uses the translation feature to read a chapter in Urdu.

**Independent Test**: An existing user can log in, navigate to a chapter, and translate the content to Urdu.

### Implementation for User Story 2

- [X] T017 [US2] Create Login page component in `frontend/src/pages/Login.js`.
- [X] T018 [US2] Implement login form in `frontend/src/pages/Login.js`.
- [X] T019 [US2] Implement translation service in `backend/src/services/translationService.js` using a translation API.
- [X] T020 [US2] Create API endpoint in `backend/src/api/translate.js` to handle translation requests.
- [X] T021 [US2] Create a "Translate" button component in `frontend/src/components/TranslateButton.js`.
- [X] T022 [US2] Integrate the translation functionality into chapter pages.
- [X] T023 [US2] Implement caching for translations in the `translations` database table.

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently.

---

## Phase 5: User Story 3 - User Profile Management (Priority: P3)

**Goal**: A user wants to update their profile information or delete their account.

**Independent Test**: A user can navigate to their profile page, update their background information, and see the changes reflected in the content personalization.

### Implementation for User Story 3

- [X] T024 [US3] Create User Profile page in `frontend/src/pages/Profile.js`.
- [X] T025 [US3] Display current user's information on the profile page.
- [X] T026 [US3] Implement a form to update user profile information.
- [X] T027 [US3] Create an API endpoint in `backend/src/api/users.js` to update user profile.
- [X] T028 [US3] Implement "Delete Account" functionality with a confirmation dialog.
- [X] T029 [US3] Create an API endpoint in `backend/src/api/users.js` to delete a user account.

**Checkpoint**: All user stories should now be independently functional.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories.

- [X] T030 Implement "Forgot Password" flow using Better Auth's password reset feature.
- [X] T031 [P] Implement OAuth login with Google and GitHub.
- [X] T032 Add robust error handling for API failures (e.g., translation API down).
- [X] T033 Ensure that the experience for non-logged-in users is not disrupted.
- [X] T034 [P] Add unit and integration tests for all new backend and frontend components.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately.
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories.
- **User Stories (Phase 3+)**: All depend on Foundational phase completion.
- **Polish (Final Phase)**: Depends on all desired user stories being complete.

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2).
- **User Story 2 (P2)**: Can start after Foundational (Phase 2).
- **User Story 3 (P3)**: Can start after Foundational (Phase 2).

### Parallel Opportunities

- Once the Foundational phase is complete, all user stories can be worked on in parallel.
- Tasks marked with [P] within each phase can be executed in parallel.

## Implementation Strategy

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready.
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!).
3. Add User Story 2 → Test independently → Deploy/Demo.
4. Add User Story 3 → Test independently → Deploy/Demo.
5. Each story adds value without breaking previous stories.
