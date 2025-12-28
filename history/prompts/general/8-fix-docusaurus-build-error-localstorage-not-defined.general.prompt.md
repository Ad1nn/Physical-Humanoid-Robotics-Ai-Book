---
phr_id: 8
phr_title: Fix Docusaurus Build Error - localStorage Not Defined
phr_stage: general
phr_date_iso: 2025-12-28
phr_surface: agent
phr_model: 
phr_feature: 
phr_branch: 001-ui-ux-enhancement
phr_user: 
phr_command: "run npm run build there are some errors"
phr_labels: ["docusaurus", "build", "error", "localStorage", "SSR", "fix"]
phr_links: 
phr_files_yaml:
  - frontend/src/context/AuthContext.js
  - history/prompts/general/8-fix-docusaurus-build-error-localstorage-not-defined.general.prompt.md
phr_tests_yaml:
- none
---
# Fix Docusaurus Build Error - localStorage Not Defined

## Prompt

```text
run npm run build there are some errors
```

## Response

```text
The `npm run build` command has now completed successfully.

The previous build errors, specifically `ReferenceError: localStorage is not defined`, were occurring because the `AuthProvider` component was attempting to access `localStorage` during Docusaurus's server-side rendering (SSR) process. `localStorage` is a browser-specific API and is not available in a Node.js environment.

The fix involved modifying `frontend/src/context/AuthContext.js`:
-   Imported `ExecutionEnvironment` from `@docusaurus/ExecutionEnvironment`.
-   Initialized `user` to `null` and `isAuthenticated` to `false` in their `useState` hooks.
-   Added a `useEffect` hook that runs only on the client-side (`ExecutionEnvironment.canUseDOM`) to read the actual `user` and `isAuthenticated` values from `localStorage` and update the state.
-   Ensured the `useEffect` responsible for syncing state with `localStorage` also checks `ExecutionEnvironment.canUseDOM`.

This ensures that `localStorage` is only accessed when the browser environment is available, resolving the SSR build errors.
```