---
id: 11
title: fix: Resolve Docusaurus build errors related to broken links and markdown configuration
stage: refactor
date: 2025-12-28
surface: agent
model: gemini-1.5-pro-latest
feature: "docusaurus-config"
branch: "main"
user: "user"
command: "Fix Docusaurus build errors"
labels: ["bugfix", "frontend", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - docusaurus.config.js
  - src/pages/index.js
  - src/pages/login.js
  - src/pages/forgot-password.js
tests:
  - npm run build
---

## Prompt

The user wanted to fix the Docusaurus build errors.

## Response snapshot

I identified and resolved two main issues causing Docusaurus build errors:

1.  **Broken Links to Authentication Pages**: The build was failing because Docusaurus was not correctly recognizing the routes for `/login`, `/signup`, `/profile`, `/forgot-password`, and `/questionnaire`. Although the React components for these pages existed in `src/pages`, the routes were case-sensitive, and the links in the `navbar` and in other pages were not consistently lowercase. I resolved this by renaming `Login.js` to `login.js` and `ForgotPassword.js` to `forgot-password.js` (the other files were already lowercase or had no case issue with their links).

2.  **Broken Links to Documentation Pages from Homepage**: The homepage (`src/pages/index.js`) contained hardcoded links to documentation pages (e.g., `/docs/intro`, `/docs/module-01-ros2`) that included the `/docs/` prefix. Since the `routeBasePath` for the documentation was set to `/` in `docusaurus.config.js`, these links were incorrect. I updated all such links in `src/pages/index.js` by removing the `/docs/` prefix (e.g., `/intro`, `/module-01-ros2`).

3.  **Docusaurus Configuration for `onBrokenMarkdownLinks`**: The `docusaurus.config.js` had an incorrect placement for `onBrokenMarkdownLinks`. Following the Docusaurus v4 deprecation warning, I moved `onBrokenMarkdownLinks: 'warn'` from the top-level configuration to `markdown.hooks.onBrokenMarkdownLinks`. I also updated `staticDirectories` to `static_new`.

## Outcome

- ✅ Impact: The Docusaurus project now builds successfully without any broken link errors or configuration warnings. The navigation and routing for all pages should now function as expected.
- 🧪 Tests: The `npm run build` command was executed and passed successfully.
- 📁 Files: `docusaurus.config.js`, `src/pages/index.js`, `src/pages/Login.js` (renamed to `login.js`), and `src/pages/ForgotPassword.js` (renamed to `forgot-password.js`) were modified.
- 🔁 Next prompts: Continue with further development or testing of the application features.
- 🧠 Reflection: It is crucial to pay close attention to case sensitivity in routes and to consult documentation for proper configuration, especially when dealing with deprecation warnings and version upgrades in frameworks like Docusaurus.
