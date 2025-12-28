# Quickstart Guide: UI/UX Enhancement

**Feature Branch**: `001-ui-ux-enhancement`  
**Created**: 2025-12-22  
**Related Spec**: specs/001-ui-ux-enhancement/spec.md
**Related Plan**: specs/001-ui-ux-enhancement/plan.md

## Overview

This Quickstart Guide is for developers who want to understand the foundational aspects of the UI/UX enhancements implemented for the "Physical AI & Humanoid Robotics" Docusaurus book. It provides guidance on how to set up the development environment, understand the custom theme structure, and begin contributing to the frontend.

## Goals of this Quickstart

-   To enable quick setup of the Docusaurus development environment.
-   To provide an overview of the custom theme and component structure.
-   To guide contributors on how to modify existing UI elements or add new ones.
-   To explain how to run and test UI/UX changes locally.

## Development Setup

1.  **Clone the repository**:
    ```bash
    git clone [repository-url]
    cd physical-ai-book
    git checkout 001-ui-ux-enhancement
    ```
2.  **Install dependencies**:
    ```bash
    npm install
    ```
3.  **Start the Docusaurus development server**:
    ```bash
    npm start
    ```
    (Note: If port 3000 is in use, Docusaurus will suggest an alternative port like 3001 or 3002.)

## Custom Theme Structure

The UI/UX enhancements are primarily implemented within the Docusaurus custom theme. Key files and directories include:

-   `src/theme/`: Contains overridden Docusaurus components and custom layouts.
-   `src/css/custom.css`: For global styling and utility classes.
-   `src/components/`: Directory for custom React components (e.g., `Admonition`, `LearningObjectives`, `ProgressBar`, `Quiz`, `LazyLoad`, `HeavyContent`). These can be imported using the `@components` alias.

## Contributing to UI/UX

-   **Modifying existing elements**: Locate the relevant React component in `src/theme/` or `src/components/` and apply changes.
-   **Adding new components**: Create new React components in `src/components/` and integrate them into relevant page layouts or MDX content.
-   **Styling**: Apply styling via `custom.css`.

## Testing Your Changes

-   **Local development**: Changes are live-reloaded during `npm start`.
-   **Component testing**: Run Jest tests for individual React components.
    ```bash
    npm test
    ```
-   **End-to-end testing**: Run Playwright tests for full user flows and UI interactions across different browsers and viewports.
    ```bash
    npx playwright test
    ```

## Further Documentation

Refer to the Docusaurus official documentation for detailed guides on theme customization and plugin development.
