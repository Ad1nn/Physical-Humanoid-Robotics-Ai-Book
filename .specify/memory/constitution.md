<!--
Sync Impact Report:
- Version change: 1.1.1 → 1.2.0
- List of modified principles: Added UI/UX principles.
- Added sections: UI/UX Principles, Design Quality Standards, Component Standards, Content Presentation Standards, Navigation Standards, Performance Standards, UI/UX Technical Constraints, UI/UX Scope, UI/UX Success Criteria, UI/UX Deliverables.
- Removed sections: None.
- Templates requiring updates:
  - ⚠ .specify/templates/plan-template.md
  - ⚠ .specify/templates/spec-template.md
  - ⚠ .specify/templates/tasks-template.md
  - ✅ .specify/templates/commands/*.md
  - ⚠ README.md
- Follow-up TODOs: None.
-->

# Physical AI & Humanoid Robotics: From Digital Brain to Physical Body Constitution

## Core Principles

### I. Practical-First Pedagogy
This book follows a 60% hands-on, 40% theory split. Every theoretical concept must be accompanied by practical, runnable examples to ensure a learning-by-doing approach.

### II. Technical Accuracy
All technical content, especially code and framework references, must be accurate and version-specific. Vague or untested instructions are prohibited.

### III. Progressive Complexity
The content will be structured to guide learners from beginner to expert. Each module and chapter must build upon the previous one, with clearly defined prerequisites.

### IV. Simulation-First Approach
The primary learning environment is simulation. All projects and examples must be runnable in a simulated environment, with physical hardware being an optional extension.

## UI/UX Principles
### I. Professional technical documentation aesthetic
The website must maintain a professional and polished appearance consistent with high-quality technical documentation.

### II. Mobile-first responsive design
The website must be designed and implemented with a mobile-first approach, ensuring optimal viewing and interaction across various screen sizes and devices.

### III. Accessibility (WCAG 2.1 AA compliance)
The website must adhere to WCAG 2.1 AA guidelines to ensure it is accessible to users with disabilities.

### IV. Performance-focused (fast page loads)
The website must prioritize performance, aiming for fast page loads and a smooth user experience.

### V. Student-centric user experience
The user experience must be tailored to the needs of students, facilitating easy navigation, clear content presentation, and engaging interactive elements.

## Key Standards

### Content Quality
- All code examples must be complete, tested, and runnable.
- Every concept must be paired with a working implementation.
- Version-specific references for all frameworks (e.g., ROS 2 Humble, Unity 2022.3 LTS) are mandatory.
- Learning objectives must be explicitly stated at the beginning of each module and chapter.
- Prerequisites must be clearly defined before each section.

### Code Standards
- Python code must adhere to PEP 8, include type hints, and have clear docstrings.
- ROS 2 development must follow `rclpy` best practices and use a proper package structure.
- All dependencies must be pinned in a `requirements.txt` file for reproducibility.

### Repository Structure
The project must adhere to the following directory structure:
```
physical-ai-book/
├── docs/                    # Docusaurus content
├── code-examples/           # Tested code samples
├── specs/                   # SDD specifications
└── assets/                  # Images, diagrams
```

### Legal & Ethics
- All source code is licensed under the MIT License.
- All documentation and written content is licensed under CC BY 4.0.
- Clear safety warnings must be provided before any instructions related to physical hardware deployment.
- The project and its outputs must not be used for weaponization or surveillance applications.

## Design Quality Standards
- Consistent visual hierarchy across all pages
- Clear typography (readability score 80+)
- Robotics/AI themed color palette
- Professional technical book appearance
- Dark mode as default with light mode support

## Component Standards
- Reusable React components for common patterns
- Custom MDX components for interactive elements
- Consistent spacing and layout grid
- Responsive breakpoints: mobile (375px), tablet (768px), desktop (1024px+)

## Content Presentation Standards
- Code blocks: syntax highlighting, copy button, line numbers
- Admonitions: tip, warning, info, danger, note with custom icons
- Learning objectives highlighted at chapter start
- Progress indicators for module completion
- Assessment quizzes with interactive UI

## Navigation Standards
- Clear sidebar hierarchy with module icons
- Breadcrumbs for orientation
- Next/Previous chapter navigation
- Search functionality prominent
- Mobile hamburger menu

## Performance Standards
- Page load < 2 seconds
- Build time < 5 minutes
- Lighthouse score > 90
- Optimized images (WebP format)
- Lazy loading for heavy content

## Technical Constraints

- **ROS 2:** Humble Hawksbill (LTS)
- **Simulators:** Gazebo Garden/Harmonic, NVIDIA Isaac Sim 2023.1.1+
- **Game Engine:** Unity 2022.3 LTS
- **Python:** 3.10+
- **OS:** Ubuntu 22.04 LTS (Docker recommended for cross-platform compatibility)
- **Hardware:** Minimum 16GB RAM and an NVIDIA GPU with at least 6GB VRAM.

## UI/UX Technical Constraints
- Docusaurus 3.x
- React 18+
- Tailwind CSS or vanilla CSS (custom theme)
- No breaking changes to existing content
- Must work with all 4 modules

## Book Structure

The book will be structured as follows, with approximate word counts:

- Introduction page (1,200-1,800 words)
- Prerequisites & Setup guide (1,800-2,200 words)
- Module 1: The Robotic Nervous System (ROS 2)
  - Chapter 1.1: ROS 2 Fundamentals (3,000-4,000 words)
  - Chapter 1.2: Nodes, Topics & Services (3,500-4,500 words)
  - Chapter 1.3: URDF for Humanoids (3,000-4,000 words)
  - Hands-on Project + Assessment
- Module 2: The Digital Twin (Gazebo & Unity)
  - Chapter 2.1: Gazebo Physics Simulation (3,000-4,000 words)
  - Chapter 2.2: Unity High-Fidelity Rendering (3,000-4,000 words)
  - Chapter 2.3: Sensor Simulation (2,500-3,500 words)
  - Hands-on Project + Assessment
- Module 3: The AI-Robot Brain (NVIDIA Isaac)
  - Chapter 3.1: Isaac Sim & Synthetic Data (3,500-4,500 words)
  - Chapter 3.2: Isaac ROS & VSLAM (3,000-4,000 words)
  - Chapter 3.3: Nav2 Path Planning (3,000-4,000 words)
  - Hands-on Project + Assessment
- Module 4: Vision-Language-Action (VLA)
  - Chapter 4.1: Voice Commands with Whisper (2,500-3,500 words)
  - Chapter 4.2: LLM Cognitive Planning (3,000-4,000 words)
  - Chapter 4.3: Autonomous Humanoid Integration (3,500-4,500 words)
  - Final Capstone Project
- Each chapter: learning objectives, theory, code examples, exercises, summary

## UI/UX Scope
- Homepage/landing page design
- Custom theme configuration
- Module and chapter page layouts
- Navigation improvements
- Interactive UI components
- Assessment quiz interfaces

## UI/UX Success Criteria
- Professional appearance comparable to top tech documentation sites
- Students can navigate intuitively without training
- All modules render consistently
- Mobile experience equals desktop quality
- Zero accessibility violations
- Positive user feedback on readability

## UI/UX Deliverables
- Custom Docusaurus theme
- Homepage with course overview
- Enhanced module/chapter layouts
- Reusable UI components
- Style guide documentation
- Deployment-ready build

## Governance

This constitution is the single source of truth for all project principles and standards. All contributions, reviews, and decisions must align with it. Amendments require a documented proposal, review, and an update to the version number according to semantic versioning.

**Version**: 1.2.0 | **Ratified**: 2025-12-21 | **Last Amended**: 2025-12-22