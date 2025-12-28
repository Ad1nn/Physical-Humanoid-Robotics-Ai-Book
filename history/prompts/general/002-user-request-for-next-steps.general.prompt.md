---
id: 002
title: User request for next steps
stage: general
date: 2025-12-28
surface: agent
model: gemini-1.5-pro
feature: none
branch: main
user: User
command: continue
labels: [next-step, user-instruction]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - history/prompts/general/001-project-analysis-and-familiarization.misc.prompt.md
tests:
  - none
---

## Prompt

continue

## Response snapshot

Now that the PHR has been created and I have a good understanding of the project, I need to know what the user wants me to do next. I will ask the user for their next instruction.

## Outcome

- ✅ Impact: User prompted for next steps, awaiting instruction.
- 🧪 Tests: N/A
- 📁 Files: N/A
- 🔁 Next prompts: Awaiting user instruction.
- 🧠 Reflection: Successfully prompted the user for their next action after initial project analysis.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
