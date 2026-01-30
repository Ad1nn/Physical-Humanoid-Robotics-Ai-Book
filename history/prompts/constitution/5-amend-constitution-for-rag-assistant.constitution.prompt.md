---
id: 5
title: Amend Constitution for RAG Assistant
stage: constitution
date: 2025-12-29
surface: agent
model: gemini-pro
feature: none
branch: master
user: User
command: /sp.constitution
labels: [constitution, rag, ai-assistant]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
- .specify/templates/plan-template.md
tests:
- none
---

## Prompt

/sp.constitution  Project: Physical Humanoid Robotics AI Book  Phase: RAG-Based AI Assistant IntegrationContext:This phase extends the existing Robotics AI book project.The book is already written and deployed.This phase introduces an AI-powered Retrieval-Augmented Generation (RAG) assistant embedded inside the book.Core principles:- Accuracy over creativity- Strict reliance on retrieved content- No hallucination under any circumstance- Deterministic and explainable behavior- Clear separation of retrieval, reasoning, and generation- Engineering-first, research-grade implementationKey standards:- All answers must be grounded in retrieved text- Selected-text mode must ONLY use the provided selection- No external knowledge allowed unless explicitly permitted- All responses must be reproducible- Retrieval logic must be debuggable- Prompts must be explicit and version-controlledTechnical constraints:- Frontend: Existing Docusaurus site- Backend: FastAPI- Agent framework: OpenAI Agents SDK- LLM: Cohere Command / Command R+- Embeddings: Cohere Embed v3- Vector database: Qdrant Cloud- Metadata storage: Neon Postgres- Deployment: Vercel (frontend), Railway/Render (backend)Architectural rules:- RAG pipeline must be modular- No direct LLM calls without retrieval- Agent must support:  - Full-book query mode  - Selected-text-only query mode- Retrieval must be traceable to source documents- Prompt injection resistance is mandatorySecurity & safety:- No storage of user conversations- No exposure of API keys- Input validation required- Strict context isolation- No training on user dataPerformance constraints:- Retrieval latency ≤ 1.5s- End-to-end response ≤ 5s- Embeddings must be cached- System must scale without redesignSuccess criteria:- Users can query the book and receive correct answers- Selected-text queries never hallucinate- All answers trace back to source text- System passes manual verification- Ready for production deployment

## Response snapshot

The constitution was updated to version 1.4.0 to include a new set of principles for a RAG-based AI assistant. The `plan-template.md` was also updated to reflect these new principles.

## Outcome

- ✅ Impact: The project constitution was successfully amended to include principles for the new RAG AI assistant. This provides a clear governance framework for this new feature.
- 🧪 Tests: No tests were run as this was a documentation change.
- 📁 Files: 2 files were modified: `.specify/memory/constitution.md` and `.specify/templates/plan-template.md`.
- 🔁 Next prompts: The next logical step would be to create a specification for the RAG assistant feature using `/sp.specify`.
- 🧠 Reflection: The process of updating the constitution and related templates was smooth. The new principles are well-defined and will guide the development of the RAG assistant.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): n/a
