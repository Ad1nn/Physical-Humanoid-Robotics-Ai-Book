# Implementation Plan: RAG Knowledge Ingestion & Embedding

**Branch**: `1-rag-knowledge-ingestion` | **Date**: 2025-12-29 | **Spec**: [link to spec.md](spec.md)
**Input**: Feature specification from `specs/1-rag-knowledge-ingestion/spec.md`

## Summary

This plan outlines the technical implementation for creating a RAG knowledge ingestion pipeline. It will take Docusaurus markdown files, chunk them, create embeddings with Cohere, and store them in Qdrant and Neon Postgres.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**: cohere, qdrant-client, psycopg2-binary, python-dotenv
**Storage**: Qdrant Cloud (vector store), Neon Postgres (metadata store)
**Testing**: pytest
**Target Platform**: Local or CI/CD environment with access to the cloud services.
**Project Type**: Single project (ingestion script)
**Performance Goals**: The pipeline should be able to process the entire book content in a reasonable time.
**Constraints**: Must be idempotent, no LLM usage in this phase, only Docusaurus markdown content.
**Scale/Scope**: The entire content of the book in the `docs/` directory.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **Practical-First Pedagogy**: N/A for this feature.
- [x] **Technical Accuracy**: The plan uses the specified technologies (Cohere, Qdrant, Neon).
- [x] **Progressive Complexity**: N/A for this feature.
- [x] **Simulation-First**: N/A for this feature.
- [x] **Content Quality**: The plan includes validation steps to ensure content is processed correctly.
- [x] **Code Standards**: The code will adhere to PEP 8 and use type hints.
- [x] **Legal & Ethics**: N/A for this feature.
- [x] **Technical Constraints**: The plan respects the defined technical constraints.
- [x] **User-Centric Personalization**: N/A for this feature.
- [x] **Seamless Authentication**: N/A for this feature.
- [x] **Privacy-Focused Data Collection**: N/A for this feature.
- [x] **Accessible Multilingual Support**: N/A for this feature.
- [x] **Non-Intrusive Feature Integration**: N/A for this feature.

### RAG Assistant Constitution Check
- [x] **Accuracy Over Creativity**: The ingestion pipeline is focused on accurately representing the source material.
- [x] **Strict Content Grounding**: The pipeline ensures traceability between chunks and source documents.
- [x] **Zero Hallucination**: N/A for this phase (no LLM).
- [x] **Deterministic & Explainable**: The process is deterministic and the code will be explainable.
- [x] **Clear Separation of Concerns**: The pipeline is designed with modular components.
- [x] **Engineering-First Implementation**: The plan follows engineering best practices.

## Project Structure

### Documentation (this feature)

```text
specs/1-rag-knowledge-ingestion/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
└── tasks.md             # Phase 2 output
```

### Source Code (repository root)
```text
ingestion/
├── main.py              # Main script to run the ingestion pipeline
├── discover.py          # Functions for discovering content
├── parser.py            # Functions for parsing and chunking
├── embed.py             # Functions for creating embeddings
├── storage.py           # Functions for storing embeddings and metadata
└── validation.py        # Functions for validating the ingestion
tests/
└── ingestion/
    ├── test_parser.py
    └── test_storage.py
```

**Structure Decision**: A single project structure is chosen as this feature is a self-contained script. The code is organized into modules based on functionality.

