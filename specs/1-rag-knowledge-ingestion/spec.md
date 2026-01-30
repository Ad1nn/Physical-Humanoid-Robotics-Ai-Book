# Feature Specification: RAG Knowledge Ingestion & Embedding

**Feature Branch**: `1-rag-knowledge-ingestion`
**Created**: 2025-12-29
**Status**: Draft
**Input**: User description: "Project: Physical Humanoid Robotics AI Book Phase: RAG Knowledge Ingestion & Embedding..."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Content Ingestion and Embedding (Priority: P1)

As a developer, I want to run a script that ingests all Docusaurus markdown content, processes it into chunks, and generates embeddings, so that the book's knowledge is ready for the RAG assistant.

**Why this priority**: This is the core functionality of the feature.

**Independent Test**: The script runs successfully and the data is stored in the vector and metadata stores.

**Acceptance Scenarios**:

1. **Given** the `docs/` directory contains markdown files, **When** the ingestion script is run, **Then** embeddings and metadata are stored in Qdrant and Postgres respectively.
2. **Given** a clean database and vector store, **When** the ingestion script is run, **Then** the number of chunks in the database matches the number of chunks generated from the source files.

### User Story 2 - Idempotent Ingestion (Priority: P2)

As a developer, I want the ingestion pipeline to be idempotent, so that I can re-run it safely without creating duplicate or corrupted data.

**Why this priority**: This is crucial for maintainability and resilience.

**Independent Test**: Running the ingestion script multiple times does not change the state of the data stores after the first successful run.

**Acceptance Scenarios**:

1. **Given** a successful initial ingestion, **When** the ingestion script is run again, **Then** the number of items in the database and vector store remains the same.
2. **Given** a successful initial ingestion, **When** a single source file is updated and the script is re-run, **Then** only the chunks from the updated file are changed in the data stores.

### User Story 3 - Documentation and Validation (Priority: P3)

As a developer, I want to have clear documentation on how to run the ingestion script and validate its output, so that I can easily maintain the knowledge base.

**Why this priority**: Good documentation is essential for long-term maintenance.

**Independent Test**: A new developer can successfully run the ingestion script and validate the output by following the documentation.

**Acceptance Scenarios**:

1. **Given** the project README, **When** a developer follows the instructions for knowledge ingestion, **Then** they are able to successfully run the script.
2. **Given** the validation documentation, **When** a developer follows the steps, **Then** they can confirm the integrity of the ingested data.

### Edge Cases

- What happens if a markdown file is empty?
- How does the system handle non-markdown files in the `docs/` directory?
- What happens if the Cohere API is unavailable?
- What happens if the Qdrant or Postgres databases are unavailable?

### Assumptions

- The Docusaurus markdown files are well-formed.
- The machine running the ingestion script has the necessary credentials for Cohere, Qdrant, and Postgres.
- The machine running the ingestion script has sufficient resources to process all the content.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The system MUST parse all markdown files (`.md` and `.mdx`) from the `docs/` directory.
- **FR-002**: The system MUST chunk the parsed text into segments of 400-600 tokens.
- **FR-003**: The system MUST generate embeddings for each chunk using the Cohere Embed v3 model.
- **FR-004**: The system MUST store the generated embeddings in a Qdrant Cloud collection.
- **FR-005**: The system MUST store metadata for each chunk (e.g., source file, chunk ID) in a Neon Postgres database.
- **FR-006**: The ingestion process MUST be idempotent.
- **FR-007**: The system MUST NOT use any LLM for the ingestion process.

### Key Entities *(include if feature involves data)*

- **Document**: Represents a single markdown file from the book.
- **Chunk**: A segment of text from a document.
- **Embedding**: A vector representation of a chunk.
- **Metadata**: Information associated with a chunk, such as its source document, chunk ID, and any other relevant data.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of the book's content from the `docs/` directory is successfully parsed and chunked.
- **SC-002**: All generated embeddings are successfully stored in Qdrant Cloud.
- **SC-003**: All chunk metadata is successfully stored in Neon Postgres.
- **SC-004**: The ingestion script can be re-run without causing data duplication or corruption.
- **SC-005**: Retrieval tests consistently return relevant document sections for sample queries.
