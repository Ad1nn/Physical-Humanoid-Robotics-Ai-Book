# Research: RAG Knowledge Ingestion & Embedding

This document outlines the key decisions made during the planning phase of the RAG knowledge ingestion and embedding feature.

## Chunking Strategy

**Decision**: Chunks will be between 400 and 600 tokens with an overlap of 80-100 tokens.

**Rationale**: This size is a good balance between providing enough context for the embedding model and not being too large to slow down retrieval. The overlap helps to ensure that semantic context is not lost between chunks.

**Alternatives considered**:
- Smaller chunks: Might lose important context.
- Larger chunks: Could be less precise for retrieval and hit context window limits.
- No overlap: Risks splitting sentences and losing semantic meaning at the boundaries.

## Embedding Model

**Decision**: We will use the Cohere Embed v3 model as specified in the project constraints.

**Rationale**: This model is a powerful and widely used embedding model that is well-suited for this task.

**Alternatives considered**:
- Other embedding models (e.g., from OpenAI or Hugging Face): The project has a constraint to use Cohere Embed v3.

## Storage Schema

### Qdrant (Vector Store)

**Decision**: A single collection will be created in Qdrant to store the embeddings. Each vector will be associated with a unique chunk ID.

**Rationale**: This is a simple and effective way to store and retrieve embeddings.

### Neon Postgres (Metadata Store)

**Decision**: A table will be created to store the metadata for each chunk. The schema will be:

| Column      | Type      | Description                               |
|-------------|-----------|-------------------------------------------|
| `chunk_id`  | `UUID`    | Primary Key, unique identifier for the chunk |
| `doc_path`  | `TEXT`    | Path to the source markdown file          |
| `chunk_text`| `TEXT`    | The actual text of the chunk              |
| `created_at`| `TIMESTAMPTZ` | Timestamp of when the chunk was created   |

**Rationale**: This schema provides all the necessary information to trace a chunk back to its source and understand its content.

## Idempotency and Re-ingestion Strategy

**Decision**: The ingestion script will be idempotent. It will first check if a document has been modified since the last ingestion. If a document has been modified, all its existing chunks will be deleted and re-ingested.

**Rationale**: This strategy ensures that the data is always up-to-date and prevents data duplication.

**Alternatives considered**:
- Deleting all data and re-ingesting everything: This is inefficient and not scalable.
- Checking each chunk individually: This would be too complex to implement and maintain.

## Handling of Special Content

**Decision**:
- **Code blocks**: The content of code blocks will be preserved and included in the chunks.
- **Images and other non-text elements**: These will be ignored during the parsing phase.
- **Special characters**: These will be kept as part of the text.

**Rationale**: The goal is to embed the textual content of the book. Code is an important part of the content, but images are not relevant for the RAG assistant at this stage.
