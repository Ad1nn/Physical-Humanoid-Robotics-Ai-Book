# Data Model: RAG Knowledge Ingestion & Embedding

This document describes the data model for the metadata store (Neon Postgres) for the RAG knowledge ingestion and embedding feature.

## Entities

### Chunk

Represents a chunk of text from a source document.

**Fields**:

| Column      | Type      | Description                               |
|-------------|-----------|-------------------------------------------|
| `chunk_id`  | `UUID`    | Primary Key, unique identifier for the chunk |
| `doc_path`  | `TEXT`    | Path to the source markdown file          |
| `chunk_text`| `TEXT`    | The actual text of the chunk              |
| `created_at`| `TIMESTAMPTZ` | Timestamp of when the chunk was created   |

**Relationships**:

- A `Chunk` belongs to a `Document` (implicitly, via `doc_path`).

## Schema Definition (SQL)

```sql
CREATE TABLE chunks (
    chunk_id UUID PRIMARY KEY,
    doc_path TEXT NOT NULL,
    chunk_text TEXT NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_chunks_doc_path ON chunks (doc_path);
```
