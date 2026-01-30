# Quickstart: RAG Knowledge Ingestion & Embedding

This guide explains how to run the RAG knowledge ingestion and embedding pipeline.

## Prerequisites

1.  **Python 3.10+**: Make sure you have Python 3.10 or a later version installed.
2.  **Environment Variables**: Create a `.env` file in the root of the project and add the following environment variables:
    ```
    COHERE_API_KEY=...
    QDRANT_URL=...
    QDRANT_API_KEY=...
    POSTGRES_URL=...
    ```
3.  **Dependencies**: Install the required Python packages:
    ```bash
    pip install -r ingestion/requirements.txt
    ```

## Running the Ingestion Script

To run the ingestion pipeline, execute the following command from the root of the project:

```bash
python -m ingestion.main
```

The script will:
1.  Discover all markdown files in the `docs/` directory.
2.  Parse the files and create chunks.
3.  Generate embeddings for the chunks using the Cohere API.
4.  Store the embeddings in Qdrant.
5.  Store the metadata in Neon Postgres.

## Validation

After the script has finished, you can validate the ingestion by:
1.  Checking the number of vectors in the Qdrant collection.
2.  Checking the number of rows in the `chunks` table in the Postgres database.
3.  Running the validation script (to be created).
