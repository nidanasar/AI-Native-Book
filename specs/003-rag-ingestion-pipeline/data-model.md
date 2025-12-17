# Data Model: RAG Ingestion Pipeline

**Feature**: 003-rag-ingestion-pipeline
**Date**: 2025-12-17

## Overview

This document defines the data structures used in the RAG ingestion pipeline. Since this is a single-file script without a traditional database, entities are represented as Python dataclasses and Qdrant payloads.

## Entities

### 1. Document

Represents a single web page being ingested.

```python
@dataclass
class Document:
    url: str              # Source URL (unique identifier)
    title: str            # Page title extracted from HTML
    content: str          # Raw text content after HTML parsing
    fetched_at: datetime  # When the page was fetched
```

**Validation Rules**:
- `url` MUST be a valid HTTPS URL
- `url` MUST belong to the configured base domain
- `title` MUST NOT be empty (use URL slug as fallback)
- `content` MAY be empty (page with no text)

### 2. Chunk

A semantically meaningful piece of a document, stored in Qdrant.

```python
@dataclass
class Chunk:
    id: str                    # Deterministic: hash(url + chunk_index)
    url: str                   # Parent document URL
    chunk_index: int           # Position in document (0-based)
    text: str                  # Chunk text content
    embedding: list[float]     # 1024-dim Cohere embedding vector
    title: str                 # Parent document title
    ingested_at: datetime      # When chunk was stored
```

**Validation Rules**:
- `id` MUST be deterministic (same input → same ID for upsert)
- `chunk_index` MUST be >= 0
- `text` MUST NOT be empty
- `embedding` MUST have exactly 1024 dimensions (Cohere embed-english-v3.0)

**Identity Rule** (per clarification):
- Chunk uniqueness: `url` + `chunk_index`
- Re-ingestion replaces all chunks for that URL

### 3. IngestionResult

Tracks the outcome of processing a single URL.

```python
@dataclass
class IngestionResult:
    url: str                   # Processed URL
    status: str                # "success" | "failed" | "skipped"
    chunk_count: int           # Number of chunks created
    error: str | None          # Error message if failed
    duration_ms: int           # Processing time
```

**Status Values**:
- `success`: URL processed and chunks stored
- `failed`: Processing error (network, API, etc.)
- `skipped`: No content to process (empty page)

## Qdrant Collection Schema

### Collection: `book_embeddings`

```python
# Collection configuration
{
    "name": "book_embeddings",
    "vectors": {
        "size": 1024,           # Cohere embed-english-v3.0
        "distance": "Cosine"
    }
}
```

### Point Structure (stored in Qdrant)

```python
{
    "id": int,                  # Deterministic hash of url+chunk_index
    "vector": [float, ...],     # 1024-dimensional embedding
    "payload": {
        "url": str,             # Source URL
        "title": str,           # Page title
        "chunk_index": int,     # Position in document
        "text": str,            # Original chunk text (for retrieval display)
        "ingested_at": str      # ISO 8601 timestamp
    }
}
```

### Payload Index Configuration

For efficient filtering during retrieval (future feature):

```python
client.create_payload_index(
    collection_name="book_embeddings",
    field_name="url",
    field_schema="keyword"
)
```

## ID Generation Strategy

Chunk IDs must be deterministic for upsert behavior:

```python
import hashlib

def generate_chunk_id(url: str, chunk_index: int) -> int:
    """Generate deterministic ID for Qdrant point."""
    key = f"{url}_{chunk_index}"
    # Use first 16 hex chars of SHA256, convert to int
    hash_hex = hashlib.sha256(key.encode()).hexdigest()[:16]
    return int(hash_hex, 16)
```

**Properties**:
- Same URL + chunk_index → Same ID (deterministic)
- Collision probability: Negligible for <1M chunks
- Fits Qdrant's u64 ID type

## Relationships

```
Document (1) ──────< Chunk (many)
    │                    │
    │ url                │ url (foreign key)
    │                    │ chunk_index (position)
    │                    │
    └── content ───────> └── text (derived via chunking)
```

## State Transitions

### Document Processing States

```
URL Submitted
     │
     ▼
[Fetching] ──error──> Failed
     │
     ▼ success
[Parsing] ──error──> Failed
     │
     ▼ success
[Empty?] ──yes──> Skipped
     │
     ▼ no
[Chunking]
     │
     ▼
[Embedding] ──error──> Failed (retry 3x)
     │
     ▼ success
[Upserting] ──error──> Failed
     │
     ▼ success
Completed
```

## Data Volume Estimates

| Metric | Estimate |
|--------|----------|
| Total pages | 50-100 |
| Avg chunks per page | 5-10 |
| Total chunks | 250-1000 |
| Embedding size | 1024 × 4 bytes = 4KB per chunk |
| Total vector storage | 1-4 MB |
| Payload storage | ~500 bytes per chunk |
| Total Qdrant storage | <10 MB |

Well within Qdrant Cloud free tier (1GB limit).
