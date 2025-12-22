# Implementation Plan: RAG Retrieval API

**Branch**: `004-rag-retrieval-api` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-rag-retrieval-api/spec.md`

## Summary

Extend the existing `Backend/main.py` with retrieval capabilities: accept a natural language query, generate a query embedding via Cohere (`input_type="search_query"`), perform vector similarity search against Qdrant, and return top-k relevant chunks as clean JSON.

## Technical Context

**Language/Version**: Python 3.11+ (same as Spec 1)
**Primary Dependencies**: cohere, qdrant-client, python-dotenv (already in pyproject.toml)
**Storage**: Qdrant Cloud collection `book_embeddings` (already exists from Spec 1)
**Testing**: Manual CLI testing + optional pytest
**Target Platform**: CLI / importable module
**Project Type**: Single file extension (Backend/main.py)
**Performance Goals**: <3 seconds per query (including embedding + search)
**Constraints**: Reuse existing Cohere/Qdrant client setup from Spec 1

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Accuracy First | PASS | Reuses same verified APIs from Spec 1 |
| II. AI-Native Pedagogy | N/A | Backend infrastructure, not content |
| III. Clarity & Accessibility | PASS | Simple CLI interface with clear output |
| IV. Embodied Intelligence Focus | N/A | Backend infrastructure, not content |
| V. Reproducibility & Traceability | PASS | CLI commands are reproducible, JSON output is traceable |

**Gate Status**: PASS - No violations

---

## 1. Project Structure

Extend the existing `Backend/main.py` file (single-file architecture, per user request).

| Location | Purpose |
|----------|---------|
| `Backend/main.py` | Add retrieval functions alongside existing ingestion code |
| `Backend/.env` | Reuse existing config (no changes needed) |
| `Backend/pyproject.toml` | No new dependencies required |

**New sections to add to main.py**:

```text
# -----------------------------------------------------------------------------
# Data Models (add to existing section)
# -----------------------------------------------------------------------------
@dataclass RetrievedChunk      # Query result with text, score, metadata

# -----------------------------------------------------------------------------
# Retrieval Functions (new section)
# -----------------------------------------------------------------------------
generate_query_embedding()     # Query → embedding vector
search_qdrant()                # Vector → top-k hits
retrieve_chunks()              # Main public function: query → results
format_results_json()          # Results → clean JSON string

# -----------------------------------------------------------------------------
# CLI Entry Point (extend existing)
# -----------------------------------------------------------------------------
main()                         # Add --query and --top-k flags
```

---

## 2. Retrieval API and Data Model

### Data Model

```python
@dataclass
class RetrievedChunk:
    """A chunk returned from vector search."""
    text: str           # The chunk content
    url: str            # Source page URL
    title: str          # Page title
    chunk_index: int    # Position in source document
    score: float        # Cosine similarity score (0.0 to 1.0)
```

### Public Functions

```python
def generate_query_embedding(
    query: str,
    cohere_client: cohere.Client
) -> list[float]:
    """Generate embedding for a search query.

    Args:
        query: Natural language query string
        cohere_client: Initialized Cohere client

    Returns:
        1024-dimension embedding vector

    Note: Uses input_type="search_query" (vs "search_document" for ingestion)
    """

def search_qdrant(
    query_vector: list[float],
    qdrant_client: QdrantClient,
    top_k: int = 5,
    url_filter: Optional[str] = None
) -> list[models.ScoredPoint]:
    """Search Qdrant for similar vectors.

    Args:
        query_vector: 1024-dim embedding from generate_query_embedding
        qdrant_client: Initialized Qdrant client
        top_k: Number of results to return (1-50)
        url_filter: Optional URL prefix to filter results

    Returns:
        List of ScoredPoint with payload and score
    """

def retrieve_chunks(
    query: str,
    cohere_client: cohere.Client,
    qdrant_client: QdrantClient,
    top_k: int = 5,
    url_filter: Optional[str] = None
) -> list[RetrievedChunk]:
    """Main retrieval function: query → relevant chunks.

    Args:
        query: Natural language query
        cohere_client: Initialized Cohere client
        qdrant_client: Initialized Qdrant client
        top_k: Number of results (default 5)
        url_filter: Optional URL prefix filter

    Returns:
        List of RetrievedChunk sorted by relevance (highest first)
    """

def format_results_json(results: list[RetrievedChunk]) -> str:
    """Format retrieval results as clean JSON.

    Returns JSON structure:
    {
        "query": "...",
        "count": N,
        "results": [
            {
                "rank": 1,
                "score": 0.85,
                "url": "https://...",
                "title": "...",
                "chunk_index": 0,
                "text": "..."
            },
            ...
        ]
    }
    """
```

### Qdrant Result → RetrievedChunk Conversion

```python
# Inside search_qdrant or retrieve_chunks:
for point in qdrant_results:
    chunk = RetrievedChunk(
        text=point.payload["text"],
        url=point.payload["url"],
        title=point.payload["title"],
        chunk_index=point.payload["chunk_index"],
        score=point.score  # Cosine similarity from Qdrant
    )
```

---

## 3. Qdrant Search Behavior

| Setting | Value | Notes |
|---------|-------|-------|
| Collection | `book_embeddings` | Reuse from Spec 1 |
| Distance metric | Cosine | Matches Spec 1 ingestion |
| Default top_k | 5 | Configurable 1-50 |
| Vector size | 1024 | Cohere embed-english-v3.0 |

### Search Logic

```python
qdrant_client.search(
    collection_name=COLLECTION_NAME,
    query_vector=query_vector,
    limit=top_k,
    query_filter=models.Filter(...) if url_filter else None
)
```

### Result Ordering

Results are returned by Qdrant in descending score order (best match first). No additional sorting needed.

### Error Handling

| Scenario | Behavior |
|----------|----------|
| Empty query | Return error dict `{"error": "Query is required"}` |
| Whitespace-only query | Treat as empty query |
| No results found | Return `{"query": "...", "count": 0, "results": []}` |
| Qdrant connection error | Log error, return `{"error": "Search failed: <message>"}` |
| Cohere API error | Retry 3x with backoff, then return error dict |
| Query > 1000 chars | Truncate to 1000 chars before embedding |

---

## 4. CLI / Manual Test Harness

### CLI Modes

Extend `main.py` CLI with new flags:

```bash
# Existing ingestion modes (unchanged):
uv run python main.py                     # Ingest from sitemap
uv run python main.py --url <url>         # Ingest single URL
uv run python main.py --urls <file>       # Ingest from file

# New retrieval modes:
uv run python main.py --query "What is ROS 2?"           # Query with default top_k=5
uv run python main.py --query "..." --top-k 10           # Query with custom top_k
uv run python main.py --query "..." --url-filter "module-1"  # Filter by URL
uv run python main.py --interactive                      # Interactive REPL mode
```

### Interactive Mode Flow

```text
$ uv run python main.py --interactive

RAG Retrieval CLI (type 'exit' to quit, 'help' for commands)
Collection: book_embeddings (44 chunks)

> What is ROS 2?

Results (top 5, 0.8s):

[1] Score: 0.847 | module-1-ros2/01-introduction-ros2
    "ROS 2 (Robot Operating System 2) is middleware that provides..."

[2] Score: 0.812 | module-1-ros2/02-nodes-topics-services
    "In ROS 2, nodes are the fundamental building blocks..."

[3] Score: 0.798 | module-1-ros2/03-python-rclpy-bridge
    "The rclpy library provides Python bindings for ROS 2..."

> exit
Goodbye!
```

### Single Query Mode Output

```json
{
  "query": "What is ROS 2?",
  "count": 5,
  "duration_ms": 823,
  "results": [
    {
      "rank": 1,
      "score": 0.847,
      "url": "https://ai-native-book.../docs/module-1-ros2/01-introduction-ros2",
      "title": "Introduction to ROS 2",
      "chunk_index": 0,
      "text": "ROS 2 (Robot Operating System 2) is middleware that provides..."
    }
  ]
}
```

---

## 5. Configuration

### Existing Config (Reused)

| Variable | Source | Used By |
|----------|--------|---------|
| `COHERE_API_KEY` | `.env` | Query embedding generation |
| `QDRANT_URL` | `.env` | Vector search |
| `QDRANT_API_KEY` | `.env` | Vector search |
| `COLLECTION_NAME` | Constant | `book_embeddings` |
| `EMBEDDING_MODEL` | Constant | `embed-english-v3.0` |

### New Config (Optional)

| Variable | Default | Purpose |
|----------|---------|---------|
| `RETRIEVAL_DEFAULT_TOP_K` | `5` | Default number of results |
| `RETRIEVAL_MAX_TOP_K` | `50` | Maximum allowed top_k |
| `RETRIEVAL_MAX_QUERY_LENGTH` | `1000` | Truncate queries beyond this |

These can be added as constants in main.py (no .env changes required for MVP).

---

## 6. Testing & Validation

### Preconditions

- Spec 1 ingestion has populated `book_embeddings` collection
- Environment variables configured in `Backend/.env`
- 44 chunks available (from previous ingestion run)

### Manual Test Plan

Run these queries and verify relevance:

| Query | Expected Top Result Contains |
|-------|------------------------------|
| "What is ROS 2?" | Introduction chapter, ROS 2 definition |
| "How do nodes communicate in ROS?" | Nodes/topics chapter, pub/sub pattern |
| "What is URDF?" | URDF chapter, robot description format |
| "digital twin simulation" | Module 2, Gazebo content |
| "humanoid robot joints" | URDF chapter, joint definitions |

### Validation Criteria

1. Top result is from expected chapter (semantic relevance)
2. Scores decrease from rank 1 → 5 (proper ordering)
3. JSON output is valid and parseable
4. Response time < 3 seconds
5. No crashes on edge cases (empty query, special chars)

### Optional Automated Test

```python
# tests/test_retrieval.py
def test_retrieval_returns_results():
    """Smoke test: known query returns at least one result."""
    cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
    qdrant_client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))

    results = retrieve_chunks("What is ROS 2?", cohere_client, qdrant_client)

    assert len(results) > 0
    assert results[0].score > 0.5
    assert "ros" in results[0].text.lower() or "ros" in results[0].title.lower()
```

---

## Implementation Order

1. **Add data model**: `RetrievedChunk` dataclass
2. **Add `generate_query_embedding()`**: Cohere with `input_type="search_query"`
3. **Add `search_qdrant()`**: Vector search with optional filter
4. **Add `retrieve_chunks()`**: Compose embedding + search
5. **Add `format_results_json()`**: Clean JSON output
6. **Extend CLI**: Add `--query`, `--top-k`, `--interactive` flags
7. **Test manually**: Run 5 validation queries
8. **Optional**: Add pytest smoke test

---

## Files Modified

| File | Changes |
|------|---------|
| `Backend/main.py` | Add retrieval section (~150 lines) |
| `Backend/pyproject.toml` | No changes needed |
| `Backend/.env` | No changes needed |

---

## Complexity Tracking

No violations - single file extension with minimal new code.
