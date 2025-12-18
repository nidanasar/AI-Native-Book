# Data Model: RAG LLM Endpoint

**Feature**: 005-rag-llm-endpoint
**Date**: 2025-12-17

## Entities

### 1. RagQueryRequest

**Purpose**: Incoming API request for RAG query

| Field | Type | Required | Constraints | Description |
|-------|------|----------|-------------|-------------|
| `question` | string | Yes | 1-2000 chars | Natural language question |
| `top_k` | integer | No | 1-50, default: 5 | Number of context chunks to retrieve |

**Validation Rules**:
- `question` must not be empty or whitespace-only
- `question` is trimmed before processing
- `top_k` is clamped to valid range if out of bounds

### 2. ContextChunk

**Purpose**: Simplified view of a retrieved chunk for API response

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `text` | string | Yes | Chunk text content |
| `url` | string | Yes | Source URL of the original document |
| `title` | string | Yes | Page title from source |
| `chunk_index` | integer | Yes | Index of chunk within source document |
| `score` | float | Yes | Relevance score (0.0-1.0, higher = more relevant) |

**Mapping from RetrievedChunk** (main.py):
```
RetrievedChunk.text       → ContextChunk.text
RetrievedChunk.url        → ContextChunk.url
RetrievedChunk.title      → ContextChunk.title
RetrievedChunk.chunk_index → ContextChunk.chunk_index
RetrievedChunk.score      → ContextChunk.score
```

### 3. ResponseMetadata

**Purpose**: Metadata about the RAG response for debugging/transparency

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `used_top_k` | integer | Yes | Actual number of chunks retrieved |
| `model_name` | string | Yes | LLM model used for answer generation |

### 4. RagQueryResponse

**Purpose**: Complete API response containing answer and context

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `answer` | string | Yes | LLM-generated answer to the question |
| `context_chunks` | list[ContextChunk] | Yes | Retrieved chunks used for context |
| `metadata` | ResponseMetadata | Yes | Response metadata |

## Entity Relationships

```
RagQueryRequest (input)
        │
        ▼
┌───────────────────┐
│  RAG Service      │
│  - retrieve       │
│  - build prompt   │
│  - call LLM       │
└───────────────────┘
        │
        ▼
RagQueryResponse (output)
├── answer: string
├── context_chunks: list[ContextChunk]
│   └── (mapped from RetrievedChunk)
└── metadata: ResponseMetadata
```

## State Transitions

This feature is stateless - no persistent state changes. Each request is independent:

```
Request → Validate → Retrieve → Generate → Response
```

## Data Flow

```
1. API receives RagQueryRequest
   ├── question: "What is ROS 2?"
   └── top_k: 5

2. Retrieval (uses existing main.py)
   ├── Generate query embedding (Cohere)
   ├── Search Qdrant collection
   └── Return list[RetrievedChunk]

3. Prompt Construction
   ├── Format chunks as context
   └── Build: system + context + question

4. LLM Generation (OpenAI)
   ├── Send prompt to GPT model
   └── Receive generated answer

5. Response Assembly
   ├── Map RetrievedChunk → ContextChunk
   ├── Build ResponseMetadata
   └── Return RagQueryResponse
```

## Existing Entities (from Spec 3-4)

These entities exist in `main.py` and are reused:

### RetrievedChunk (main.py:106-113)

```python
@dataclass
class RetrievedChunk:
    text: str
    url: str
    title: str
    chunk_index: int
    score: float
```

**Used by**: `retrieve_chunks()` function returns `list[RetrievedChunk]`
