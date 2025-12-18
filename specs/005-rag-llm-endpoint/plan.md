# Implementation Plan: RAG LLM Endpoint

**Branch**: `005-rag-llm-endpoint` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/005-rag-llm-endpoint/spec.md`

## Summary

Build a FastAPI endpoint (`POST /rag/query`) that accepts natural language questions, retrieves relevant context using existing Spec-2 retrieval functions, calls OpenAI LLM to generate grounded answers, and returns the answer with source context chunks.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**: FastAPI, Pydantic, OpenAI SDK, existing Cohere/Qdrant from main.py
**Storage**: N/A (uses existing Qdrant collection from Spec-3/4)
**Testing**: Manual API testing via curl/httpie, pytest for unit tests
**Target Platform**: Linux/Windows server (local development)
**Project Type**: Backend API extension
**Performance Goals**: <10s response time for typical queries (including LLM call)
**Constraints**: Requires OPENAI_API_KEY, reuses existing COHERE/QDRANT credentials
**Scale/Scope**: Single endpoint, synchronous processing

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Accuracy First | PASS | Uses existing verified retrieval; LLM grounded in retrieved context |
| II. AI-Native Pedagogy | PASS | RAG approach aligns with AI-native learning (context-grounded answers) |
| III. Clarity & Accessibility | PASS | Simple API contract, clear request/response models |
| IV. Embodied Intelligence Focus | N/A | Backend infrastructure, not robotics content |
| V. Reproducibility & Traceability | PASS | Context chunks returned with answers enable verification |

**Gate Status**: PASS - No violations

## Project Structure

### Documentation (this feature)

```text
specs/005-rag-llm-endpoint/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (OpenAPI spec)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
Backend/
├── main.py              # Existing: ingestion + retrieval (Spec 3-4)
├── app.py               # NEW: FastAPI application entry point
├── api/
│   ├── __init__.py
│   ├── routes.py        # NEW: /rag/query endpoint
│   └── schemas.py       # NEW: Pydantic request/response models
├── rag/
│   ├── __init__.py
│   └── service.py       # NEW: RAG orchestration logic
├── config.py            # NEW: Centralized configuration
└── .env                 # Existing: API keys (add OPENAI_API_KEY)
```

**Structure Decision**: Extend existing Backend/ with modular FastAPI structure. Keep main.py for CLI ingestion/retrieval, add app.py for HTTP API.

## Complexity Tracking

No violations - simple extension of existing codebase.

---

## Phase 1: Files and Structure

| File | Purpose |
|------|---------|
| `Backend/app.py` | FastAPI application instance, include routers, startup config |
| `Backend/api/routes.py` | Define `POST /rag/query` endpoint |
| `Backend/api/schemas.py` | Pydantic models: `RagQueryRequest`, `RagQueryResponse`, `ContextChunk` |
| `Backend/rag/service.py` | RAG orchestration: retrieve → build prompt → call LLM → return |
| `Backend/config.py` | Centralized env config (reuse existing + add OpenAI) |

**Key Integration**: Import `retrieve_chunks()` from `main.py` - no duplication of retrieval logic.

## Phase 2: Models and Functions

### Pydantic Models (`api/schemas.py`)

```python
class RagQueryRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=2000)
    top_k: int | None = Field(default=5, ge=1, le=50)

class ContextChunk(BaseModel):
    text: str
    url: str
    title: str
    chunk_index: int
    score: float

class ResponseMetadata(BaseModel):
    used_top_k: int
    model_name: str

class RagQueryResponse(BaseModel):
    answer: str
    context_chunks: list[ContextChunk]
    metadata: ResponseMetadata
```

### RAG Service Function (`rag/service.py`)

```python
def answer_question(question: str, top_k: int = 5) -> RagQueryResponse:
    """
    Main RAG function: question → retrieve → LLM → answer

    Steps:
    1. Validate question (non-empty)
    2. Call retrieve_chunks(question, top_k) from main.py
    3. Build prompt: system instruction + context chunks + user question
    4. Call OpenAI chat completion
    5. Return RagQueryResponse with answer + context + metadata
    """
```

### Prompt Template

```text
You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
Answer the user's question based ONLY on the provided context.
If the context doesn't contain relevant information, say so.

Context:
{formatted_chunks}

Question: {question}

Answer:
```

## Phase 3: Endpoint

### Route Definition (`api/routes.py`)

```python
router = APIRouter(prefix="/rag", tags=["RAG"])

@router.post("/query", response_model=RagQueryResponse)
def query_rag(request: RagQueryRequest) -> RagQueryResponse:
    """
    RAG query endpoint.

    Flow:
    1. Extract question and top_k from request
    2. Call answer_question(question, top_k)
    3. Return RagQueryResponse

    Errors:
    - 400: Invalid request (empty question, invalid top_k)
    - 503: Service unavailable (LLM/retrieval failure)
    """
    return answer_question(request.question, request.top_k or 5)
```

### App Setup (`app.py`)

```python
from fastapi import FastAPI
from api.routes import router

app = FastAPI(title="RAG API", version="1.0.0")
app.include_router(router)
```

## Phase 4: Configuration

### Environment Variables

| Variable | Source | Purpose |
|----------|--------|---------|
| `COHERE_API_KEY` | Existing (.env) | Query embedding generation |
| `QDRANT_URL` | Existing (.env) | Vector store connection |
| `QDRANT_API_KEY` | Existing (.env) | Vector store auth |
| `OPENAI_API_KEY` | NEW (add to .env) | LLM answer generation |
| `OPENAI_MODEL_NAME` | NEW (optional, default: gpt-3.5-turbo) | LLM model selection |

### Config Module (`config.py`)

```python
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    cohere_api_key: str
    qdrant_url: str
    qdrant_api_key: str
    openai_api_key: str
    openai_model_name: str = "gpt-3.5-turbo"

    class Config:
        env_file = ".env"

settings = Settings()
```

## Phase 5: Testing

### Manual Test Steps

1. **Start server**:
   ```bash
   cd Backend
   uvicorn app:app --reload --port 8000
   ```

2. **Test queries**:
   ```bash
   # Basic query
   curl -X POST http://localhost:8000/rag/query \
     -H "Content-Type: application/json" \
     -d '{"question": "What is ROS 2?"}'

   # With custom top_k
   curl -X POST http://localhost:8000/rag/query \
     -H "Content-Type: application/json" \
     -d '{"question": "How do humanoid robots maintain balance?", "top_k": 3}'

   # Edge case: empty question (should return 400)
   curl -X POST http://localhost:8000/rag/query \
     -H "Content-Type: application/json" \
     -d '{"question": ""}'
   ```

3. **Verify response structure**:
   - Status: 200 OK
   - Body contains: `answer` (non-empty), `context_chunks` (array), `metadata`
   - `context_chunks` have: `text`, `url`, `title`, `score`
   - Answer is grounded in context (manual inspection)

### Automated Tests (Future)

```text
tests/
├── test_schemas.py      # Pydantic model validation
├── test_service.py      # RAG service unit tests (mocked LLM)
└── test_routes.py       # API endpoint integration tests
```
