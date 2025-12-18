# Research: RAG LLM Endpoint

**Feature**: 005-rag-llm-endpoint
**Date**: 2025-12-17

## Technology Decisions

### 1. LLM Provider: OpenAI

**Decision**: Use OpenAI GPT models via the official `openai` Python SDK

**Rationale**:
- Industry standard, well-documented API
- Consistent quality for text generation
- Simple integration with Python
- Supports structured prompts for RAG use cases

**Alternatives Considered**:
| Option | Pros | Cons | Rejected Because |
|--------|------|------|------------------|
| Anthropic Claude | High quality, long context | Additional API key management | User specified OpenAI in requirements |
| Local LLM (Ollama) | No API costs, privacy | Requires GPU, slower, setup complexity | Adds deployment complexity |
| Cohere Generate | Already using Cohere for embeddings | Less established for chat/RAG | OpenAI explicitly requested |

**Model Selection**: `gpt-3.5-turbo` (default), configurable via `OPENAI_MODEL_NAME`
- Cost-effective for RAG queries
- Sufficient quality for textbook Q&A
- Fast response times (<3s typical)
- User can upgrade to `gpt-4` via config if needed

### 2. Web Framework: FastAPI

**Decision**: FastAPI with Pydantic models

**Rationale**:
- Automatic OpenAPI documentation
- Built-in request/response validation via Pydantic
- Async support (future enhancement)
- Industry standard for Python APIs

**Alternatives Considered**:
| Option | Pros | Cons | Rejected Because |
|--------|------|------|------------------|
| Flask | Simpler, lightweight | Manual validation, no auto-docs | FastAPI provides better DX |
| Django REST | Full-featured | Overkill for single endpoint | Unnecessary complexity |

### 3. Retrieval Integration

**Decision**: Import and reuse `retrieve_chunks()` from existing `main.py`

**Rationale**:
- DRY principle - no code duplication
- Existing function is tested and working (Spec-4)
- Maintains consistency with CLI retrieval

**Integration Pattern**:
```python
from main import retrieve_chunks, RetrievedChunk
```

### 4. Prompt Engineering

**Decision**: Simple context-grounded prompt template

**Rationale**:
- RAG best practice: provide context explicitly
- Instruction to answer "ONLY based on context" reduces hallucination
- Clear separation: system instruction → context → question

**Prompt Structure**:
```
[System]: You are a helpful assistant. Answer based ONLY on provided context.
[Context]: {retrieved_chunks}
[User]: {question}
```

**Context Formatting**:
- Each chunk prefixed with source info: `[Source: {title}]\n{text}`
- Chunks separated by blank lines for clarity
- Total context limited by LLM context window (~4000 tokens for gpt-3.5-turbo)

### 5. Error Handling Strategy

**Decision**: Structured error responses with appropriate HTTP status codes

| Scenario | Status Code | Response |
|----------|-------------|----------|
| Empty/invalid question | 400 | `{"detail": "Question is required"}` |
| OpenAI API failure | 503 | `{"detail": "LLM service unavailable"}` |
| Retrieval failure | 503 | `{"detail": "Retrieval service unavailable"}` |
| No relevant chunks | 200 | Answer indicates no relevant content found |

### 6. Configuration Management

**Decision**: Pydantic Settings with `.env` file

**Rationale**:
- Type-safe configuration
- Automatic environment variable parsing
- Validation on startup
- Consistent with Python best practices

## Open Questions (Resolved)

| Question | Resolution |
|----------|------------|
| Which LLM model? | gpt-3.5-turbo (default), configurable |
| Sync or async? | Sync for MVP, async available via FastAPI |
| How to handle long questions? | Truncate to 2000 chars (already in spec) |
| Response streaming? | Out of scope per spec |

## Dependencies

### New Dependencies (add to requirements.txt)

```
fastapi>=0.104.0
uvicorn[standard]>=0.24.0
openai>=1.0.0
pydantic-settings>=2.0.0
```

### Existing Dependencies (from Spec 3-4)

```
cohere
qdrant-client
python-dotenv
httpx
beautifulsoup4
tenacity
```

## References

- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [OpenAI Python SDK](https://github.com/openai/openai-python)
- [RAG Best Practices](https://www.pinecone.io/learn/retrieval-augmented-generation/)
