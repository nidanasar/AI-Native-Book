---
id: 002
title: RAG LLM Endpoint Implementation Plan
stage: plan
date: 2025-12-17
surface: agent
model: claude-opus-4-5-20251101
feature: 005-rag-llm-endpoint
branch: 004-rag-retrieval-api
user: nida.nasarr
command: /sp.plan
labels: ["rag", "fastapi", "openai", "planning", "architecture"]
links:
  spec: specs/005-rag-llm-endpoint/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-rag-llm-endpoint/plan.md
  - specs/005-rag-llm-endpoint/research.md
  - specs/005-rag-llm-endpoint/data-model.md
  - specs/005-rag-llm-endpoint/contracts/openapi.yaml
  - specs/005-rag-llm-endpoint/quickstart.md
tests:
  - N/A (planning phase)
---

## Prompt

RAG Spec-3: implementation planning for FastAPI RAG endpoint

You are planning how to implement Spec-3 (RAG API + LLM answer) based on this:

- FastAPI endpoint
- Input: question (+ optional top_k)
- Use existing retrieval from Spec-2
- Call LLM with question + retrieved chunks
- Return answer + context chunks

Make a concise, implementation-ready plan.

## 1. Files / structure

List concrete files and one-line purpose, for example:
- `backend/app/main.py`        – create FastAPI app and include routers
- `backend/app/api/routes.py`  – define `/rag/query` endpoint
- `backend/app/api/schemas.py` – Pydantic models for request/response
- `backend/app/rag/service.py` – RAG logic: question → retrieve → LLM → answer

State that retrieval is imported from the Spec-2 module (no duplication).

## 2. Models and functions

Define:
- `RagQueryRequest` (question: str, top_k: int | None)
- `ContextChunk` (content, url, title, section_heading, chunk_index, score)
- `RagQueryResponse` (answer: str, context_chunks: list[ContextChunk])

Define main function in `rag/service.py`, e.g.:
- `async def answer_question(question: str, top_k: int | None = None) -> RagQueryResponse`

Briefly list its steps: validate question → call `retrieve_chunks` → build prompt → call LLM → return response.

## 3. Endpoint

Specify:
- `@router.post("/rag/query", response_model=RagQueryResponse)`
- Flow: parse `RagQueryRequest` → call `answer_question` → return result.
- Mention router is included in `main.py` (e.g., `app.include_router(router)`).

## 4. Config

Mention env/config keys needed:
- reuse existing config for Qdrant/Cohere
- add `OPENAI_API_KEY`, `OPENAI_MODEL_NAME`
- how `rag/service.py` reads them (shared `config.py`).

## 5. Testing

Plan quick tests:
- run app with uvicorn
- send POST `/rag/query` with 2–3 sample questions
- verify: 200 status, non-empty `answer`, non-empty `context_chunks`.

Output the plan in Markdown with these 5 sections, short and concrete so it can be turned directly into tasks and code.

## Response snapshot

Created comprehensive implementation plan with 5 Phase 1 artifacts:

**1. plan.md** - Complete implementation plan with:
- 5 files structure (app.py, api/routes.py, api/schemas.py, rag/service.py, config.py)
- Pydantic models: RagQueryRequest, ContextChunk, ResponseMetadata, RagQueryResponse
- RAG service function: validate → retrieve → prompt → LLM → return
- Endpoint: POST /rag/query with router pattern
- Config via pydantic-settings + .env
- Manual testing steps with curl examples

**2. research.md** - Technology decisions:
- LLM: OpenAI GPT (gpt-3.5-turbo default, configurable)
- Framework: FastAPI with Pydantic
- Retrieval: Import from existing main.py (no duplication)
- Prompt: Context-grounded template
- Error handling: Structured responses with HTTP status codes

**3. data-model.md** - Entity definitions:
- Request/Response models with field constraints
- Mapping from RetrievedChunk to ContextChunk
- Data flow diagram: Request → Retrieve → Generate → Response

**4. contracts/openapi.yaml** - Full OpenAPI 3.1 spec:
- POST /rag/query endpoint
- All schemas with examples
- Error responses (400, 503)

**5. quickstart.md** - Developer guide:
- Setup steps (install, configure, run)
- Usage examples (curl, httpie, Python)
- Expected response format
- Troubleshooting table

## Outcome

- Impact: Complete Phase 1 planning artifacts ready for task generation
- Tests: N/A (planning phase)
- Files: 5 files created (plan.md, research.md, data-model.md, openapi.yaml, quickstart.md)
- Next prompts: `/sp.tasks` to generate implementation tasks
- Reflection: Clear spec made planning straightforward; existing retrieval code provides solid foundation

## Evaluation notes (flywheel)

- Failure modes observed: None - user provided clear requirements and structure guidance
- Graders run and results (PASS/FAIL): Manual review PASS - all required sections filled, constitution check passed
- Prompt variant (if applicable): Standard /sp.plan workflow with user-provided structure
- Next experiment (smallest change to try): Proceed to task generation
