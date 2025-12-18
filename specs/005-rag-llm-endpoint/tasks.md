# Tasks: RAG LLM Endpoint

**Input**: Design documents from `/specs/005-rag-llm-endpoint/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/openapi.yaml, quickstart.md

**Tests**: Manual testing only (no automated tests requested in spec)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md, this feature extends the existing `Backend/` directory:

```
Backend/
├── main.py              # Existing (Spec 3-4)
├── app.py               # NEW: FastAPI entry
├── api/
│   ├── __init__.py
│   ├── routes.py        # NEW: /rag/query endpoint
│   └── schemas.py       # NEW: Pydantic models
├── rag/
│   ├── __init__.py
│   └── service.py       # NEW: RAG orchestration
└── config.py            # NEW: Configuration
```

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and dependency installation

- [x] T001 Install new dependencies: `fastapi`, `uvicorn[standard]`, `openai`, `pydantic-settings` in Backend/
- [x] T002 [P] Create `Backend/api/__init__.py` package file
- [x] T003 [P] Create `Backend/rag/__init__.py` package file
- [x] T004 Add `GEMINI_API_KEY` to `Backend/.env` file (using Gemini free tier)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 Create `Backend/config.py` with Settings class using pydantic-settings (Gemini config)
- [x] T006 Create `Backend/api/schemas.py` with all Pydantic models: RagQueryRequest, ContextChunk, ResponseMetadata, RagQueryResponse
- [x] T007 Create `Backend/app.py` with FastAPI app instance, health check endpoint, router included

**Checkpoint**: Foundation ready - user story implementation can begin

---

## Phase 3: User Story 1 - Ask Question and Get Answer (Priority: P1) 🎯 MVP

**Goal**: User sends a question to `/rag/query` and receives an AI-generated answer with source context

**Independent Test**:
```bash
curl -X POST http://localhost:8000/rag/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
# Verify: 200 status, non-empty answer, context_chunks array present
```

### Implementation for User Story 1

- [x] T008 [US1] Create prompt template constant in `Backend/rag/service.py` (system instruction for context-grounded answering)
- [x] T009 [US1] Implement `format_context()` helper in `Backend/rag/service.py` to format RetrievedChunk list as context string
- [x] T010 [US1] Implement `call_llm()` function in `Backend/rag/service.py` using Gemini via OpenAI SDK
- [x] T011 [US1] Implement `answer_question()` main function in `Backend/rag/service.py` (orchestrates: retrieve → format → llm → response)
- [x] T012 [US1] Create `Backend/api/routes.py` with APIRouter and `POST /rag/query` endpoint calling answer_question()
- [x] T013 [US1] Update `Backend/app.py` to include the router from routes.py
- [x] T014 [US1] Add error handling in routes.py: 400 for empty question, 503 for LLM/retrieval failures with HTTPException

**Checkpoint**: User Story 1 complete - basic RAG Q&A functional with default top_k=5

---

## Phase 4: User Story 2 - Configure Result Count (Priority: P2)

**Goal**: User can specify `top_k` parameter to control number of context chunks retrieved

**Independent Test**:
```bash
curl -X POST http://localhost:8000/rag/query \
  -H "Content-Type: application/json" \
  -d '{"question": "How do humanoid robots balance?", "top_k": 3}'
# Verify: exactly 3 chunks in context_chunks array
```

### Implementation for User Story 2

- [x] T015 [US2] Update `answer_question()` in `Backend/rag/service.py` to pass top_k parameter to retrieve_chunks()
- [x] T016 [US2] Add validation in `Backend/api/schemas.py` RagQueryRequest: top_k range 1-50, default 5
- [x] T017 [US2] Update endpoint in `Backend/api/routes.py` to extract and pass top_k from request

**Checkpoint**: User Story 2 complete - top_k parameter works correctly

---

## Phase 5: User Story 3 - View Response Metadata (Priority: P3)

**Goal**: Response includes metadata with used_top_k and model_name for debugging/transparency

**Independent Test**:
```bash
curl -X POST http://localhost:8000/rag/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
# Verify: response.metadata.used_top_k = 5, response.metadata.model_name = "gpt-3.5-turbo"
```

### Implementation for User Story 3

- [x] T018 [US3] Update `answer_question()` in `Backend/rag/service.py` to populate ResponseMetadata with actual used_top_k and model_name from config
- [x] T019 [US3] Ensure RagQueryResponse in `Backend/api/schemas.py` correctly includes metadata field (already defined in T006)

**Checkpoint**: User Story 3 complete - metadata included in all responses

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Edge cases, error handling improvements, documentation

- [x] T020 [P] Add edge case handling in `Backend/rag/service.py`: empty question validation, whitespace-only check
- [x] T021 [P] Add edge case handling in `Backend/rag/service.py`: no relevant chunks found (return helpful message)
- [x] T022 [P] Add logging statements in `Backend/rag/service.py` for debugging (question received, chunks retrieved, LLM called)
- [ ] T023 Run manual validation per `specs/005-rag-llm-endpoint/quickstart.md` test steps
- [ ] T024 Verify API documentation at http://localhost:8000/docs matches OpenAPI contract

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1: Setup
    ↓
Phase 2: Foundational (BLOCKS all user stories)
    ↓
┌───────────────────────────────────────────┐
│  User Stories can proceed in parallel     │
│  or sequentially by priority              │
├───────────────────────────────────────────┤
│  Phase 3: US1 (P1) - Core RAG Q&A  🎯 MVP │
│  Phase 4: US2 (P2) - top_k parameter      │
│  Phase 5: US3 (P3) - Response metadata    │
└───────────────────────────────────────────┘
    ↓
Phase 6: Polish
```

### User Story Dependencies

| Story | Depends On | Can Start After |
|-------|------------|-----------------|
| US1 (P1) | Foundational (Phase 2) | T007 complete |
| US2 (P2) | US1 | T014 complete (endpoint exists) |
| US3 (P3) | US1 | T014 complete (endpoint exists) |

**Note**: US2 and US3 can be done in parallel after US1 is complete, as they modify different parts of the response.

### Within Each Phase

- Tasks marked [P] can run in parallel
- Tasks without [P] must run sequentially as listed
- For US1: T008-T010 can run in parallel, then T011, then T012-T014 sequentially

---

## Parallel Opportunities

### Phase 1 Parallel Group
```bash
# These can run simultaneously:
Task T002: Create Backend/api/__init__.py
Task T003: Create Backend/rag/__init__.py
```

### Phase 6 Parallel Group
```bash
# These can run simultaneously:
Task T020: Edge case - empty question
Task T021: Edge case - no relevant chunks
Task T022: Add logging
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T004)
2. Complete Phase 2: Foundational (T005-T007)
3. Complete Phase 3: User Story 1 (T008-T014)
4. **STOP and VALIDATE**: Test with curl per Independent Test
5. Deploy/demo if ready - basic RAG Q&A works!

### Full Feature

1. Complete MVP (Phases 1-3)
2. Add Phase 4: User Story 2 (T015-T017) - configurable top_k
3. Add Phase 5: User Story 3 (T018-T019) - response metadata
4. Complete Phase 6: Polish (T020-T024)

---

## Task Summary

| Phase | Task Count | Parallel Tasks |
|-------|------------|----------------|
| Setup | 4 | 2 |
| Foundational | 3 | 0 |
| US1 (MVP) | 7 | 0 |
| US2 | 3 | 0 |
| US3 | 2 | 0 |
| Polish | 5 | 3 |
| **Total** | **24** | **5** |

### By User Story

| User Story | Tasks | Core Files |
|------------|-------|------------|
| US1 - Core Q&A | T008-T014 (7) | rag/service.py, api/routes.py, app.py |
| US2 - top_k | T015-T017 (3) | rag/service.py, api/schemas.py, api/routes.py |
| US3 - metadata | T018-T019 (2) | rag/service.py |

---

## Notes

- All file paths are relative to `Backend/` directory
- Import `retrieve_chunks` from `main.py` - do NOT reimplement retrieval
- Use `config.settings` for all API keys and model configuration
- Commit after each completed user story for clean git history
- Run server with: `cd Backend && uvicorn app:app --reload --port 8000`
