# Tasks: RAG Retrieval API

**Input**: Design documents from `/specs/004-rag-retrieval-api/`
**Prerequisites**: plan.md (complete), spec.md (complete)
**Branch**: `004-rag-retrieval-api`

**Tests**: Manual testing only (no automated tests requested in spec)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files/sections, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- All paths relative to repository root

## Path Conventions

- **Target file**: `Backend/main.py` (single file extension per user request)
- All new code is added to existing file in designated sections

---

## Phase 1: Setup

**Purpose**: Add retrieval constants and imports to existing file

- [x] T001 Add retrieval constants (DEFAULT_TOP_K=5, MAX_TOP_K=50, MAX_QUERY_LENGTH=1000) to Backend/main.py after line 45
- [x] T002 Add `import json` to existing imports section in Backend/main.py

---

## Phase 2: Foundational (Data Model)

**Purpose**: Add RetrievedChunk dataclass that all retrieval functions depend on

**⚠️ CRITICAL**: No retrieval function can be implemented until this phase is complete

- [x] T003 Add `@dataclass RetrievedChunk` (text, url, title, chunk_index, score) to Data Models section in Backend/main.py after IngestionResult class

**Checkpoint**: Data model ready - retrieval functions can now be implemented

---

## Phase 3: User Story 1 - Query for Relevant Content (Priority: P1) 🎯 MVP

**Goal**: Accept a natural language query and return top-k relevant chunks as JSON

**Independent Test**: Run `uv run python main.py --query "What is ROS 2?"` and verify JSON response with relevant chunks

### Implementation for User Story 1

- [x] T004 [P] [US1] Add `generate_query_embedding(query, cohere_client) -> list[float]` function with input_type="search_query" in Backend/main.py
- [x] T005 [P] [US1] Add `search_qdrant(query_vector, qdrant_client, top_k) -> list[ScoredPoint]` function in Backend/main.py
- [x] T006 [US1] Add `retrieve_chunks(query, cohere_client, qdrant_client, top_k) -> list[RetrievedChunk]` function composing T004+T005 in Backend/main.py
- [x] T007 [US1] Add `format_results_json(query, results, duration_ms) -> str` function for clean JSON output in Backend/main.py
- [x] T008 [US1] Add error handling for empty/whitespace query (return error dict) in retrieve_chunks function
- [x] T009 [US1] Add retry decorator to generate_query_embedding for Cohere API resilience in Backend/main.py
- [x] T010 [US1] Extend main() CLI to handle `--query <text>` argument in Backend/main.py
- [x] T011 [US1] Add `run_query(query, top_k)` orchestration function that initializes clients and calls retrieve_chunks in Backend/main.py
- [x] T012 [US1] Manual test: Run `uv run python main.py --query "What is ROS 2?"` and verify JSON output with results

**Checkpoint**: User Story 1 complete - basic query→JSON retrieval works

---

## Phase 4: User Story 2 - Configurable Result Count (Priority: P2)

**Goal**: Support --top-k parameter to control number of results

**Independent Test**: Run `uv run python main.py --query "..." --top-k 10` and verify exactly 10 results returned

### Implementation for User Story 2

- [x] T013 [US2] Extend CLI argument parsing to accept `--top-k <int>` with default=5 in Backend/main.py
- [x] T014 [US2] Add validation for top_k range (1-50) in run_query function, clamp to bounds
- [x] T015 [US2] Manual test: Run queries with --top-k 1, --top-k 10, --top-k 50 and verify correct counts

**Checkpoint**: User Story 2 complete - configurable top-k works

---

## Phase 5: User Story 3 - Validate Ingestion Pipeline (Priority: P2)

**Goal**: Interactive mode for running multiple test queries to validate ingestion

**Independent Test**: Run `uv run python main.py --interactive` and enter test queries

### Implementation for User Story 3

- [x] T016 [US3] Add `--interactive` flag to CLI argument parsing in Backend/main.py
- [x] T017 [US3] Add `run_interactive_mode(cohere_client, qdrant_client)` function with REPL loop in Backend/main.py
- [x] T018 [US3] Add `print_results_formatted(results)` function for human-readable CLI output in Backend/main.py
- [x] T019 [US3] Add collection info display (chunk count) at interactive mode startup
- [x] T020 [US3] Manual test: Run 5 validation queries in interactive mode per plan.md test plan

**Checkpoint**: User Story 3 complete - interactive validation mode works

---

## Phase 6: User Story 4 - Filter by Source URL (Priority: P3)

**Goal**: Optionally filter results to specific URL patterns

**Independent Test**: Run query with `--url-filter "module-1"` and verify all results are from Module 1

### Implementation for User Story 4

- [x] T021 [US4] Add `url_filter` parameter to search_qdrant function with Qdrant Filter in Backend/main.py
- [x] T022 [US4] Add `url_filter` parameter to retrieve_chunks function (pass through to search_qdrant)
- [x] T023 [US4] Extend CLI to accept `--url-filter <prefix>` argument in Backend/main.py
- [x] T024 [US4] Update run_query to pass url_filter to retrieve_chunks
- [x] T025 [US4] Manual test: Run `uv run python main.py --query "nodes" --url-filter "module-1"` and verify all results from Module 1

**Checkpoint**: User Story 4 complete - URL filtering works

---

## Phase 7: Polish & Validation

**Purpose**: Edge cases, final validation, documentation

- [x] T026 Add query length truncation (>1000 chars) in generate_query_embedding function
- [x] T027 Add graceful handling for Qdrant connection errors in search_qdrant function
- [x] T028 Update module docstring at top of Backend/main.py to include retrieval usage examples
- [x] T029 Final validation: Run all 5 test queries from plan.md and verify semantic relevance
- [x] T030 Verify JSON output is valid with `python -c "import json; json.loads(...)"` test

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational - MVP deliverable
- **User Story 2 (Phase 4)**: Depends on US1 (extends CLI parsing)
- **User Story 3 (Phase 5)**: Depends on US1 (uses retrieve_chunks)
- **User Story 4 (Phase 6)**: Depends on US1 (extends search_qdrant)
- **Polish (Phase 7)**: Depends on all user stories

### User Story Dependencies

```
Phase 1 (Setup) → Phase 2 (Foundational)
                         ↓
                  Phase 3 (US1) ← MVP STOP POINT
                    ↓    ↓    ↓
                 US2   US3   US4  (can run in parallel after US1)
                    ↓    ↓    ↓
                  Phase 7 (Polish)
```

### Parallel Opportunities

**Within Phase 3 (US1)**:
- T004 and T005 can run in parallel (different functions)

**After US1 Complete**:
- US2, US3, US4 can all start in parallel (different aspects of retrieval)

---

## Parallel Example: User Story 1 Core Functions

```bash
# Launch in parallel (different functions, no dependencies):
Task T004: "Add generate_query_embedding function"
Task T005: "Add search_qdrant function"

# Then sequential:
Task T006: "Add retrieve_chunks function" (depends on T004, T005)
Task T007: "Add format_results_json function"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T002)
2. Complete Phase 2: Foundational (T003)
3. Complete Phase 3: User Story 1 (T004-T012)
4. **STOP and VALIDATE**: `uv run python main.py --query "What is ROS 2?"`
5. If MVP works → can proceed to US2/US3/US4 or deploy

### Full Implementation

1. Setup → Foundational → US1 (MVP)
2. US2 (top-k config)
3. US3 (interactive mode)
4. US4 (URL filtering)
5. Polish (edge cases, docs)

---

## Summary

| Phase | Tasks | Description |
|-------|-------|-------------|
| Setup | T001-T002 | Constants and imports |
| Foundational | T003 | RetrievedChunk dataclass |
| US1 (P1) 🎯 | T004-T012 | Core query→JSON retrieval (MVP) |
| US2 (P2) | T013-T015 | Configurable top-k |
| US3 (P2) | T016-T020 | Interactive validation mode |
| US4 (P3) | T021-T025 | URL prefix filtering |
| Polish | T026-T030 | Edge cases, validation |

**Total Tasks**: 30
**MVP Scope**: T001-T012 (12 tasks)
**Parallel Opportunities**: T004+T005, US2+US3+US4 after US1

---

## Notes

- All code goes in single file: `Backend/main.py`
- Reuses existing Cohere/Qdrant client initialization from ingestion code
- Key difference for retrieval: `input_type="search_query"` (vs "search_document")
- Manual testing via CLI (no pytest requested)
- Stop at US1 checkpoint for working MVP
