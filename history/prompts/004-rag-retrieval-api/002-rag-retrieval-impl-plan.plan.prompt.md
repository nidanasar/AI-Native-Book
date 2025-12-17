---
id: 002
title: RAG Retrieval Implementation Plan
stage: plan
date: 2025-12-17
surface: agent
model: claude-opus-4-5-20251101
feature: 004-rag-retrieval-api
branch: 004-rag-retrieval-api
user: nida.nasarr
command: /sp.plan
labels: ["rag", "retrieval", "qdrant", "cohere", "implementation-plan"]
links:
  spec: specs/004-rag-retrieval-api/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-rag-retrieval-api/plan.md
tests:
  - Constitution check passed (all 5 principles)
---

## Prompt

RAG Spec-2: implementation planning for retrieval from Qdrant and validation

You are planning how to implement Spec-2, based on this spec:

- Implement a retrieval layer and test harness that:
  - Takes a natural-language query as input
  - Generates an embedding using the same Cohere model from Spec 1
  - Performs vector similarity search against Qdrant
  - Returns the top-k most relevant chunks with their metadata
  - Allows manual testing to confirm that ingestion + retrieval are working correctly

Create a simple, implementation-ready plan with clear structure and function names.

## Response snapshot

Created comprehensive implementation plan covering all 6 requested sections:

**1. Project Structure**: Single-file extension to Backend/main.py (per user request)

**2. Retrieval API & Data Model**:
- `RetrievedChunk` dataclass: text, url, title, chunk_index, score
- Functions: `generate_query_embedding()`, `search_qdrant()`, `retrieve_chunks()`, `format_results_json()`
- Key: Use `input_type="search_query"` for Cohere (vs "search_document" for ingestion)

**3. Qdrant Search Behavior**:
- Reuse `book_embeddings` collection with cosine distance
- Default top_k=5, configurable 1-50
- Error handling for empty queries, connection failures, API errors

**4. CLI / Manual Test Harness**:
- New flags: `--query`, `--top-k`, `--url-filter`, `--interactive`
- Interactive REPL mode for testing
- JSON output format for single queries

**5. Configuration**: Reuse existing env vars, add optional constants

**6. Testing & Validation**:
- 5 manual test queries with expected results
- Optional pytest smoke test

**Implementation Order**: 8 steps from data model → CLI → testing

## Outcome

- Impact: Created actionable implementation plan for RAG retrieval extending Spec 1 pipeline
- Tests: Constitution check PASS on all applicable principles
- Files: 1 file created (plan.md)
- Next prompts: `/sp.tasks` to generate implementation tasks, then implement
- Reflection: Plan is simple and focused - single file extension with ~150 lines of new code

## Evaluation notes (flywheel)

- Failure modes observed: None - clear requirements from user prompt
- Graders run and results (PASS/FAIL): Constitution check PASS
- Prompt variant (if applicable): User provided detailed 6-section structure
- Next experiment (smallest change to try): N/A - plan complete and ready for tasks
