# Feature Specification: RAG LLM Endpoint

**Feature Branch**: `005-rag-llm-endpoint`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "RAG Spec-3: Add FastAPI RAG endpoint with LLM answer generation"

## Context

This feature extends the existing RAG pipeline (Specs 003 and 004) by adding an HTTP API endpoint that combines retrieval with LLM-powered answer generation. The existing pipeline already:
- Crawls and chunks Docusaurus textbook pages (Spec 003)
- Stores embeddings in Qdrant Cloud (Spec 003)
- Retrieves relevant chunks via `retrieve_chunks()` function (Spec 004)

This spec defines the FastAPI-based RAG endpoint that:
1. Accepts natural language questions via HTTP
2. Retrieves relevant context using the existing retrieval function
3. Sends question + context to an LLM for answer synthesis
4. Returns both the generated answer and source context

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Question and Get Answer (Priority: P1)

A developer or frontend application sends a question about Physical AI/Humanoid Robotics topics to the API and receives a synthesized answer with supporting source context.

**Why this priority**: This is the core RAG functionality - without it, users cannot get AI-generated answers from the textbook content. It completes the end-to-end RAG experience.

**Independent Test**: Can be fully tested via curl/Postman by sending a POST request to `/rag/query` with a question and verifying the response contains an answer and context chunks.

**Acceptance Scenarios**:

1. **Given** the API is running and Qdrant contains ingested textbook chunks, **When** a user sends `{"question": "What is ROS 2?"}` to `/rag/query`, **Then** the system returns a JSON response with `answer` (non-empty string), `context_chunks` (array of relevant chunks), and `metadata`.

2. **Given** a valid question, **When** the API processes the request, **Then** the answer should be grounded in the retrieved context chunks (not hallucinated from general knowledge).

3. **Given** the LLM generates an answer, **When** the response is returned, **Then** the `context_chunks` array contains the actual chunks that were used to generate the answer, allowing users to verify sources.

---

### User Story 2 - Configure Result Count (Priority: P2)

A developer can specify how many context chunks to retrieve (top_k parameter) to balance between answer comprehensiveness and response latency.

**Why this priority**: Different use cases need different context depths - quick questions might need 3 chunks while detailed explanations might need 10.

**Independent Test**: Can be tested by varying the `top_k` parameter and verifying the correct number of context chunks are returned.

**Acceptance Scenarios**:

1. **Given** a request with `{"question": "...", "top_k": 3}`, **When** processed, **Then** exactly 3 context chunks are retrieved (or fewer if total chunks < 3).

2. **Given** no `top_k` is specified, **When** a query is executed, **Then** a sensible default (5) is used.

---

### User Story 3 - View Response Metadata (Priority: P3)

A developer can see metadata about the response including the number of chunks used and the LLM model name for debugging and transparency.

**Why this priority**: Useful for debugging, monitoring, and understanding system behavior, but not required for core functionality.

**Independent Test**: Can be tested by verifying the response includes a `metadata` object with expected fields.

**Acceptance Scenarios**:

1. **Given** any successful query, **When** the response is returned, **Then** it includes `metadata` with at least `used_top_k` and `model_name` fields.

---

### Edge Cases

- Empty question string: Return 400 error with clear message indicating question is required
- Question with only whitespace: Treat as empty question (400 error)
- OpenAI API failure: Return 503 error with message, do not expose internal error details
- Qdrant/Cohere failure during retrieval: Return 503 error with message
- No relevant chunks found: Return answer indicating no relevant content was found (not an error)
- Very long question (>2000 characters): Accept but truncate internally before processing
- Missing API keys: Return 500 error on startup or 503 on request if keys become invalid

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST expose a FastAPI application with a POST endpoint at `/rag/query`
- **FR-002**: System MUST accept a JSON request body with required `question` (string) and optional `top_k` (integer, default 5)
- **FR-003**: System MUST use the existing `retrieve_chunks()` function from the retrieval module to get relevant context
- **FR-004**: System MUST construct a prompt combining the user question with retrieved context chunks
- **FR-005**: System MUST call an LLM (OpenAI GPT model) with the constructed prompt to generate an answer
- **FR-006**: System MUST return a JSON response containing `answer`, `context_chunks`, and `metadata`
- **FR-007**: System MUST return context chunks in a simplified format: `text`, `url`, `title`, and `score`
- **FR-008**: System MUST validate that `question` is non-empty and `top_k` is within valid range (1-50)
- **FR-009**: System MUST return appropriate HTTP status codes: 200 for success, 400 for invalid input, 503 for service unavailable
- **FR-010**: System MUST read API keys from environment variables (`OPENAI_API_KEY`, `COHERE_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`)

### Key Entities

- **RAGQueryRequest**: The input containing `question` (string, required) and `top_k` (integer, optional, default 5)
- **RAGQueryResponse**: The response containing `answer` (string), `context_chunks` (list), and `metadata` (object)
- **ContextChunk**: A simplified view of a retrieved chunk with `text`, `url`, `title`, and `score` fields
- **ResponseMetadata**: Contains `used_top_k` (integer) and `model_name` (string)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users receive an answer to their question within 10 seconds for typical queries (including retrieval and LLM generation)
- **SC-002**: Answers are grounded in retrieved context (manual validation with 5 test queries shows answers reference provided context)
- **SC-003**: API returns valid JSON responses that can be parsed by standard JSON parsers
- **SC-004**: System gracefully handles errors without crashing (returns structured error responses with appropriate HTTP status codes)
- **SC-005**: Context chunks in response match the chunks that were sent to the LLM (transparency/verifiability)

## Assumptions

- The Qdrant collection `book_embeddings` already exists and contains ingested vectors from Spec 003
- Environment variables (OPENAI_API_KEY, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY) are configured
- The existing `retrieve_chunks()` function from `Backend/main.py` is importable and functional
- OpenAI GPT-3.5-turbo or GPT-4 is available via the OpenAI API
- The LLM prompt will instruct the model to answer based only on provided context
- No authentication is required for the API endpoint (matches existing pattern)

## Out of Scope

- Streaming responses (SSE/WebSocket)
- Complex agent workflows or tool use
- Conversation history / multi-turn chat
- Frontend UI
- Authentication/authorization
- Rate limiting
- Caching of responses
- Reranking of retrieved chunks
- Hybrid search (keyword + semantic)

## Deliverables

1. **FastAPI Application Structure**
   - `Backend/app.py` or similar entry point with FastAPI app
   - Pydantic models for request/response validation
   - `/rag/query` POST endpoint

2. **RAG Service Function**
   - Function that orchestrates: retrieve → build prompt → call LLM → return
   - Uses existing `retrieve_chunks()` from main.py
   - Constructs appropriate prompt for LLM

3. **Manual Test Steps**
   - Start the FastAPI server
   - Send test queries via curl/Postman
   - Verify response structure and answer quality
