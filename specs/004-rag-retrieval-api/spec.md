# Feature Specification: RAG Retrieval API

**Feature Branch**: `004-rag-retrieval-api`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "RAG Spec-2: Retrieve stored vectors from Qdrant and validate the ingestion pipeline"

## Context

This feature extends the existing RAG ingestion pipeline (Spec 003) by adding retrieval capabilities. The ingestion pipeline already:
- Crawls Docusaurus textbook pages
- Chunks content (~800 tokens per chunk)
- Generates embeddings via Cohere (embed-english-v3.0, 1024 dimensions)
- Stores vectors in Qdrant Cloud collection `book_embeddings`

This spec defines the retrieval layer that queries the stored vectors to find relevant content.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Query for Relevant Content (Priority: P1)

A developer or the frontend chatbot sends a natural language question about robotics/ROS 2 topics and receives the most relevant textbook chunks that can answer the question.

**Why this priority**: This is the core functionality - without it, the ingested vectors have no utility. It's the foundation for the chatbot experience.

**Independent Test**: Can be fully tested via CLI by running a query command and verifying relevant chunks are returned in JSON format.

**Acceptance Scenarios**:

1. **Given** the Qdrant collection contains ingested textbook chunks, **When** a user queries "What is ROS 2?", **Then** the system returns the top-k most semantically similar chunks with their source URLs and text.

2. **Given** a valid query string, **When** the retrieval function is called, **Then** results are returned in clean JSON format containing chunk text, source URL, title, relevance score, and chunk index.

3. **Given** no matching content exists for a query, **When** the query is executed, **Then** the system returns an empty results array (not an error).

---

### User Story 2 - Configurable Result Count (Priority: P2)

A developer can specify how many results to retrieve (top-k parameter) to balance between comprehensive coverage and response size.

**Why this priority**: Different use cases need different result counts - a chatbot might need 3-5 chunks while a research tool might need 10-20.

**Independent Test**: Can be tested by varying the top-k parameter and verifying the correct number of results are returned.

**Acceptance Scenarios**:

1. **Given** a query with top_k=5, **When** executed against a collection with 44+ chunks, **Then** exactly 5 results are returned (or fewer if total chunks < 5).

2. **Given** no top_k is specified, **When** a query is executed, **Then** a sensible default (5) is used.

---

### User Story 3 - Validate Ingestion Pipeline (Priority: P2)

A developer can run test queries to validate that the ingestion pipeline worked correctly - verifying embeddings are searchable and return semantically relevant results.

**Why this priority**: Essential for debugging and confidence in the pipeline before connecting to frontend.

**Independent Test**: Can be tested by querying known content (e.g., "URDF robot model") and verifying results come from the expected chapter.

**Acceptance Scenarios**:

1. **Given** ingested content about "ROS 2 nodes and topics", **When** querying "how do ROS nodes communicate", **Then** results include chunks from the nodes/topics chapter.

2. **Given** ingested content about "URDF humanoid robots", **When** querying "robot joint definitions", **Then** results include chunks from the URDF chapter.

---

### User Story 4 - Filter by Source URL (Priority: P3)

A developer can optionally filter results to only return chunks from specific source URLs or URL patterns.

**Why this priority**: Useful for scoping queries to specific modules/chapters, but not required for MVP.

**Independent Test**: Can be tested by filtering to a specific module URL and verifying all results match that filter.

**Acceptance Scenarios**:

1. **Given** a query with URL filter for "module-1-ros2", **When** executed, **Then** only chunks from Module 1 pages are returned.

---

### Edge Cases

- Empty query string: Return error message indicating query is required
- Query with only whitespace: Treat as empty query
- Qdrant connection failure: Return error with clear message, do not crash
- Cohere API failure: Return error with clear message, retry up to 3 times
- Very long query (>1000 characters): Truncate to reasonable length before embedding
- Special characters in query: Handle gracefully (queries are just text)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST accept a natural language query string as input
- **FR-002**: System MUST generate a query embedding using Cohere embed-english-v3.0 with `input_type="search_query"`
- **FR-003**: System MUST perform vector similarity search against the Qdrant `book_embeddings` collection
- **FR-004**: System MUST return results ranked by cosine similarity score (highest first)
- **FR-005**: System MUST include in each result: chunk text, source URL, page title, similarity score, and chunk index
- **FR-006**: System MUST support configurable top-k parameter (default: 5, range: 1-50)
- **FR-007**: System MUST return results in clean, parseable JSON format
- **FR-008**: System MUST reuse the same Cohere and Qdrant client configuration from the ingestion pipeline
- **FR-009**: System MUST handle errors gracefully and return structured error responses
- **FR-010**: System MUST support optional URL prefix filter for scoping results

### Key Entities

- **QueryRequest**: The input containing the natural language query, optional top_k, and optional URL filter
- **QueryResult**: The response containing a list of matching chunks with metadata
- **ChunkMatch**: A single matching chunk with text, url, title, score, and chunk_index

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Queries return results in under 3 seconds for typical queries (including embedding generation and vector search)
- **SC-002**: Results are correctly ranked by semantic similarity (manual validation with 5 test queries)
- **SC-003**: JSON output is valid and parseable by standard JSON parsers
- **SC-004**: System gracefully handles errors without crashing (returns structured error response)
- **SC-005**: Retrieval function can be invoked both via CLI and programmatically (importable function)

## Assumptions

- The Qdrant collection `book_embeddings` already exists and contains ingested vectors from Spec 003
- Environment variables (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY) are already configured
- The same embedding model must be used for queries as was used for document ingestion (embed-english-v3.0)
- Cohere requires `input_type="search_query"` for query embeddings (vs "search_document" for ingestion)

## Out of Scope

- Reranking with a separate reranker model (future enhancement)
- Hybrid search combining keyword and semantic search
- Caching of query results
- Authentication/authorization for the retrieval endpoint
- Rate limiting
