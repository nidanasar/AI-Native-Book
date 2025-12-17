# Feature Specification: RAG Ingestion Pipeline

**Feature Branch**: `003-rag-ingestion-pipeline`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "RAG Spec-1: Crawl Docusaurus book URLs, generate embeddings with Cohere, and store in Qdrant"

## Clarifications

### Session 2025-12-17

- Q: URL Domain Restriction - How should domain allowlist be configured? → A: Restrict to single configured base domain (e.g., `textbook.example.com`)
- Q: Retry Policy - How many retries for external service failures? → A: 3 retries max with exponential backoff (~30 seconds total wait)
- Q: Chunk Identity - How are chunks uniquely identified for upsert? → A: URL + chunk index (position-based; re-ingestion replaces all chunks for that URL)

## Overview

This feature implements a content ingestion pipeline that prepares the Physical AI & Humanoid Robotics Docusaurus textbook for a RAG (Retrieval-Augmented Generation) chatbot. The pipeline crawls deployed book pages, extracts and chunks textual content, generates vector embeddings, and stores them in a vector database with rich metadata for efficient retrieval.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ingest Single URL (Priority: P1)

As a content administrator, I want to submit a single Docusaurus page URL so that its content becomes searchable by the RAG chatbot.

**Why this priority**: This is the fundamental building block. Without single-URL ingestion working correctly, batch processing cannot function. It validates the entire pipeline end-to-end with minimal complexity.

**Independent Test**: Can be fully tested by submitting one textbook chapter URL and verifying the content appears as searchable vectors in the database with correct metadata.

**Acceptance Scenarios**:

1. **Given** a valid Docusaurus page URL and configured credentials, **When** I submit the URL for ingestion, **Then** the system extracts the page content, chunks it, generates embeddings, and stores them in the vector database with page metadata (URL, title, section).
2. **Given** a URL that has already been ingested, **When** I submit the same URL again, **Then** the system updates existing vectors rather than creating duplicates.
3. **Given** an invalid or inaccessible URL, **When** I submit it for ingestion, **Then** the system returns a clear error message explaining why ingestion failed.

---

### User Story 2 - Batch URL Ingestion (Priority: P2)

As a content administrator, I want to submit multiple URLs at once so that I can efficiently ingest entire sections or the whole textbook.

**Why this priority**: After single-URL works, batch processing enables practical use for the full textbook. This is essential for initial setup and bulk updates.

**Independent Test**: Can be tested by submitting a list of 5-10 chapter URLs and verifying all are processed with progress feedback and final summary.

**Acceptance Scenarios**:

1. **Given** a list of valid Docusaurus page URLs, **When** I submit the batch for ingestion, **Then** the system processes each URL and reports progress and final results (success count, failure count, errors).
2. **Given** a batch where some URLs fail, **When** processing completes, **Then** successful URLs are ingested and failed URLs are reported with specific error reasons without stopping the entire batch.
3. **Given** a large batch of URLs, **When** processing is in progress, **Then** I can see incremental progress updates.

---

### User Story 3 - Content Re-ingestion (Priority: P3)

As a content administrator, I want to re-ingest previously processed URLs so that updated textbook content is reflected in the chatbot's knowledge base.

**Why this priority**: Content updates are inevitable. The system must handle updates gracefully to maintain accuracy without manual cleanup.

**Independent Test**: Can be tested by modifying a test page, re-ingesting its URL, and verifying the vector database reflects the updated content.

**Acceptance Scenarios**:

1. **Given** a URL that was previously ingested and has since been updated, **When** I trigger re-ingestion, **Then** old vectors for that URL are replaced with new ones based on current content.
2. **Given** a request to re-ingest all previously ingested URLs, **When** I trigger bulk re-ingestion, **Then** the system processes all known URLs and updates their vectors.

---

### User Story 4 - Ingestion Status Check (Priority: P3)

As a content administrator, I want to check what content has been ingested so that I can verify coverage and identify gaps.

**Why this priority**: Visibility into what's been processed is important for administration but not blocking for core functionality.

**Independent Test**: Can be tested by ingesting several URLs and then querying status to verify the list matches what was submitted.

**Acceptance Scenarios**:

1. **Given** content has been ingested, **When** I request ingestion status, **Then** the system returns a list of ingested URLs with metadata (ingestion date, chunk count, status).

---

### Edge Cases

- What happens when a Docusaurus page has no textual content (only images/videos)?
  - System logs a warning and skips the page without failing the batch.
- How does the system handle extremely long pages that exceed chunking limits?
  - Content is split into multiple chunks with overlap to preserve context continuity.
- What happens when the embedding service is temporarily unavailable?
  - System retries with exponential backoff (3 retries max, ~30 seconds total) and reports failure after max retries.
- How does the system handle non-English content (e.g., future Urdu translation)?
  - Embedding model selection should support multilingual content; initial implementation focuses on English.
- What happens when the vector database connection fails mid-batch?
  - Partial progress is preserved; failed items are reported for retry.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST accept one or more valid HTTP/HTTPS URLs pointing to Docusaurus-rendered pages.
- **FR-002**: System MUST extract clean textual content from HTML pages, removing navigation, headers, footers, and non-content elements.
- **FR-003**: System MUST chunk extracted content into semantically meaningful pieces with configurable chunk size (default: 500-1000 tokens) and overlap (default: 10-20%).
- **FR-004**: System MUST generate vector embeddings for each chunk using the configured embedding model.
- **FR-005**: System MUST store vectors in the configured vector database with metadata including: source URL, page title, section heading, chunk index, ingestion timestamp.
- **FR-006**: System MUST handle duplicate URLs by updating existing vectors rather than creating duplicates (upsert behavior).
- **FR-007**: System MUST provide clear error messages for failed operations including: invalid URLs, network errors, embedding failures, database errors.
- **FR-008**: System MUST support batch processing of multiple URLs in a single request.
- **FR-009**: System MUST continue processing remaining URLs when individual URLs fail in a batch.
- **FR-010**: System MUST provide progress feedback during batch operations.
- **FR-011**: System MUST validate URLs before processing (format validation, single configured base domain restriction).
- **FR-012**: System MUST log all ingestion operations for debugging and audit purposes.
- **FR-013**: System MUST securely handle API credentials for embedding service and vector database (no hardcoded secrets).

### Key Entities

- **Document**: Represents a single ingested URL; contains source URL, title, raw content, ingestion status, timestamps.
- **Chunk**: A semantically meaningful piece of a document; uniquely identified by source URL + chunk index; contains text content, embedding vector, position metadata, parent document reference.
- **IngestionJob**: Tracks a batch ingestion request; contains list of URLs, status per URL, overall progress, start/end timestamps.
- **Metadata**: Structured information attached to each chunk; includes source URL, page title, section heading, chunk index, ingestion timestamp.

## Assumptions

- The Docusaurus book is deployed and publicly accessible at a known base URL.
- The Docusaurus pages use standard HTML structure with content in predictable containers.
- Cohere embedding API is available and the free tier quota is sufficient for initial ingestion.
- Qdrant Cloud free tier provides adequate storage for the textbook content (estimated <1GB vectors).
- Network connectivity to external services (Cohere, Qdrant Cloud) is reliable.
- Content is primarily English text; code blocks and technical notation should be preserved.

## Non-Goals (Out of Scope)

- User authentication/authorization for the ingestion API (admin-only internal tool initially).
- Automatic crawling/discovery of URLs (URLs must be explicitly provided).
- Real-time content change detection and automatic re-ingestion.
- Support for non-HTML content formats (PDF, video, images).
- The RAG query/retrieval functionality (separate feature).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Single URL ingestion completes in under 30 seconds for typical textbook pages.
- **SC-002**: Batch ingestion of 50 URLs completes in under 10 minutes.
- **SC-003**: 100% of successfully ingested pages produce searchable vectors in the database.
- **SC-004**: Re-ingestion of updated content reflects changes within one processing cycle.
- **SC-005**: System handles network failures gracefully with automatic retry and clear error reporting.
- **SC-006**: Zero hardcoded secrets in the codebase; all credentials loaded from environment configuration.
- **SC-007**: Ingestion pipeline can process the entire textbook (estimated 50-100 pages) without manual intervention.

## Dependencies

- Deployed Docusaurus textbook (Frontend)
- Cohere API account and credentials
- Qdrant Cloud account and collection
- Python FastAPI backend environment
