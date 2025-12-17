# Implementation Plan: RAG Ingestion Pipeline

**Branch**: `003-rag-ingestion-pipeline` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-rag-ingestion-pipeline/spec.md`

## Summary

Implement a single-file Python backend (`main.py`) that crawls the deployed Docusaurus textbook, extracts and chunks content, generates embeddings via Cohere, and stores vectors in Qdrant Cloud. The implementation uses UV for package management and follows a simple function-based architecture.

## Technical Context

**Language/Version**: Python 3.11+
**Package Manager**: UV (modern Python package manager)
**Primary Dependencies**: cohere, qdrant-client, httpx, beautifulsoup4
**Storage**: Qdrant Cloud (free tier) - Collection: `book_embeddings`
**Testing**: pytest (minimal - script-based execution)
**Target Platform**: Local execution / CLI script
**Project Type**: Single script backend
**Performance Goals**: Process 50-100 pages in under 10 minutes
**Constraints**: Single configured domain restriction, 3 retries max for failures
**Scale/Scope**: ~50-100 textbook pages, estimated <1GB vectors

**Deployed Site**: `ai-native-book-1fj5bdpkr-nida-nasars-projects.vercel.app`
**Sitemap URL**: ai-native-book-1fj5bdpkr-nida-nasars-projects.vercel.app/sitemap.xml

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Accuracy First | ✅ PASS | Using documented APIs (Cohere, Qdrant) - no hallucinated capabilities |
| II. AI-Native Pedagogy | ✅ PASS | Supports RAG chatbot for AI-native learning |
| III. Clarity & Accessibility | ✅ PASS | Clear function names, single-file simplicity |
| IV. Embodied Intelligence Focus | N/A | Backend infrastructure, not content |
| V. Reproducibility & Traceability | ✅ PASS | Documented dependencies, environment config |

**Gate Result**: PASS - No violations requiring justification.

## Project Structure

### Documentation (this feature)

```text
specs/003-rag-ingestion-pipeline/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
└── tasks.md             # Phase 2 output (via /sp.tasks)
```

### Source Code (repository root)

```text
Backend/
├── main.py              # Single implementation file
├── pyproject.toml       # UV package configuration
├── .env.example         # Environment variable template
└── .env                 # Local credentials (gitignored)
```

**Structure Decision**: Single-file architecture per user requirement. All functions in `main.py` with execution via `main()` function.

## Implementation Architecture

### Function Flow

```text
main()
  ├── get_all_urls(base_url) → List[str]
  │     └── Crawl sitemap or known paths to discover all page URLs
  │
  ├── For each URL:
  │     ├── extract_text_from_url(url) → str
  │     │     └── Fetch HTML, parse with BeautifulSoup, extract article content
  │     │
  │     ├── chunk_text(text, chunk_size, overlap) → List[str]
  │     │     └── Split text into overlapping chunks
  │     │
  │     ├── embed(chunks) → List[List[float]]
  │     │     └── Generate embeddings via Cohere API
  │     │
  │     └── save_chunk_qdrant(chunks, embeddings, metadata)
  │           └── Upsert vectors to Qdrant collection "book_embeddings"
  │
  └── Report results
```

### External Service Configuration

| Service | Purpose | Configuration |
|---------|---------|---------------|
| Cohere | Embedding generation | `COHERE_API_KEY` env var |
| Qdrant Cloud | Vector storage | `QDRANT_URL`, `QDRANT_API_KEY` env vars |

### Key Design Decisions

1. **Single File**: All code in `main.py` for simplicity and easy execution
2. **UV Package Manager**: Modern, fast Python package management
3. **Chunk Identity**: URL + chunk index (per clarification)
4. **Domain Restriction**: Single base domain (`ai-native-book-1fj5bdpkr-nida-nasars-projects.vercel.app`)
5. **Retry Policy**: 3 retries with exponential backoff for external service failures
6. **Collection Name**: `book_embeddings` (per user specification)

## Dependencies

```toml
[project]
dependencies = [
    "cohere>=5.0.0",
    "qdrant-client>=1.7.0",
    "httpx>=0.25.0",
    "beautifulsoup4>=4.12.0",
    "python-dotenv>=1.0.0",
]
```

## Environment Variables

```bash
# .env.example
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
BASE_URL=https://ai-native-book-1fj5bdpkr-nida-nasars-projects.vercel.app
```

## Complexity Tracking

No constitution violations requiring justification. Single-file architecture is the simplest viable approach for this use case.
