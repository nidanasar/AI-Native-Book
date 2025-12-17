# Research: RAG Ingestion Pipeline

**Feature**: 003-rag-ingestion-pipeline
**Date**: 2025-12-17

## 1. UV Package Manager

**Decision**: Use UV for Python package management

**Rationale**:
- 10-100x faster than pip for dependency resolution
- Built-in virtual environment management
- Lock file support for reproducible builds
- Single tool replaces pip, pip-tools, virtualenv

**Alternatives Considered**:
- pip + venv: Slower, no lock file by default
- Poetry: More complex, heavier for simple projects
- Pipenv: Slower, less active development

**Usage Pattern**:
```bash
# Initialize project
uv init

# Add dependencies
uv add cohere qdrant-client httpx beautifulsoup4 python-dotenv

# Run script
uv run python main.py
```

## 2. Cohere Embeddings

**Decision**: Use Cohere `embed-english-v3.0` model

**Rationale**:
- Free tier: 1,000 API calls/month (sufficient for initial ingestion)
- High-quality English embeddings (1024 dimensions)
- Simple SDK with retry support built-in
- Good documentation and Python support

**Alternatives Considered**:
- OpenAI embeddings: More expensive, no free tier
- Sentence-transformers (local): Slower, requires GPU for performance
- Voyage AI: Less documentation, newer service

**API Pattern**:
```python
import cohere

co = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))

response = co.embed(
    texts=["chunk text here"],
    model="embed-english-v3.0",
    input_type="search_document"  # for stored documents
)
embeddings = response.embeddings
```

**Key Configuration**:
- `input_type="search_document"` for ingestion
- `input_type="search_query"` for retrieval (future feature)
- Batch size: Up to 96 texts per request

## 3. Qdrant Cloud

**Decision**: Use Qdrant Cloud free tier with `book_embeddings` collection

**Rationale**:
- Free tier: 1GB storage, sufficient for textbook
- Managed service, no infrastructure to maintain
- Native upsert support for duplicate handling
- Excellent Python SDK with async support

**Alternatives Considered**:
- Pinecone: Free tier more limited
- Weaviate: More complex setup
- ChromaDB (local): No persistence without extra setup
- pgvector: Requires PostgreSQL infrastructure

**Collection Configuration**:
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Create collection (idempotent)
client.recreate_collection(
    collection_name="book_embeddings",
    vectors_config=VectorParams(
        size=1024,  # Cohere embed-english-v3.0 dimension
        distance=Distance.COSINE
    )
)
```

**Point Structure**:
```python
from qdrant_client.models import PointStruct

point = PointStruct(
    id=hash(f"{url}_{chunk_index}"),  # Deterministic ID for upsert
    vector=embedding,
    payload={
        "url": url,
        "title": page_title,
        "chunk_index": chunk_index,
        "text": chunk_text,
        "ingested_at": datetime.utcnow().isoformat()
    }
)
```

## 4. HTML Content Extraction

**Decision**: Use BeautifulSoup4 with Docusaurus-specific selectors

**Rationale**:
- Docusaurus uses predictable HTML structure
- BeautifulSoup is mature, well-documented
- Easy to target article content, skip navigation

**Docusaurus Content Selectors**:
```python
from bs4 import BeautifulSoup

def extract_text_from_url(url: str) -> str:
    response = httpx.get(url)
    soup = BeautifulSoup(response.text, "html.parser")

    # Docusaurus main content container
    article = soup.find("article") or soup.find("main")

    # Remove navigation, sidebar, footer
    for element in article.find_all(["nav", "aside", "footer", "script", "style"]):
        element.decompose()

    return article.get_text(separator="\n", strip=True)
```

## 5. Text Chunking Strategy

**Decision**: Token-based chunking with overlap

**Rationale**:
- 500-800 tokens per chunk (optimal for retrieval)
- 10-15% overlap preserves context at boundaries
- Simple splitting by paragraphs where possible

**Implementation**:
```python
def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> list[str]:
    """Split text into overlapping chunks."""
    words = text.split()
    chunks = []
    start = 0

    while start < len(words):
        end = start + chunk_size
        chunk = " ".join(words[start:end])
        chunks.append(chunk)
        start = end - overlap  # Overlap with previous chunk

    return chunks
```

## 6. URL Discovery

**Decision**: Parse Docusaurus sitemap.xml

**Rationale**:
- Docusaurus auto-generates sitemap at `/sitemap.xml`
- Contains all published pages
- More reliable than crawling links

**Implementation**:
```python
def get_all_urls(base_url: str) -> list[str]:
    """Fetch all URLs from Docusaurus sitemap."""
    sitemap_url = f"{base_url}/sitemap.xml"
    response = httpx.get(sitemap_url)
    soup = BeautifulSoup(response.text, "xml")

    urls = [loc.text for loc in soup.find_all("loc")]
    return [url for url in urls if url.startswith(base_url)]
```

## 7. Error Handling & Retry

**Decision**: Tenacity library for retries with exponential backoff

**Rationale**:
- Cleaner than manual retry loops
- Configurable retry strategies
- Works with both sync and async code

**Implementation**:
```python
from tenacity import retry, stop_after_attempt, wait_exponential

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=2, max=10)
)
def embed_with_retry(texts: list[str]) -> list[list[float]]:
    response = co.embed(texts=texts, model="embed-english-v3.0", input_type="search_document")
    return response.embeddings
```

## Summary of Technology Choices

| Component | Choice | Rationale |
|-----------|--------|-----------|
| Package Manager | UV | Fast, modern, single tool |
| Embeddings | Cohere embed-english-v3.0 | Free tier, quality, simple SDK |
| Vector DB | Qdrant Cloud | Free tier, managed, upsert support |
| HTTP Client | httpx | Modern, async-capable |
| HTML Parser | BeautifulSoup4 | Mature, Docusaurus compatible |
| Retry | tenacity | Clean retry patterns |

## Resolved NEEDS CLARIFICATION

All technical context items have been resolved through this research. No outstanding unknowns remain.
