"""
RAG Pipeline for AI-Native Textbook

This script provides both ingestion and retrieval capabilities for the textbook RAG system:
- Crawls the deployed Docusaurus textbook, extracts and chunks content
- Generates embeddings via Cohere and stores vectors in Qdrant Cloud
- Queries stored vectors to find relevant content for user questions

Ingestion Usage:
    uv run python main.py                              # Ingest all pages from sitemap
    uv run python main.py --url <url>                  # Ingest single URL
    uv run python main.py --urls <file>                # Ingest URLs from file

Retrieval Usage:
    uv run python main.py --query "What is ROS 2?"     # Query with default top_k=5
    uv run python main.py --query "..." --top-k 10     # Query with custom result count
    uv run python main.py --query "..." --url-filter "module-1"  # Filter by URL
    uv run python main.py --interactive                # Interactive query mode
"""

import hashlib
import json
import logging
import os
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional
from urllib.parse import urljoin, urlparse

import cohere
import httpx
from bs4 import BeautifulSoup
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type

# Load environment variables
load_dotenv()

# Configuration
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
BASE_URL = os.getenv("TARGET_URL", "https://ai-native-book-1fj5bdpkr-nida-nasars-projects.vercel.app")

# Constants
COLLECTION_NAME = "book_embeddings"
EMBEDDING_MODEL = "embed-english-v3.0"
EMBEDDING_DIMENSION = 1024
CHUNK_SIZE = 800  # Target tokens per chunk
CHUNK_OVERLAP = 100  # Overlap tokens between chunks

# Retrieval Constants
DEFAULT_TOP_K = 5
MAX_TOP_K = 50
MAX_QUERY_LENGTH = 1000

# Logging setup
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger(__name__)


# -----------------------------------------------------------------------------
# Data Models
# -----------------------------------------------------------------------------

@dataclass
class Document:
    """Represents a single web page being ingested."""
    url: str
    title: str
    content: str
    fetched_at: datetime


@dataclass
class Chunk:
    """A semantically meaningful piece of a document."""
    id: int
    url: str
    chunk_index: int
    text: str
    embedding: list[float]
    title: str
    ingested_at: datetime


@dataclass
class IngestionResult:
    """Tracks the outcome of processing a single URL."""
    url: str
    status: str  # "success" | "failed" | "skipped"
    chunk_count: int
    error: Optional[str]
    duration_ms: int


@dataclass
class RetrievedChunk:
    """A chunk returned from vector search."""
    text: str
    url: str
    title: str
    chunk_index: int
    score: float


# -----------------------------------------------------------------------------
# Utility Functions
# -----------------------------------------------------------------------------

def generate_chunk_id(url: str, chunk_index: int) -> int:
    """Generate deterministic ID for Qdrant point.

    Uses SHA256 hash of url+chunk_index, taking first 16 hex chars.
    This ensures same input always produces same ID for upsert behavior.
    """
    key = f"{url}_{chunk_index}"
    hash_hex = hashlib.sha256(key.encode()).hexdigest()[:16]
    return int(hash_hex, 16)


def validate_url(url: str, base_domain: str) -> bool:
    """Validate URL belongs to the configured base domain."""
    try:
        parsed = urlparse(url)
        base_parsed = urlparse(base_domain)
        return parsed.netloc == base_parsed.netloc
    except Exception:
        return False


def estimate_tokens(text: str) -> int:
    """Rough token estimation (words * 1.3)."""
    return int(len(text.split()) * 1.3)


# -----------------------------------------------------------------------------
# URL Discovery
# -----------------------------------------------------------------------------

def get_all_urls(base_url: str) -> list[str]:
    """Discover all page URLs from sitemap.xml.

    Args:
        base_url: The base URL of the Docusaurus site

    Returns:
        List of page URLs to ingest
    """
    sitemap_url = urljoin(base_url, "/sitemap.xml")
    logger.info(f"Fetching sitemap from {sitemap_url}")

    try:
        with httpx.Client(timeout=30.0) as client:
            response = client.get(sitemap_url)
            response.raise_for_status()
    except httpx.HTTPError as e:
        logger.error(f"Failed to fetch sitemap: {e}")
        return []

    soup = BeautifulSoup(response.text, "html.parser")
    urls = []

    for loc in soup.find_all("loc"):
        sitemap_url_text = loc.text.strip()
        # Extract path from sitemap URL and rebuild with base_url
        parsed = urlparse(sitemap_url_text)
        path = parsed.path

        # Filter to only include docs pages
        if "/docs/" in path or path.endswith("/docs"):
            # Rebuild URL with the configured base_url
            rebuilt_url = urljoin(base_url, path)
            urls.append(rebuilt_url)

    logger.info(f"Discovered {len(urls)} URLs from sitemap")
    return urls


# -----------------------------------------------------------------------------
# Content Extraction
# -----------------------------------------------------------------------------

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=2, max=10),
    retry=retry_if_exception_type((httpx.HTTPError, httpx.TimeoutException))
)
def fetch_url(url: str) -> str:
    """Fetch URL content with retry logic."""
    with httpx.Client(timeout=30.0, follow_redirects=True) as client:
        response = client.get(url)
        response.raise_for_status()
        return response.text


def extract_text_from_url(url: str) -> Document:
    """Extract text content from a Docusaurus page.

    Args:
        url: The URL to fetch and parse

    Returns:
        Document with extracted title and content
    """
    logger.info(f"Extracting content from {url}")

    html = fetch_url(url)
    soup = BeautifulSoup(html, "html.parser")

    # Extract title
    title = ""
    title_tag = soup.find("title")
    if title_tag:
        title = title_tag.text.strip()
        # Remove site suffix (e.g., " | My Site")
        if " | " in title:
            title = title.split(" | ")[0]

    # Extract main content from Docusaurus article
    content = ""

    # Try to find the main article content
    article = soup.find("article")
    if article:
        # Remove navigation, table of contents, etc.
        for selector in ["nav", ".table-of-contents", ".pagination-nav", "footer", "header"]:
            for element in article.select(selector):
                element.decompose()

        # Get text content
        content = article.get_text(separator="\n", strip=True)
    else:
        # Fallback to main content area
        main = soup.find("main")
        if main:
            content = main.get_text(separator="\n", strip=True)

    # Clean up content
    lines = [line.strip() for line in content.split("\n") if line.strip()]
    content = "\n".join(lines)

    return Document(
        url=url,
        title=title or url.split("/")[-1],  # Fallback to URL slug
        content=content,
        fetched_at=datetime.now(timezone.utc)
    )


# -----------------------------------------------------------------------------
# Text Chunking
# -----------------------------------------------------------------------------

def chunk_text(text: str, chunk_size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP) -> list[str]:
    """Split text into overlapping chunks based on token estimate.

    Args:
        text: The text to chunk
        chunk_size: Target tokens per chunk
        overlap: Overlap tokens between chunks

    Returns:
        List of text chunks
    """
    if not text.strip():
        return []

    words = text.split()
    chunks = []

    # Approximate words per chunk (tokens / 1.3)
    words_per_chunk = int(chunk_size / 1.3)
    overlap_words = int(overlap / 1.3)

    start = 0
    while start < len(words):
        end = start + words_per_chunk
        chunk_words = words[start:end]
        chunk = " ".join(chunk_words)

        if chunk.strip():
            chunks.append(chunk)

        # Move start forward, accounting for overlap
        start = end - overlap_words

        # Prevent infinite loop
        if start <= 0 and end >= len(words):
            break
        if end >= len(words):
            break

    logger.debug(f"Split text into {len(chunks)} chunks")
    return chunks


# -----------------------------------------------------------------------------
# Embedding Generation
# -----------------------------------------------------------------------------

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=2, min=4, max=30),
    retry=retry_if_exception_type(Exception)
)
def generate_embeddings(chunks: list[str], cohere_client: cohere.Client) -> list[list[float]]:
    """Generate embeddings for text chunks using Cohere.

    Args:
        chunks: List of text chunks to embed
        cohere_client: Initialized Cohere client

    Returns:
        List of embedding vectors (1024 dimensions each)
    """
    if not chunks:
        return []

    logger.info(f"Generating embeddings for {len(chunks)} chunks")

    response = cohere_client.embed(
        texts=chunks,
        model=EMBEDDING_MODEL,
        input_type="search_document"
    )

    return response.embeddings


# -----------------------------------------------------------------------------
# Qdrant Storage
# -----------------------------------------------------------------------------

def ensure_collection_exists(qdrant_client: QdrantClient) -> None:
    """Create the Qdrant collection if it doesn't exist."""
    collections = qdrant_client.get_collections().collections
    collection_names = [c.name for c in collections]

    if COLLECTION_NAME not in collection_names:
        logger.info(f"Creating collection '{COLLECTION_NAME}'")
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(
                size=EMBEDDING_DIMENSION,
                distance=models.Distance.COSINE
            )
        )

        # Create payload index for URL filtering
        qdrant_client.create_payload_index(
            collection_name=COLLECTION_NAME,
            field_name="url",
            field_schema=models.PayloadSchemaType.KEYWORD
        )
    else:
        logger.info(f"Collection '{COLLECTION_NAME}' already exists")


def delete_existing_chunks(qdrant_client: QdrantClient, url: str) -> None:
    """Delete all existing chunks for a URL before re-ingestion."""
    qdrant_client.delete(
        collection_name=COLLECTION_NAME,
        points_selector=models.FilterSelector(
            filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="url",
                        match=models.MatchValue(value=url)
                    )
                ]
            )
        )
    )


def save_chunks_to_qdrant(
    chunks: list[str],
    embeddings: list[list[float]],
    document: Document,
    qdrant_client: QdrantClient
) -> int:
    """Store chunks with embeddings in Qdrant.

    Args:
        chunks: Text chunks
        embeddings: Corresponding embedding vectors
        document: Source document metadata
        qdrant_client: Initialized Qdrant client

    Returns:
        Number of chunks stored
    """
    if not chunks or not embeddings:
        return 0

    # Delete existing chunks for this URL (upsert behavior)
    delete_existing_chunks(qdrant_client, document.url)

    # Prepare points for upload
    points = []
    ingested_at = datetime.now(timezone.utc).isoformat()

    for i, (chunk_text, embedding) in enumerate(zip(chunks, embeddings)):
        point_id = generate_chunk_id(document.url, i)

        points.append(models.PointStruct(
            id=point_id,
            vector=embedding,
            payload={
                "url": document.url,
                "title": document.title,
                "chunk_index": i,
                "text": chunk_text,
                "ingested_at": ingested_at
            }
        ))

    # Upsert points to Qdrant
    qdrant_client.upsert(
        collection_name=COLLECTION_NAME,
        points=points
    )

    logger.info(f"Stored {len(points)} chunks for {document.url}")
    return len(points)


# -----------------------------------------------------------------------------
# Retrieval Functions
# -----------------------------------------------------------------------------

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=2, min=4, max=30),
    retry=retry_if_exception_type(Exception)
)
def generate_query_embedding(query: str, cohere_client: cohere.Client) -> list[float]:
    """Generate embedding for a search query.

    Args:
        query: Natural language query string
        cohere_client: Initialized Cohere client

    Returns:
        1024-dimension embedding vector

    Note: Uses input_type="search_query" (vs "search_document" for ingestion)
    """
    # Truncate query if too long
    if len(query) > MAX_QUERY_LENGTH:
        query = query[:MAX_QUERY_LENGTH]
        logger.warning(f"Query truncated to {MAX_QUERY_LENGTH} characters")

    response = cohere_client.embed(
        texts=[query],
        model=EMBEDDING_MODEL,
        input_type="search_query"
    )

    return response.embeddings[0]


def search_qdrant(
    query_vector: list[float],
    qdrant_client: QdrantClient,
    top_k: int = DEFAULT_TOP_K,
    url_filter: Optional[str] = None
) -> list[models.ScoredPoint]:
    """Search Qdrant for similar vectors.

    Args:
        query_vector: 1024-dim embedding from generate_query_embedding
        qdrant_client: Initialized Qdrant client
        top_k: Number of results to return (1-50)
        url_filter: Optional URL prefix to filter results

    Returns:
        List of ScoredPoint with payload and score
    """
    try:
        # Build filter if URL prefix provided
        query_filter = None
        if url_filter:
            query_filter = models.Filter(
                must=[
                    models.FieldCondition(
                        key="url",
                        match=models.MatchText(text=url_filter)
                    )
                ]
            )

        results = qdrant_client.query_points(
            collection_name=COLLECTION_NAME,
            query=query_vector,
            limit=top_k,
            query_filter=query_filter
        ).points

        return results

    except Exception as e:
        logger.error(f"Qdrant search failed: {e}")
        return []


def retrieve_chunks(
    query: str,
    cohere_client: cohere.Client,
    qdrant_client: QdrantClient,
    top_k: int = DEFAULT_TOP_K,
    url_filter: Optional[str] = None
) -> list[RetrievedChunk]:
    """Main retrieval function: query → relevant chunks.

    Args:
        query: Natural language query
        cohere_client: Initialized Cohere client
        qdrant_client: Initialized Qdrant client
        top_k: Number of results (default 5)
        url_filter: Optional URL prefix filter

    Returns:
        List of RetrievedChunk sorted by relevance (highest first)
    """
    # Validate query
    if not query or not query.strip():
        return []

    # Clamp top_k to valid range
    top_k = max(1, min(top_k, MAX_TOP_K))

    # Generate query embedding
    query_vector = generate_query_embedding(query.strip(), cohere_client)

    # Search Qdrant
    results = search_qdrant(query_vector, qdrant_client, top_k, url_filter)

    # Convert to RetrievedChunk objects
    chunks = []
    for point in results:
        chunk = RetrievedChunk(
            text=point.payload.get("text", ""),
            url=point.payload.get("url", ""),
            title=point.payload.get("title", ""),
            chunk_index=point.payload.get("chunk_index", 0),
            score=point.score
        )
        chunks.append(chunk)

    return chunks


def format_results_json(query: str, results: list[RetrievedChunk], duration_ms: int) -> str:
    """Format retrieval results as clean JSON.

    Args:
        query: The original query string
        results: List of RetrievedChunk objects
        duration_ms: Query duration in milliseconds

    Returns:
        JSON string with query results
    """
    output = {
        "query": query,
        "count": len(results),
        "duration_ms": duration_ms,
        "results": [
            {
                "rank": i + 1,
                "score": round(chunk.score, 4),
                "url": chunk.url,
                "title": chunk.title,
                "chunk_index": chunk.chunk_index,
                "text": chunk.text[:500] + "..." if len(chunk.text) > 500 else chunk.text
            }
            for i, chunk in enumerate(results)
        ]
    }

    return json.dumps(output, indent=2, ensure_ascii=False)


def print_results_formatted(results: list[RetrievedChunk], duration_ms: int) -> None:
    """Print retrieval results in human-readable format for interactive mode.

    Args:
        results: List of RetrievedChunk objects
        duration_ms: Query duration in milliseconds
    """
    if not results:
        print("\nNo results found.")
        return

    print(f"\nResults (top {len(results)}, {duration_ms/1000:.1f}s):\n")

    for i, chunk in enumerate(results, 1):
        # Extract short path from URL for display
        url_path = chunk.url.split("/docs/")[-1] if "/docs/" in chunk.url else chunk.url
        score_display = f"{chunk.score:.3f}"

        print(f"[{i}] Score: {score_display} | {url_path}")
        # Show first 200 chars of text
        text_preview = chunk.text[:200].replace("\n", " ")
        if len(chunk.text) > 200:
            text_preview += "..."
        print(f'    "{text_preview}"')
        print()


def run_query(
    query: str,
    top_k: int = DEFAULT_TOP_K,
    url_filter: Optional[str] = None,
    output_json: bool = True
) -> None:
    """Run a single query and print results.

    Args:
        query: Natural language query
        top_k: Number of results to return
        url_filter: Optional URL prefix filter
        output_json: If True, output JSON; else human-readable format
    """
    # Validate configuration
    if not COHERE_API_KEY or not QDRANT_URL or not QDRANT_API_KEY:
        error = {"error": "Missing API configuration. Check COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY."}
        print(json.dumps(error, indent=2))
        return

    # Validate query
    if not query or not query.strip():
        error = {"error": "Query is required"}
        print(json.dumps(error, indent=2))
        return

    # Initialize clients
    cohere_client = cohere.Client(COHERE_API_KEY)
    qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

    # Execute query with timing
    start_time = time.time()
    try:
        results = retrieve_chunks(query, cohere_client, qdrant_client, top_k, url_filter)
        duration_ms = int((time.time() - start_time) * 1000)

        if output_json:
            # Use sys.stdout with UTF-8 encoding for proper character handling
            json_output = format_results_json(query, results, duration_ms)
            sys.stdout.buffer.write(json_output.encode('utf-8'))
            sys.stdout.buffer.write(b'\n')
        else:
            print_results_formatted(results, duration_ms)

    except Exception as e:
        error = {"error": f"Query failed: {str(e)}"}
        print(json.dumps(error, indent=2))


def run_interactive_mode() -> None:
    """Run interactive REPL mode for testing queries."""
    # Validate configuration
    if not COHERE_API_KEY or not QDRANT_URL or not QDRANT_API_KEY:
        print("Error: Missing API configuration. Check environment variables.")
        return

    # Initialize clients
    cohere_client = cohere.Client(COHERE_API_KEY)
    qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

    # Get collection info
    try:
        collection_info = qdrant_client.get_collection(COLLECTION_NAME)
        chunk_count = collection_info.points_count
    except Exception:
        chunk_count = "unknown"

    print("\n" + "=" * 60)
    print("RAG Retrieval CLI (type 'exit' to quit, 'help' for commands)")
    print(f"Collection: {COLLECTION_NAME} ({chunk_count} chunks)")
    print("=" * 60 + "\n")

    top_k = DEFAULT_TOP_K

    while True:
        try:
            user_input = input("> ").strip()

            if not user_input:
                continue

            if user_input.lower() == "exit":
                print("Goodbye!")
                break

            if user_input.lower() == "help":
                print("\nCommands:")
                print("  <query>      - Search for relevant chunks")
                print("  top-k <n>    - Set number of results (1-50)")
                print("  exit         - Quit interactive mode")
                print()
                continue

            if user_input.lower().startswith("top-k "):
                try:
                    new_k = int(user_input.split()[1])
                    top_k = max(1, min(new_k, MAX_TOP_K))
                    print(f"top_k set to {top_k}")
                except (ValueError, IndexError):
                    print("Usage: top-k <number>")
                continue

            # Execute query
            start_time = time.time()
            results = retrieve_chunks(user_input, cohere_client, qdrant_client, top_k)
            duration_ms = int((time.time() - start_time) * 1000)

            print_results_formatted(results, duration_ms)

        except KeyboardInterrupt:
            print("\nGoodbye!")
            break
        except EOFError:
            print("\nGoodbye!")
            break


# -----------------------------------------------------------------------------
# Main Pipeline
# -----------------------------------------------------------------------------

def ingest_url(
    url: str,
    cohere_client: cohere.Client,
    qdrant_client: QdrantClient
) -> IngestionResult:
    """Ingest a single URL into the vector database.

    Args:
        url: The URL to ingest
        cohere_client: Initialized Cohere client
        qdrant_client: Initialized Qdrant client

    Returns:
        IngestionResult with status and metrics
    """
    start_time = time.time()

    try:
        # Validate URL
        if not validate_url(url, BASE_URL):
            return IngestionResult(
                url=url,
                status="failed",
                chunk_count=0,
                error=f"URL not in allowed domain: {BASE_URL}",
                duration_ms=int((time.time() - start_time) * 1000)
            )

        # Extract content
        document = extract_text_from_url(url)

        # Check for empty content
        if not document.content.strip():
            logger.warning(f"No content found for {url}")
            return IngestionResult(
                url=url,
                status="skipped",
                chunk_count=0,
                error="No textual content found",
                duration_ms=int((time.time() - start_time) * 1000)
            )

        # Chunk content
        chunks = chunk_text(document.content)

        if not chunks:
            return IngestionResult(
                url=url,
                status="skipped",
                chunk_count=0,
                error="Content too short to chunk",
                duration_ms=int((time.time() - start_time) * 1000)
            )

        # Generate embeddings
        embeddings = generate_embeddings(chunks, cohere_client)

        # Store in Qdrant
        chunk_count = save_chunks_to_qdrant(chunks, embeddings, document, qdrant_client)

        return IngestionResult(
            url=url,
            status="success",
            chunk_count=chunk_count,
            error=None,
            duration_ms=int((time.time() - start_time) * 1000)
        )

    except Exception as e:
        logger.error(f"Failed to ingest {url}: {e}")
        return IngestionResult(
            url=url,
            status="failed",
            chunk_count=0,
            error=str(e),
            duration_ms=int((time.time() - start_time) * 1000)
        )


def ingest_batch(
    urls: list[str],
    cohere_client: cohere.Client,
    qdrant_client: QdrantClient
) -> list[IngestionResult]:
    """Ingest multiple URLs with progress reporting.

    Args:
        urls: List of URLs to ingest
        cohere_client: Initialized Cohere client
        qdrant_client: Initialized Qdrant client

    Returns:
        List of IngestionResult for each URL
    """
    results = []
    total = len(urls)

    for i, url in enumerate(urls, 1):
        logger.info(f"Processing [{i}/{total}]: {url}")
        result = ingest_url(url, cohere_client, qdrant_client)
        results.append(result)

        # Brief delay to respect rate limits
        if i < total:
            time.sleep(0.5)

    return results


def print_summary(results: list[IngestionResult]) -> None:
    """Print summary of ingestion results."""
    success = [r for r in results if r.status == "success"]
    failed = [r for r in results if r.status == "failed"]
    skipped = [r for r in results if r.status == "skipped"]

    total_chunks = sum(r.chunk_count for r in success)
    total_time = sum(r.duration_ms for r in results) / 1000

    print("\n" + "=" * 60)
    print("INGESTION SUMMARY")
    print("=" * 60)
    print(f"Total URLs processed: {len(results)}")
    print(f"  - Success: {len(success)}")
    print(f"  - Failed:  {len(failed)}")
    print(f"  - Skipped: {len(skipped)}")
    print(f"Total chunks created: {total_chunks}")
    print(f"Total time: {total_time:.1f} seconds")

    if failed:
        print("\nFailed URLs:")
        for r in failed:
            print(f"  - {r.url}: {r.error}")

    if skipped:
        print("\nSkipped URLs:")
        for r in skipped:
            print(f"  - {r.url}: {r.error}")

    print("=" * 60)


# -----------------------------------------------------------------------------
# Entry Point
# -----------------------------------------------------------------------------

def main():
    """Main entry point for the RAG pipeline (ingestion and retrieval)."""
    # Parse command line arguments first to determine mode
    args = sys.argv[1:]

    # Check for retrieval modes first
    if "--query" in args:
        # Query mode
        try:
            query_idx = args.index("--query")
            query = args[query_idx + 1] if query_idx + 1 < len(args) else ""
        except (ValueError, IndexError):
            print('{"error": "Usage: python main.py --query \\"your query\\" [--top-k N] [--url-filter prefix]"}')
            sys.exit(1)

        # Parse optional top-k
        top_k = DEFAULT_TOP_K
        if "--top-k" in args:
            try:
                top_k_idx = args.index("--top-k")
                top_k = int(args[top_k_idx + 1])
            except (ValueError, IndexError):
                pass

        # Parse optional url-filter
        url_filter = None
        if "--url-filter" in args:
            try:
                filter_idx = args.index("--url-filter")
                url_filter = args[filter_idx + 1]
            except (ValueError, IndexError):
                pass

        run_query(query, top_k, url_filter)
        return

    if "--interactive" in args:
        # Interactive mode
        run_interactive_mode()
        return

    # Ingestion mode - validate configuration
    if not COHERE_API_KEY:
        logger.error("COHERE_API_KEY not set in environment")
        sys.exit(1)
    if not QDRANT_URL:
        logger.error("QDRANT_URL not set in environment")
        sys.exit(1)
    if not QDRANT_API_KEY:
        logger.error("QDRANT_API_KEY not set in environment")
        sys.exit(1)

    logger.info("Initializing RAG Ingestion Pipeline")
    logger.info(f"Base URL: {BASE_URL}")
    logger.info(f"Collection: {COLLECTION_NAME}")

    # Initialize clients
    cohere_client = cohere.Client(COHERE_API_KEY)
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY
    )

    # Ensure collection exists
    ensure_collection_exists(qdrant_client)

    # Parse ingestion arguments
    urls_to_process = []

    if "--url" in args:
        try:
            url_idx = args.index("--url")
            urls_to_process = [args[url_idx + 1]]
        except (ValueError, IndexError):
            print("Usage: python main.py --url <url>")
            sys.exit(1)
    elif "--urls" in args:
        try:
            urls_idx = args.index("--urls")
            with open(args[urls_idx + 1], "r") as f:
                urls_to_process = [line.strip() for line in f if line.strip()]
        except (ValueError, IndexError, FileNotFoundError) as e:
            print(f"Error: {e}")
            sys.exit(1)
    elif len(args) == 0:
        # Default: discover all URLs from sitemap
        urls_to_process = get_all_urls(BASE_URL)
    else:
        print("Usage:")
        print("  python main.py                              # Ingest all pages from sitemap")
        print("  python main.py --url <url>                  # Ingest single URL")
        print("  python main.py --urls <file>                # Ingest URLs from file")
        print("")
        print("Retrieval:")
        print('  python main.py --query "your question"      # Query with default top_k=5')
        print('  python main.py --query "..." --top-k 10     # Query with custom top_k')
        print('  python main.py --query "..." --url-filter "module-1"  # Filter by URL')
        print("  python main.py --interactive                # Interactive query mode")
        sys.exit(1)

    if not urls_to_process:
        logger.warning("No URLs to process")
        sys.exit(0)

    logger.info(f"Starting ingestion of {len(urls_to_process)} URLs")

    # Run ingestion
    results = ingest_batch(urls_to_process, cohere_client, qdrant_client)

    # Print summary
    print_summary(results)

    # Exit with error code if any failures
    failed_count = sum(1 for r in results if r.status == "failed")
    sys.exit(1 if failed_count > 0 else 0)


if __name__ == "__main__":
    main()
