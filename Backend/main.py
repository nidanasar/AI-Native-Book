"""
RAG Ingestion Pipeline for AI-Native Textbook

This script crawls the deployed Docusaurus textbook, extracts and chunks content,
generates embeddings via Cohere, and stores vectors in Qdrant Cloud.

Usage:
    uv run python main.py                    # Ingest all pages from sitemap
    uv run python main.py --url <url>        # Ingest single URL
    uv run python main.py --urls <file>      # Ingest URLs from file (one per line)
"""

import hashlib
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
    """Main entry point for the ingestion pipeline."""
    # Validate configuration
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

    # Parse command line arguments
    urls_to_process = []

    if len(sys.argv) > 1:
        if sys.argv[1] == "--url" and len(sys.argv) > 2:
            # Single URL mode
            urls_to_process = [sys.argv[2]]
        elif sys.argv[1] == "--urls" and len(sys.argv) > 2:
            # File mode - read URLs from file
            with open(sys.argv[2], "r") as f:
                urls_to_process = [line.strip() for line in f if line.strip()]
        else:
            print("Usage:")
            print("  python main.py                    # Ingest all pages from sitemap")
            print("  python main.py --url <url>        # Ingest single URL")
            print("  python main.py --urls <file>      # Ingest URLs from file")
            sys.exit(1)
    else:
        # Default: discover all URLs from sitemap
        urls_to_process = get_all_urls(BASE_URL)

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
