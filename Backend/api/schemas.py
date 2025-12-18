"""
Pydantic models for RAG API request/response validation.
"""

from pydantic import BaseModel, Field


class RagQueryRequest(BaseModel):
    """Request model for RAG query endpoint."""

    question: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="Natural language question about the textbook content"
    )
    top_k: int = Field(
        default=5,
        ge=1,
        le=50,
        description="Number of context chunks to retrieve"
    )


class ContextChunk(BaseModel):
    """Simplified view of a retrieved chunk for API response."""

    text: str = Field(..., description="Chunk text content")
    url: str = Field(..., description="Source URL of the original document")
    title: str = Field(..., description="Page title from source")
    chunk_index: int = Field(..., description="Index of chunk within source document")
    score: float = Field(..., description="Relevance score (0.0-1.0)")


class ResponseMetadata(BaseModel):
    """Metadata about the RAG response."""

    used_top_k: int = Field(..., description="Actual number of chunks retrieved")
    model_name: str = Field(..., description="LLM model used for answer generation")


class RagQueryResponse(BaseModel):
    """Response model for RAG query endpoint."""

    answer: str = Field(..., description="LLM-generated answer to the question")
    context_chunks: list[ContextChunk] = Field(
        ...,
        description="Retrieved chunks used as context for answer generation"
    )
    metadata: ResponseMetadata = Field(..., description="Response metadata")
