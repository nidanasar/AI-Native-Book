"""
API routes for RAG endpoints.
"""

import logging
from fastapi import APIRouter, HTTPException

from api.schemas import RagQueryRequest, RagQueryResponse
from rag.service import answer_question

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/rag", tags=["RAG"])


@router.post("/query", response_model=RagQueryResponse)
def query_rag(request: RagQueryRequest) -> RagQueryResponse:
    """
    RAG query endpoint.

    Submit a natural language question and receive an AI-generated answer
    grounded in the textbook content, along with source context chunks.

    - **question**: Natural language question (1-2000 chars)
    - **top_k**: Number of context chunks to retrieve (1-50, default: 5)

    Returns answer, context_chunks, and metadata.
    """
    logger.info(f"Received query: {request.question[:100]}...")

    # Validate question is not just whitespace
    if not request.question.strip():
        raise HTTPException(
            status_code=400,
            detail="Question is required and must not be empty or whitespace"
        )

    try:
        response = answer_question(
            question=request.question,
            top_k=request.top_k
        )
        return response

    except ValueError as e:
        # Validation errors (empty question, etc.)
        logger.warning(f"Validation error: {e}")
        raise HTTPException(status_code=400, detail=str(e))

    except Exception as e:
        # LLM or retrieval service failures
        logger.error(f"Service error: {e}")
        raise HTTPException(
            status_code=503,
            detail="Service temporarily unavailable. Please try again later."
        )
