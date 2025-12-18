"""
RAG service for question answering.

Orchestrates: retrieve → format context → call LLM → return response.
Uses Gemini (free tier) via OpenAI SDK compatibility.
"""

import logging
from openai import OpenAI

from config import settings
from main import retrieve_chunks, RetrievedChunk
from api.schemas import RagQueryResponse, ContextChunk, ResponseMetadata

# Initialize Cohere and Qdrant clients (reuse from main.py pattern)
import cohere
from qdrant_client import QdrantClient

logger = logging.getLogger(__name__)

# -----------------------------------------------------------------------------
# Prompt Template
# -----------------------------------------------------------------------------

SYSTEM_PROMPT = """You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
Answer the user's question based ONLY on the provided context from the textbook.
If the context doesn't contain relevant information to answer the question, say so clearly.
Be concise but thorough. Cite specific information from the context when possible."""

CONTEXT_TEMPLATE = """Context from the textbook:

{context}

---

Question: {question}

Answer:"""


# -----------------------------------------------------------------------------
# Helper Functions
# -----------------------------------------------------------------------------

def format_context(chunks: list[RetrievedChunk]) -> str:
    """Format retrieved chunks as context string for LLM prompt.

    Args:
        chunks: List of RetrievedChunk from retrieval

    Returns:
        Formatted context string with source attribution
    """
    if not chunks:
        return "No relevant context found in the textbook."

    formatted_parts = []
    for i, chunk in enumerate(chunks, 1):
        source_info = f"[Source {i}: {chunk.title}]"
        formatted_parts.append(f"{source_info}\n{chunk.text}")

    return "\n\n".join(formatted_parts)


def call_llm(prompt: str, system_prompt: str = SYSTEM_PROMPT) -> str:
    """Call Gemini LLM via OpenAI SDK compatibility layer.

    Args:
        prompt: User prompt with context and question
        system_prompt: System instructions for the LLM

    Returns:
        Generated answer string
    """
    logger.info(f"Calling LLM: {settings.llm_model_name}")

    # Initialize OpenAI client pointing to Gemini endpoint
    client = OpenAI(
        api_key=settings.gemini_api_key,
        base_url=settings.llm_base_url
    )

    response = client.chat.completions.create(
        model=settings.llm_model_name,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": prompt}
        ],
        temperature=0.7,
        max_tokens=1024
    )

    answer = response.choices[0].message.content
    logger.info("LLM response received")

    return answer


# -----------------------------------------------------------------------------
# Main RAG Function
# -----------------------------------------------------------------------------

def answer_question(question: str, top_k: int = 5) -> RagQueryResponse:
    """Main RAG function: question → retrieve → LLM → answer.

    Args:
        question: Natural language question from user
        top_k: Number of context chunks to retrieve (default: 5)

    Returns:
        RagQueryResponse with answer, context chunks, and metadata
    """
    logger.info(f"Processing question: {question[:100]}...")

    # Validate question
    question = question.strip()
    if not question:
        raise ValueError("Question cannot be empty")

    # Initialize clients
    cohere_client = cohere.Client(settings.cohere_api_key)
    qdrant_client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key
    )

    # Step 1: Retrieve relevant chunks
    logger.info(f"Retrieving top {top_k} chunks")
    retrieved = retrieve_chunks(
        query=question,
        cohere_client=cohere_client,
        qdrant_client=qdrant_client,
        top_k=top_k
    )
    logger.info(f"Retrieved {len(retrieved)} chunks")

    # Step 2: Format context
    context_str = format_context(retrieved)

    # Step 3: Build prompt
    prompt = CONTEXT_TEMPLATE.format(context=context_str, question=question)

    # Step 4: Call LLM
    answer = call_llm(prompt)

    # Step 5: Build response
    context_chunks = [
        ContextChunk(
            text=chunk.text,
            url=chunk.url,
            title=chunk.title,
            chunk_index=chunk.chunk_index,
            score=chunk.score
        )
        for chunk in retrieved
    ]

    metadata = ResponseMetadata(
        used_top_k=len(retrieved),
        model_name=settings.llm_model_name
    )

    return RagQueryResponse(
        answer=answer,
        context_chunks=context_chunks,
        metadata=metadata
    )
