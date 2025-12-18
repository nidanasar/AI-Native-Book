"""
FastAPI application for RAG API.

Provides endpoints for querying the Physical AI & Humanoid Robotics textbook
using retrieval-augmented generation.
"""

import logging
from fastapi import FastAPI

from api.routes import router

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)

app = FastAPI(
    title="RAG API",
    description="RAG endpoint for Physical AI & Humanoid Robotics textbook Q&A",
    version="1.0.0"
)

# Include RAG router
app.include_router(router)


@app.get("/health")
def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}
