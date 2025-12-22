"""
FastAPI application for RAG API.

Provides endpoints for querying the Physical AI & Humanoid Robotics textbook
using retrieval-augmented generation.
"""

import logging
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
import os

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

# Add CORS middleware to allow requests from the frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include RAG router
app.include_router(router)

# Serve static files (frontend build) if they exist
frontend_build_path = os.path.join(os.path.dirname(__file__), "..", "Frontend", "build")
if os.path.exists(frontend_build_path):
    app.mount("/", StaticFiles(directory=frontend_build_path, html=True), name="frontend")


@app.get("/")
def root():
    """Root endpoint with API info."""
    return {
        "message": "RAG API for Physical AI & Humanoid Robotics Textbook",
        "docs": "/docs",
        "health": "/health",
        "endpoints": {
            "query": "POST /rag/query"
        }
    }


@app.get("/health")
def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}
