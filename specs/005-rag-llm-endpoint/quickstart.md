# Quickstart: RAG LLM Endpoint

**Feature**: 005-rag-llm-endpoint
**Date**: 2025-12-17

## Prerequisites

- Python 3.10+
- Existing Qdrant collection with ingested textbook content (Spec 003)
- API keys configured in `.env`

## Setup

### 1. Install Dependencies

```bash
cd Backend
pip install fastapi uvicorn[standard] openai pydantic-settings
```

Or with uv:
```bash
cd Backend
uv pip install fastapi uvicorn[standard] openai pydantic-settings
```

### 2. Configure Environment

Add `OPENAI_API_KEY` to your existing `.env` file:

```bash
# Backend/.env
COHERE_API_KEY=your-cohere-key      # existing
QDRANT_URL=your-qdrant-url          # existing
QDRANT_API_KEY=your-qdrant-key      # existing
OPENAI_API_KEY=your-openai-key      # NEW
OPENAI_MODEL_NAME=gpt-3.5-turbo     # optional, default: gpt-3.5-turbo
```

### 3. Run the Server

```bash
cd Backend
uvicorn app:app --reload --port 8000
```

Server starts at: `http://localhost:8000`

## Usage

### Basic Query

```bash
curl -X POST http://localhost:8000/rag/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```

### Query with Custom Top-K

```bash
curl -X POST http://localhost:8000/rag/query \
  -H "Content-Type: application/json" \
  -d '{"question": "How do humanoid robots maintain balance?", "top_k": 3}'
```

### Using httpie

```bash
http POST localhost:8000/rag/query question="What is ROS 2?"
```

### Using Python

```python
import requests

response = requests.post(
    "http://localhost:8000/rag/query",
    json={"question": "What is ROS 2?", "top_k": 5}
)

data = response.json()
print(f"Answer: {data['answer']}")
print(f"Sources: {len(data['context_chunks'])} chunks")
```

## Expected Response

```json
{
  "answer": "ROS 2 (Robot Operating System 2) is an open-source robotics middleware that provides tools and libraries for building robot applications...",
  "context_chunks": [
    {
      "text": "ROS 2 is the next generation of the Robot Operating System...",
      "url": "https://ai-native-book.vercel.app/docs/module-1-ros2/chapter-1",
      "title": "Introduction to ROS 2",
      "chunk_index": 0,
      "score": 0.92
    }
  ],
  "metadata": {
    "used_top_k": 5,
    "model_name": "gpt-3.5-turbo"
  }
}
```

## API Documentation

Once the server is running, access:
- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`
- OpenAPI JSON: `http://localhost:8000/openapi.json`

## Troubleshooting

| Error | Cause | Fix |
|-------|-------|-----|
| `500: OPENAI_API_KEY not set` | Missing env var | Add to `.env` |
| `503: LLM service unavailable` | OpenAI API down | Check API status, retry |
| `503: Retrieval service unavailable` | Qdrant/Cohere issue | Check credentials, connection |
| `400: Question is required` | Empty question | Provide non-empty question |
| Empty context_chunks | No matching content | Try different question, verify ingestion |

## Next Steps

1. **Test with sample questions** from the textbook topics
2. **Verify answer quality** - answers should reference context
3. **Adjust top_k** based on answer quality needs
4. **Monitor latency** - typical: 3-8 seconds
