# Quickstart: RAG Ingestion Pipeline

**Feature**: 003-rag-ingestion-pipeline
**Date**: 2025-12-17

## Prerequisites

1. **Python 3.11+** installed
2. **UV** package manager installed:
   ```bash
   # Windows (PowerShell)
   powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"

   # macOS/Linux
   curl -LsSf https://astral.sh/uv/install.sh | sh
   ```

3. **Cohere Account** (free tier):
   - Sign up at https://dashboard.cohere.com/
   - Create API key in dashboard

4. **Qdrant Cloud Account** (free tier):
   - Sign up at https://cloud.qdrant.io/
   - Create a cluster (free tier)
   - Get cluster URL and API key

## Setup

### 1. Navigate to Backend folder

```bash
cd Backend
```

### 2. Initialize UV project

```bash
uv init
```

### 3. Add dependencies

```bash
uv add cohere qdrant-client httpx beautifulsoup4 python-dotenv tenacity lxml
```

### 4. Create environment file

Create `.env` file in the Backend folder:

```bash
# .env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=https://your-cluster-id.us-east4-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
BASE_URL=https://ai-native-book-1fj5bdpkr-nida-nasars-projects.vercel.app
```

### 5. Run the ingestion

```bash
uv run python main.py
```

## Expected Output

```
Starting RAG Ingestion Pipeline
================================
Base URL: https://ai-native-book-1fj5bdpkr-nida-nasars-projects.vercel.app

[1/2] Discovering URLs from sitemap...
Found 45 pages to process

[2/2] Processing pages...
  [1/45] /docs/intro - 5 chunks ✓
  [2/45] /docs/chapter-1 - 8 chunks ✓
  ...
  [45/45] /blog - 3 chunks ✓

================================
Ingestion Complete!
- Total pages: 45
- Successful: 43
- Skipped (no content): 2
- Failed: 0
- Total chunks: 287
- Duration: 4m 32s
```

## Verification

### Check Qdrant Collection

1. Go to Qdrant Cloud dashboard
2. Select your cluster
3. Navigate to Collections
4. Click `book_embeddings`
5. Verify point count matches chunk count

### Test Query (optional)

```python
# Quick test in Python REPL
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

load_dotenv()

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Check collection info
info = client.get_collection("book_embeddings")
print(f"Points count: {info.points_count}")
print(f"Vectors count: {info.vectors_count}")
```

## Troubleshooting

### "COHERE_API_KEY not found"
- Ensure `.env` file exists in Backend folder
- Check for typos in variable name
- Restart terminal after creating `.env`

### "Connection refused" to Qdrant
- Verify QDRANT_URL is correct (include https://)
- Check API key is valid
- Ensure cluster is running (not paused)

### "No URLs found in sitemap"
- Verify BASE_URL is accessible in browser
- Check if sitemap.xml exists at `{BASE_URL}/sitemap.xml`
- Docusaurus may need to be rebuilt if pages were added

### Rate limit errors
- Cohere free tier: 1000 calls/month
- Wait and retry, or upgrade to paid tier
- Check dashboard for quota usage

## Next Steps

After successful ingestion:

1. **Verify in Qdrant Dashboard** - Check collection has expected number of vectors
2. **Implement RAG Query** - Create retrieval endpoint (separate feature)
3. **Schedule Re-ingestion** - Run after textbook updates

## File Structure After Setup

```
Backend/
├── main.py              # Ingestion script
├── pyproject.toml       # UV project config
├── uv.lock              # Dependency lock file
├── .env                 # Local credentials (gitignored)
├── .env.example         # Template for credentials
└── .python-version      # Python version (optional)
```
