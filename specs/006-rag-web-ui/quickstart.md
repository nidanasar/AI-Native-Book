# Quickstart: RAG Web UI

## Prerequisites
- Backend API running (FastAPI server with `/rag/query` endpoint)
- Web browser (Chrome, Firefox, Safari, or Edge)

## Setup Instructions

### 1. Start Backend Server
```bash
cd Backend
uv run python app.py  # or however the backend is started
```
Ensure the `/rag/query` endpoint is accessible at the configured URL.

### 2. Serve Frontend
The frontend can be served in multiple ways:

**Option A: Local File**
- Open `frontend/index.html` directly in your web browser

**Option B: Simple HTTP Server**
```bash
cd frontend
python -m http.server 3000  # Python 3
# Then visit http://localhost:3000
```

**Option C: Using Node.js**
```bash
npm install -g http-server
cd frontend
http-server
```

### 3. Configure Backend URL
Edit `frontend/config.js` to match your backend URL:
```javascript
const CONFIG = {
  API_BASE_URL: "http://localhost:8000",  // Update this to your backend URL
  DEFAULT_TOP_K: 5,
  MAX_CHAT_HISTORY: 20
};
```

## Usage
1. Open the UI in your web browser
2. Type a question in the input field
3. Optionally adjust the top_k parameter
4. Click "Submit" to get an answer
5. View the answer and source context chunks
6. Continue the conversation with follow-up questions

## Troubleshooting
- If you get CORS errors, ensure your backend allows requests from the frontend origin
- If API calls fail, verify the backend server is running and URL is correct in config
- Check browser console for JavaScript errors