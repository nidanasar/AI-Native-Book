# API Contracts: RAG Web UI

## Frontend Configuration

### GET `/config.js`
**Purpose**: Configuration file for frontend application

**Response**:
```javascript
const CONFIG = {
  API_BASE_URL: string,  // Backend API base URL
  DEFAULT_TOP_K: number, // Default number of context chunks (default: 5)
  MAX_CHAT_HISTORY: number // Maximum chat history items (default: 20)
};
```

## UI State Management

### UI State Changes
The frontend maintains internal state for:
- Current question input
- Loading status
- Error messages
- Chat history
- Selected top_k value

## Backend API Integration

### POST `/rag/query` (External API)
**Purpose**: Query the RAG system for answers

**Request** (from existing backend):
```json
{
  "question": "string",
  "top_k": "number (optional, default: 5)"
}
```

**Response** (from existing backend):
```json
{
  "answer": "string",
  "context_chunks": [
    {
      "text": "string",
      "url": "string",
      "title": "string",
      "score": "number"
    }
  ],
  "metadata": {
    "used_top_k": "number",
    "model_name": "string"
  }
}
```

## Frontend UI Events

### User Interactions
- `SUBMIT_QUESTION`: User clicks submit button
- `UPDATE_QUESTION`: User types in question input
- `UPDATE_TOP_K`: User changes top_k selection
- `CLEAR_HISTORY`: User clears chat history

### UI State Updates
- `SET_LOADING`: Set loading state during API call
- `SET_RESPONSE`: Set response after successful API call
- `SET_ERROR`: Set error state after failed API call
- `ADD_TO_HISTORY`: Add new conversation to chat history