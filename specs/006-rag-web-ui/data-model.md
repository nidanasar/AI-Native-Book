# Data Model: RAG Web UI

## Entities

### QuestionInput
- **Fields**:
  - `text` (string, required): The user's question text
  - `top_k` (number, optional): Number of context chunks to retrieve (default: 5)
  - `timestamp` (Date, required): When the question was submitted

### RAGResponse
- **Fields**:
  - `answer` (string, required): The AI-generated answer
  - `context_chunks` (array of ContextChunk, required): Source chunks used for answer generation
  - `metadata` (object, optional): Additional response information including used_top_k and model_name

### ContextChunk
- **Fields**:
  - `text` (string, required): The text content of the chunk
  - `url` (string, required): The source URL of the chunk
  - `title` (string, required): The title of the source document
  - `score` (number, optional): Relevance score of the chunk

### ChatHistoryItem
- **Fields**:
  - `question` (QuestionInput, required): The original question
  - `response` (RAGResponse, required): The response to the question
  - `id` (string, required): Unique identifier for the history item

### ChatHistory
- **Fields**:
  - `items` (array of ChatHistoryItem, required): Chronological list of conversation items
  - `maxSize` (number, optional): Maximum number of items to store (default: 20)

### UIState
- **Fields**:
  - `isLoading` (boolean, required): Whether an API call is in progress
  - `error` (string, optional): Error message if API call failed
  - `currentQuestion` (string, optional): The current question being entered
  - `topKValue` (number, optional): Current top_k selection (default: 5)