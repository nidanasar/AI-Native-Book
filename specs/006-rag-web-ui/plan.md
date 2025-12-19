# Implementation Plan: RAG Web UI with Chat History

**Feature**: 006-rag-web-ui
**Created**: 2025-12-18
**Status**: Draft
**Input**: spec.md

## Technical Context

This feature implements a simple web UI for querying the RAG API with chat history functionality. The backend RAG API endpoints are already implemented (POST /rag/query) and accessible via HTTP. The UI will be built as a lightweight single-page application using HTML/CSS/JavaScript to maintain simplicity as per the feature requirements.

### Known Unknowns
- Backend API base URL configuration approach
- Specific UI framework choice (vanilla HTML/JS vs React)

## Constitution Check

This implementation aligns with project constitution:
- ✅ Minimal viable implementation approach
- ✅ Simple and lightweight solution
- ✅ Iterative development pattern
- ✅ Clean separation of concerns

## Gates

- **Technology Alignment**: ✅ Simple web stack (HTML/CSS/JS) aligns with lightweight requirement
- **Scope Compliance**: ✅ Focused on core UI functionality without over-engineering
- **Integration Feasibility**: ✅ API endpoints already exist and are accessible

---

## Phase 0: Research & Discovery

### Research Tasks
1. **Frontend Technology Choice**: Research whether to use vanilla HTML/CSS/JS or React for the UI
2. **API Integration Pattern**: Research best practices for calling FastAPI endpoints from frontend
3. **Chat History Storage**: Research session-based vs persistent storage approaches
4. **Configuration Management**: Research environment variable handling in frontend applications

### Expected Outcomes
- Decision on frontend technology stack
- Clear understanding of API integration approach
- Storage strategy for chat history
- Configuration management approach

## Phase 1: Design & Architecture

### 1. Tech choice and structure

Simple HTML/CSS/JavaScript approach for lightweight implementation:
- `frontend/index.html` – Main HTML structure with question input, submit button, response area, and chat history panel
- `frontend/style.css` – Basic styling for layout, responsive design, and visual presentation
- `frontend/script.js` – JavaScript logic for API calls, state management, and DOM manipulation
- `frontend/config.js` – Configuration constants including backend API URL

### 2. Core UI behavior

Main UI flow:
- Text input field for user to enter `question`
- Optional dropdown selector for `top_k` parameter (default: 5)
- Submit button triggers API call
- Loading spinner appears during request
- Response area displays: answer text and list of context chunks (URL + snippet + score)
- Previous conversations added to chat history panel
- Error messages displayed for API failures

UI elements:
- Question input: Text area with submit button
- Answer area: Plain text display with basic formatting
- Sources list: Each item shows URL, title, text snippet, and relevance score
- Chat history: Scrollable panel with conversation threads
- Error display: User-friendly error messages with retry option

### 3. API call details

Request format:
- POST to `/rag/query` endpoint
- JSON body: `{ "question": string, "top_k"?: number }`
- Headers: `Content-Type: application/json`

Response handling:
- `answer` field → Display in answer area
- `context_chunks` array → Render as list of sources with URL, text snippet, and score
- `metadata` → Display request information if needed

### 4. Config

Configuration approach:
- `frontend/config.js` file with constant: `API_BASE_URL`
- Default value: `http://localhost:8000` (for local development)
- Environment-specific configuration possible through build process
- Config file is separate from main logic for easy modification

### 5. Testing / manual check

Manual testing plan:
- Start backend FastAPI server
- Open `frontend/index.html` in web browser
- Enter 2-3 different questions about Physical AI/Humanoid Robotics
- Verify answers display correctly with source context
- Confirm chat history maintains previous conversations
- Test error handling by temporarily stopping backend
- Verify top_k parameter affects number of context chunks returned

## Phase 2: Implementation Plan

### Implementation Tasks
1. Create basic HTML structure with all required UI elements
2. Implement CSS styling for responsive layout
3. Create JavaScript API integration functions
4. Implement chat history state management
5. Add loading and error states
6. Test integration with backend API
7. Create documentation for running the UI

### Success Criteria
- UI successfully calls backend API and displays responses
- Chat history maintains conversation context
- Loading states provide user feedback during API calls
- Error handling displays user-friendly messages
- Responsive design works on desktop and mobile