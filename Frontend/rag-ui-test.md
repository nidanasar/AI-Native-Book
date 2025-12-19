# Testing the RAG Web UI

## Prerequisites
- Backend API server running with `/rag/query` endpoint
- Frontend development server running

## Test Steps

1. **Start Backend Server**
   ```bash
   cd Backend
   uv run python app.py  # or however the backend is started
   ```

2. **Start Frontend Development Server**
   ```bash
   cd frontend
   yarn start  # or yarn start:rag to use specific API URL
   ```

3. **Access the RAG UI**
   - Navigate to `http://localhost:3000/rag` in your browser
   - Verify the page loads correctly with input fields

4. **Test Basic Functionality**
   - Enter a question in the input field (e.g., "What is ROS 2?")
   - Adjust top_k value if desired (default: 5)
   - Click "Submit Question" button
   - Verify loading state appears
   - Verify answer appears in the answer area
   - Verify context chunks appear with URL and text snippet

5. **Test Chat History**
   - Ask a second question
   - Verify both questions appear in the chat history panel
   - Verify previous answers remain accessible

6. **Test Error Handling**
   - Stop the backend server temporarily
   - Submit a question
   - Verify error message appears
   - Restart backend server
   - Submit a question again
   - Verify normal functionality resumes

7. **Test Configuration**
   - Verify top_k parameter affects the number of context chunks returned
   - Test different top_k values (1, 3, 5, 10)

8. **Test Responsive Design**
   - Resize browser window to mobile size
   - Verify UI remains functional and readable

## Expected Results
- Questions are successfully sent to the backend API
- Answers and context chunks are displayed properly
- Chat history maintains previous conversations
- Error states are handled gracefully
- UI is responsive and user-friendly