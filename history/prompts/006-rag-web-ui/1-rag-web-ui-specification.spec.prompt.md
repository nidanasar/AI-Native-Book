---
id: 1
title: "RAG Web UI Specification"
stage: spec
date: 2025-12-18
model: Claude Sonnet 4.5
surface: agent
feature: rag-web-ui
branch: 006-rag-web-ui
user: Claude Code
command: /sp.specify
labels: ["spec", "web-ui", "rag", "chat-history"]
links:
  spec: "specs/006-rag-web-ui/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/006-rag-web-ui/spec.md"
  - "specs/006-rag-web-ui/checklists/requirements.md"
tests: []
---

# RAG Web UI Specification

## PROMPT_TEXT

```
RAG Spec-6: Simple web UI for querying the RAG API with chat history

Target context:
- Project: Physical AI & Humanoid Robotics Docusaurus book
- Spec 1: ingestion to Qdrant done
- Spec 2: retrieval from Qdrant done
- Spec 3: FastAPI RAG endpoint done (e.g. POST /rag/query)
- Spec 4: LLM endpoint done

Goal:
- Build a minimal web UI that:
  - Lets a user type a question
  - Sends it to the existing FastAPI /rag/query endpoint
  - Displays the model's answer
  - Shows the context chunks (sources) used for the answer
  - Maintains chat history for context

High-level behavior:
- Single-page style interface:
  - Input box for question
  - Optional top_k selector or keep default
  - Submit button
  - Chat history panel showing previous conversations
  - Area to display:
    - Answer text
    - List of source chunks with URL and short snippet
- The UI should call the backend via HTTP (browser → FastAPI RAG endpoint).

Constraints:
- Keep it simple and lightweight.
- You can choose a straightforward stack such as:
  - React (Vite/Next) or
  - a minimal HTML/JS page served by FastAPI (whichever fits the existing project better).
- No authentication, no streaming.
- Assume the backend base URL is configurable (e.g. env or config in the UI).

Non-goals:
- No complex design or styling (basic CSS is enough).
- No advanced state management, only what's needed for this page.

Deliverables:
- UI structure and components/pages.
- Logic to call the /rag/query endpoint and handle loading/error states.
- Display of answer + source chunks in a readable format.
- Chat history panel to maintain conversation context.
- Basic instructions for running the UI and pointing it at the backend.
```

## RESPONSE_TEXT

Created a comprehensive specification for the RAG Web UI with chat history feature. The spec includes:

- Context section explaining how this feature builds on previous RAG components
- User scenarios covering core functionality (asking questions, viewing answers, chat history)
- Functional requirements (FR-001 to FR-010) covering UI elements, API integration, and error handling
- Success criteria with measurable outcomes
- Assumptions about backend API availability and session-based chat history
- Out of scope items to maintain focus
- Deliverables including UI structure, API integration, and deployment instructions

The specification follows the template structure and maintains focus on user value without implementation details. All mandatory sections are completed, and a quality checklist was created and validated.

## OUTCOME

Specification created successfully for the RAG Web UI feature. The spec is complete, testable, and ready for planning phase. All quality validation criteria have been met.