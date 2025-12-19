# Research: RAG Web UI Implementation

## Decision: Frontend Technology Choice
**Rationale**: For a lightweight, simple implementation as required by the spec, vanilla HTML/CSS/JavaScript is chosen over React. This reduces complexity, build steps, and dependencies while meeting all functional requirements.

**Alternatives considered**:
- React with Vite: More complex setup, additional dependencies, overkill for simple UI
- Vue.js: Similar complexity to React, not needed for simple interface
- Vanilla HTML/CSS/JS: Simplest approach, no build steps, directly serves requirements

## Decision: API Integration Pattern
**Rationale**: Using the native `fetch()` API for calling the backend endpoints provides a simple, modern approach without additional dependencies. The API endpoint `/rag/query` is already implemented and tested.

**Alternatives considered**:
- Axios library: Additional dependency, not necessary for simple API calls
- jQuery AJAX: Legacy approach, not needed with modern fetch API
- Native fetch: Built-in browser API, no dependencies, sufficient for use case

## Decision: Chat History Storage
**Rationale**: Session-based storage using browser's in-memory JavaScript variables is chosen to maintain simplicity. This matches the "session-only" approach specified in the edge cases and avoids complexity of local storage or server persistence.

**Alternatives considered**:
- Browser localStorage: Persistent across browser refreshes, adds complexity
- Browser sessionStorage: Persistent across tabs in same session, but lost on close
- In-memory variables: Session-only, simple implementation, matches requirements

## Decision: Configuration Management
**Rationale**: A simple JavaScript configuration file (`config.js`) with a constant for the API base URL provides the necessary flexibility without complex environment variable handling. This works well for static HTML/JS deployment.

**Alternatives considered**:
- Environment variables: Complex for static HTML deployment
- Query parameters: Exposes configuration in URL, less secure
- Configuration file: Simple approach, easy to modify, keeps config centralized