# Physical AI & Humanoid Robotics Documentation

This website is built with [Docusaurus](https://docusaurus.io/), a modern static website generator.

## RAG Query Interface

The RAG (Retrieval-Augmented Generation) Query Interface is available as a floating button on the bottom right of every page and allows you to ask questions about Physical AI & Humanoid Robotics content using our AI-powered system.

### Features
- Floating interface accessible from any page
- Ask natural language questions about the textbook content
- View AI-generated answers with supporting context
- See source documents and text snippets used for answers
- Adjust the number of context chunks used (top_k parameter)
- Chat history to maintain conversation context

### Prerequisites

Before using the RAG interface, ensure the backend API is running:
1. The FastAPI backend with the `/rag/query` endpoint must be accessible
2. Set the `REACT_APP_API_BASE_URL` environment variable to point to your backend (default: `http://localhost:8000`)

### Installation

```
$ yarn
```

### Local Development

```
$ yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

To configure the backend API URL for development:
```
REACT_APP_API_BASE_URL=http://localhost:8000 yarn start
```

### Build

```
$ yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

### Configuration

The RAG interface can be configured using environment variables during build time:

- `RAG_API_BASE_URL`: The base URL for the RAG API backend (default: `http://localhost:8000`)

To set the API URL when running the development server:

```bash
RAG_API_BASE_URL=https://your-backend-url.com yarn start:rag
```

Or for production build:

```bash
RAG_API_BASE_URL=https://your-backend-url.com yarn build:rag
```

### Deployment

Using SSH:

```
$ USE_SSH=true yarn deploy
```

Not using SSH:

```
$ GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.