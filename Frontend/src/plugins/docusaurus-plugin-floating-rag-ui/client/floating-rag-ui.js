import React from 'react';
import { createRoot } from 'react-dom/client';
import FloatingRAGUI from '@site/src/components/FloatingRAGUI';

// Create a container for the floating RAG UI
function injectFloatingRAGUI() {
  // Create a container div
  const container = document.createElement('div');
  container.id = 'floating-rag-ui-container';
  document.body.appendChild(container);

  // Render the FloatingRAGUI component into the container using the new React 18 API
  const root = createRoot(container);
  root.render(<FloatingRAGUI />);
}

// Wait for the DOM to be fully loaded before injecting the component
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', injectFloatingRAGUI);
} else {
  // DOM is already ready, inject immediately
  injectFloatingRAGUI();
}

// For SPA navigation (client-side routing in Docusaurus)
// Re-inject when route changes
if (typeof window !== 'undefined') {
  let observer;

  // Function to initialize when page content changes
  function onRouteUpdate() {
    // Check if container already exists to avoid duplicates
    if (!document.getElementById('floating-rag-ui-container')) {
      injectFloatingRAGUI();
    }
  }

  // Set up a simple route change detection
  const originalPushState = history.pushState;
  history.pushState = function() {
    originalPushState.apply(history, arguments);
    setTimeout(onRouteUpdate, 100); // Small delay to ensure page content is updated
  };

  // Listen for popstate events (browser back/forward)
  window.addEventListener('popstate', () => {
    setTimeout(onRouteUpdate, 100);
  });
}