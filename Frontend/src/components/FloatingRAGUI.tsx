import React, { useState, FormEvent, useEffect } from 'react';
import './floating-rag-ui.css';

// Configuration for the RAG API
declare global {
  interface Window {
    RAG_CONFIG?: {
      API_BASE_URL?: string;
    };
  }
}

const CONFIG = {
  API_BASE_URL: window.RAG_CONFIG?.API_BASE_URL || 'http://localhost:8000',
  DEFAULT_TOP_K: 5,
  MAX_CHAT_HISTORY: 20
};

interface ContextChunk {
  text: string;
  url: string;
  title: string;
  score: number;
}

interface RAGResponse {
  answer: string;
  context_chunks: ContextChunk[];
  metadata: {
    used_top_k: number;
    model_name: string;
  };
}

interface ChatHistoryItem {
  id: string;
  question: string;
  response: RAGResponse;
  timestamp: Date;
}

const FloatingRAGUI: React.FC = () => {
  const [isOpen, setIsOpen] = useState<boolean>(false);
  const [question, setQuestion] = useState<string>('');
  const [topK, setTopK] = useState<number>(CONFIG.DEFAULT_TOP_K);
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);
  const [answer, setAnswer] = useState<string>('');
  const [chatHistory, setChatHistory] = useState<ChatHistoryItem[]>([]);
  const [showHistory, setShowHistory] = useState<boolean>(false);

  // Toggle the RAG UI panel
  const togglePanel = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      setError(null);
      setAnswer('');
    }
  };

  // Call RAG API with timeout
  const callRagAPI = async (questionText: string, topKValue: number) => {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), 30000); // 30 second timeout

    try {
      const response = await fetch(`${CONFIG.API_BASE_URL}/rag/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question: questionText.trim(),
          top_k: topKValue
        }),
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data: RAGResponse = await response.json();
      return data;
    } catch (err) {
      clearTimeout(timeoutId);

      if (err instanceof Error) {
        if (err.name === 'AbortError') {
          throw new Error('Request timed out. The query took too long to complete.');
        } else {
          throw err;
        }
      } else {
        throw new Error('An unknown error occurred while calling the RAG API.');
      }
    }
  };

  const handleSubmit = async (e: FormEvent) => {
    e.preventDefault();
    if (!question.trim()) {
      setError('Please enter a question');
      return;
    }

    setIsLoading(true);
    setError(null);
    setAnswer('');

    try {
      const data = await callRagAPI(question, topK);

      // Add to chat history
      const newHistoryItem: ChatHistoryItem = {
        id: Date.now().toString(),
        question: question.trim(),
        response: data,
        timestamp: new Date()
      };

      setChatHistory(prev => [newHistoryItem, ...prev].slice(0, CONFIG.MAX_CHAT_HISTORY));
      setAnswer(data.answer);
    } catch (err) {
      console.error('Error calling RAG API:', err);

      if (err instanceof Error) {
        setError(err.message);
      } else {
        setError('Failed to get response from RAG API. Please check that the backend is running and CORS is enabled.');
      }
    } finally {
      setIsLoading(false);
    }
  };

  const clearHistory = () => {
    setChatHistory([]);
    setAnswer('');
  };

  // Handle Enter key for submission
  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && (e.ctrlKey || e.metaKey)) {
      handleSubmit(e as any);
    }
  };

  return (
    <>
      {/* Floating button */}
      <button
        className={`floating-rag-btn ${isOpen ? 'open' : ''}`}
        onClick={togglePanel}
        aria-label={isOpen ? "Close RAG interface" : "Open RAG interface"}
        title="Ask about Physical AI & Humanoid Robotics"
      >
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M12 2C6.48 2 2 6.48 2 12C2 17.52 6.48 22 12 22C14.03 22 15.91 21.37 17.5 20.29L19.75 22.54L21.16 21.13L18.91 18.88C20.08 17.25 20.8 15.25 20.8 13C20.8 7.48 16.52 3 11 3ZM11 19C8.24 19 6 16.76 6 14C6 11.24 8.24 9 11 9C13.76 9 16 11.24 16 14C16 16.76 13.76 19 11 19ZM11 11C9.34 11 8 12.34 8 14C8 15.66 9.34 17 11 17C12.66 17 14 15.66 14 14C14 12.34 12.66 11 11 11Z" fill="currentColor"/>
        </svg>
      </button>

      {/* RAG UI Panel */}
      {isOpen && (
        <div className="floating-rag-panel">
          <div className="rag-panel-header">
            <h3>Physical AI & Humanoid Robotics RAG</h3>
            <button
              className="close-btn"
              onClick={togglePanel}
              aria-label="Close"
            >
              ×
            </button>
          </div>

          <form onSubmit={handleSubmit} className="query-form">
            <div className="input-group">
              <label htmlFor="floating-question">Your Question:</label>
              <textarea
                id="floating-question"
                value={question}
                onChange={(e) => setQuestion(e.target.value)}
                onKeyDown={handleKeyDown}
                placeholder="Ask about Physical AI & Humanoid Robotics... (Ctrl+Enter to submit)"
                rows={3}
                disabled={isLoading}
              />
            </div>

            <div className="input-group">
              <label htmlFor="floating-topK">Context Chunks (top_k):</label>
              <select
                id="floating-topK"
                value={topK}
                onChange={(e) => setTopK(Number(e.target.value))}
                disabled={isLoading}
              >
                {[1, 2, 3, 4, 5, 6, 7, 8, 9, 10].map(num => (
                  <option key={num} value={num}>{num}</option>
                ))}
              </select>
            </div>

            <button type="submit" disabled={isLoading} className="submit-btn">
              {isLoading ? 'Processing...' : 'Submit Question'}
            </button>
          </form>

          {error && (
            <div className="error-message">
              <h4>Error:</h4>
              <p>{error}</p>
            </div>
          )}

          {answer && (
            <div className="response-section">
              <h4>Answer:</h4>
              <div className="answer">{answer}</div>
            </div>
          )}

          {chatHistory.length > 0 && (
            <div className="chat-history-section">
              <div className="history-header">
                <div className="history-toggle">
                  <button
                    onClick={() => setShowHistory(!showHistory)}
                    className="toggle-history-btn"
                  >
                    {showHistory ? 'Hide History' : `Show History (${chatHistory.length})`}
                  </button>
                </div>
                <button onClick={clearHistory} className="clear-history-btn">
                  Clear
                </button>
              </div>

              {showHistory && (
                <div className="chat-history">
                  {chatHistory.map((item) => (
                    <div key={item.id} className="history-item">
                      <div className="history-question">
                        <strong>Q: {item.question}</strong>
                      </div>
                      <div className="history-answer">
                        <p>{item.response.answer}</p>
                      </div>
                    </div>
                  ))}
                </div>
              )}
            </div>
          )}
        </div>
      )}
    </>
  );
};

export default FloatingRAGUI;