"""
Centralized configuration for the RAG API.

Uses pydantic-settings to load environment variables from .env file.
Configured to use Gemini (free tier) via OpenAI SDK compatibility.
"""

from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Existing settings (from Spec 3-4)
    cohere_api_key: str
    qdrant_url: str
    qdrant_api_key: str
    target_url: str = "https://ai-native-book-1fj5bdpkr-nida-nasars-projects.vercel.app"

    # LLM settings (Gemini via OpenAI SDK)
    gemini_api_key: str
    llm_model_name: str = "gemini-2.5-flash"

    # Gemini OpenAI-compatible endpoint
    llm_base_url: str = "https://generativelanguage.googleapis.com/v1beta/openai/"

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"


# Global settings instance
settings = Settings()
