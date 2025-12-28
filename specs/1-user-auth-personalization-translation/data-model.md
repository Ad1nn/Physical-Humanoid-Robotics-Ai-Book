# Data Model: User Authentication, Personalization & Translation

This document defines the database schema for the new features.

## User Profile Storage (Postgres)

```sql
CREATE TABLE users (
    id UUID PRIMARY KEY,
    email VARCHAR UNIQUE,
    password_hash VARCHAR,
    created_at TIMESTAMP,
    last_login TIMESTAMP
);

CREATE TABLE user_profiles (
    user_id UUID REFERENCES users(id),
    python_level VARCHAR,
    ros_knowledge VARCHAR,
    ai_ml_background VARCHAR,
    simulation_experience VARCHAR,
    linux_comfort VARCHAR,
    hardware_experience VARCHAR,
    gpu_availability VARCHAR,
    learning_goal TEXT,
    updated_at TIMESTAMP
);

CREATE TABLE user_preferences (
    user_id UUID REFERENCES users(id),
    personalization_enabled BOOLEAN DEFAULT true,
    language_preference VARCHAR DEFAULT 'en',
    theme VARCHAR DEFAULT 'dark'
);
```

## Caching Strategy

```sql
CREATE TABLE translations (
    id SERIAL PRIMARY KEY,
    chapter_id VARCHAR,
    source_language VARCHAR,
    target_language VARCHAR,
    content_hash VARCHAR,
    translated_content TEXT,
    created_at TIMESTAMP
);
```
