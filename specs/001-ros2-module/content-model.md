# Content Model: Module 1 - The Robotic Nervous System (ROS 2)

**Date**: 2025-12-13
**Branch**: `001-ros2-module`

## Overview

This document defines the content structure, entities, and relationships for Module 1. It serves as the "data model" for educational content—defining what types of content exist and how they relate.

---

## Content Entities

### 1. Module

**Definition**: A collection of related chapters forming a cohesive learning unit.

| Field | Type | Description |
|-------|------|-------------|
| id | string | Unique identifier (e.g., "module-1-ros2") |
| title | string | Display title |
| description | string | One-paragraph summary |
| chapters | Chapter[] | Ordered list of chapters |
| prerequisites | string[] | Required knowledge before starting |
| learning_outcomes | string[] | What learners achieve after completion |

**Instance**: Module 1

```yaml
id: module-1-ros2
title: "The Robotic Nervous System (ROS 2)"
description: "Introduction to ROS 2 middleware for humanoid robotics"
chapters: [ch1, ch2, ch3, ch4]
prerequisites:
  - Python programming (functions, classes)
  - Basic computer science concepts
learning_outcomes:
  - Define ROS 2 and explain the middleware concept
  - Identify when to use topics vs services
  - Write Python code using rclpy
  - Read and interpret URDF files
```

---

### 2. Chapter

**Definition**: A self-contained learning unit with objectives, content sections, examples, and summary.

| Field | Type | Description |
|-------|------|-------------|
| id | string | Unique identifier |
| title | string | Chapter title |
| sidebar_position | number | Order in navigation |
| learning_objectives | string[] | Measurable outcomes (3-5 per chapter) |
| sections | Section[] | Content sections |
| code_examples | CodeExample[] | Embedded code |
| summary | string | Key takeaways |
| next_chapter | string | Link to next chapter |

**Structure Template**:

```markdown
---
sidebar_position: N
title: "Chapter N: Title"
description: "Brief description for SEO"
---

# Chapter Title

:::info Learning Objectives
- Objective 1
- Objective 2
- Objective 3
:::

## Section 1: Concept Introduction
[Conceptual explanation with analogy]

## Section 2: System Architecture
[How it fits in the ROS 2 ecosystem]

## Section 3: Practical Example
[Code example with explanation]

## Summary
[Key takeaways in bullet form]

---

:::tip Next Chapter
Continue to [Chapter N+1](./next-chapter) to learn...
:::
```

---

### 3. Section

**Definition**: A subsection within a chapter covering a specific topic.

| Field | Type | Description |
|-------|------|-------------|
| heading | string | Section title (H2) |
| content_type | enum | concept, architecture, example, summary |
| body | string | Markdown content |
| diagrams | DiagramDescription[] | Optional diagrams |
| callouts | Callout[] | Tips, warnings, info boxes |

**Content Types**:

| Type | Purpose | Typical Length |
|------|---------|----------------|
| concept | Introduce and explain an idea | 200-400 words |
| architecture | Show system relationships | 150-300 words + diagram |
| example | Demonstrate with code | 100-200 words + code |
| summary | Consolidate learning | 100-150 words |

---

### 4. CodeExample

**Definition**: An executable code snippet demonstrating a concept.

| Field | Type | Description |
|-------|------|-------------|
| id | string | Unique identifier |
| language | string | Programming language (python) |
| filename | string | Source file reference |
| title | string | Example title |
| code | string | The code itself |
| explanation | string | Line-by-line or block explanation |
| output | string | Expected output when run |
| dependencies | string[] | Required packages |

**Example Instance**:

```yaml
id: minimal-publisher
language: python
filename: examples/module-1/minimal_publisher.py
title: "Minimal Publisher Node"
dependencies:
  - rclpy
  - std_msgs
```

---

### 5. DiagramDescription

**Definition**: Textual description of a diagram (rendered separately).

| Field | Type | Description |
|-------|------|-------------|
| id | string | Unique identifier |
| type | enum | architecture, sequence, component, flowchart |
| title | string | Diagram title |
| description | string | Full textual description |
| elements | string[] | Key elements shown |
| relationships | string[] | Connections between elements |

**Example Instance**:

```yaml
id: ros2-communication-diagram
type: architecture
title: "ROS 2 Node Communication"
description: "Shows three nodes communicating via topics and services"
elements:
  - "Sensor Node (publishes sensor data)"
  - "Processing Node (subscribes to sensor, publishes commands)"
  - "Actuator Node (subscribes to commands)"
relationships:
  - "Sensor Node --[topic: /sensor_data]--> Processing Node"
  - "Processing Node --[topic: /motor_commands]--> Actuator Node"
```

---

### 6. Callout

**Definition**: A highlighted box for tips, warnings, or additional information.

| Field | Type | Description |
|-------|------|-------------|
| type | enum | tip, info, warning, danger |
| content | string | Callout text |

**Docusaurus Syntax**:

```markdown
:::tip Title
Content here
:::

:::info
Information content
:::

:::warning
Warning content
:::

:::danger
Critical warning
:::
```

---

## Chapter Specifications

### Chapter 1: Introduction to ROS 2 and Robotic Middleware

| Field | Value |
|-------|-------|
| File | `01-introduction-ros2.md` |
| Word Count Target | 3,000-3,500 |
| Reading Time | ~15 minutes |

**Sections**:
1. What is ROS 2? (concept)
2. The Nervous System Analogy (concept)
3. ROS 2 Architecture Overview (architecture)
4. Why Middleware Matters (concept)
5. Summary

**Key Concepts**:
- Middleware definition
- Nodes as specialized processors
- Distributed communication
- Comparison with monolithic code

---

### Chapter 2: Nodes, Topics, and Services

| Field | Value |
|-------|-------|
| File | `02-nodes-topics-services.md` |
| Word Count Target | 4,000-4,500 |
| Reading Time | ~20 minutes |

**Sections**:
1. Understanding Nodes (concept)
2. Topics: Publish/Subscribe (concept + architecture)
3. Services: Request/Response (concept + architecture)
4. When to Use Each Pattern (example scenarios)
5. Summary

**Key Concepts**:
- Node lifecycle
- Topic message types
- QoS basics
- Service definitions
- Decision framework

---

### Chapter 3: Bridging Python Agents to ROS 2 Controllers

| Field | Value |
|-------|-------|
| File | `03-python-rclpy-bridge.md` |
| Word Count Target | 4,500-5,000 |
| Reading Time | ~25 minutes |

**Sections**:
1. Introduction to rclpy (concept)
2. Creating Your First Node (example)
3. Publishing and Subscribing (example)
4. Working with Services (example)
5. Complete Working Example (example)
6. Summary

**Code Examples**:
- `minimal_publisher.py`
- `minimal_subscriber.py`
- `simple_service.py`
- `service_client.py`

---

### Chapter 4: Understanding URDF for Humanoid Robots

| Field | Value |
|-------|-------|
| File | `04-urdf-humanoid-robots.md` |
| Word Count Target | 3,500-4,000 |
| Reading Time | ~18 minutes |

**Sections**:
1. What is URDF? (concept)
2. Links and Joints Explained (concept + architecture)
3. Building a Simple Humanoid (example)
4. Connecting URDF to ROS 2 (architecture)
5. Summary

**Code Examples**:
- `simple_humanoid.urdf`

---

## Content Relationships

```
Module
  └── Chapter (1:N)
        ├── Section (1:N)
        │     ├── DiagramDescription (0:N)
        │     └── Callout (0:N)
        ├── CodeExample (0:N)
        └── Summary (1:1)
```

---

## Quality Validation Rules

### Per Section

- [ ] Content type matches expected pattern
- [ ] Word count within range for type
- [ ] Technical terms defined on first use
- [ ] No invented APIs or capabilities

### Per Chapter

- [ ] Learning objectives are measurable
- [ ] All sections follow Concept → System → Example → Summary
- [ ] Nervous system analogy maintained
- [ ] Code examples are complete and executable
- [ ] Summary covers all learning objectives

### Per Module

- [ ] Chapters build on each other logically
- [ ] Prerequisites clearly stated
- [ ] Learning outcomes achievable
- [ ] Consistent terminology throughout
