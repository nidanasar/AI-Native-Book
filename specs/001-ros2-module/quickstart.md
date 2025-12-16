# Quickstart: Module 1 Content Creation

**Date**: 2025-12-13
**Branch**: `001-ros2-module`

This guide provides step-by-step instructions for creating content for Module 1: The Robotic Nervous System (ROS 2).

---

## Prerequisites

Before starting content creation:

1. **Read the specification**: `specs/001-ros2-module/spec.md`
2. **Review the plan**: `specs/001-ros2-module/plan.md`
3. **Check research findings**: `specs/001-ros2-module/research.md`
4. **Understand content model**: `specs/001-ros2-module/content-model.md`
5. **Review constitution**: `.specify/memory/constitution.md`

---

## Content Locations

| Content Type | Location |
|--------------|----------|
| Chapter markdown | `docs/module-1-ros2/` |
| Code examples | `examples/module-1/` |
| Planning docs | `specs/001-ros2-module/` |

---

## Chapter Writing Workflow

### Step 1: Set Up Chapter File

Each chapter file already exists with placeholder content:

```
docs/module-1-ros2/
├── 01-introduction-ros2.md
├── 02-nodes-topics-services.md
├── 03-python-rclpy-bridge.md
└── 04-urdf-humanoid-robots.md
```

### Step 2: Follow the Section Pattern

For each section, follow the constitution-mandated pattern:

```
Concept → System Architecture → Practical Example → Summary
```

**Section Template**:

```markdown
## Section Title

[1-2 paragraph concept explanation using clear, accessible language]

[Nervous system analogy connection if applicable]

### How It Works

[System-level explanation with architecture context]

### Example

[Code or concrete example with explanation]

> **Key Point**: [One-sentence takeaway]
```

### Step 3: Use Docusaurus Features

**Learning Objectives Box**:
```markdown
:::info Learning Objectives
By the end of this chapter, you will be able to:
- Objective 1
- Objective 2
- Objective 3
:::
```

**Tips**:
```markdown
:::tip
Helpful tip content here
:::
```

**Warnings**:
```markdown
:::warning
Important warning content here
:::
```

**Code Blocks with Titles**:
```markdown
```python title="examples/module-1/minimal_publisher.py"
# Code here
```
```

### Step 4: Embed Code Examples

For Python code in Chapter 3:

1. Write complete, executable code in `examples/module-1/`
2. Embed in chapter with title and line numbers
3. Add explanation below each code block

**Format**:

```markdown
```python title="minimal_publisher.py" showLineNumbers
import rclpy
from rclpy.node import Node
# ... rest of code
```

**Explanation**: This code creates a ROS 2 node that...
```

### Step 5: Add Navigation Links

At the end of each chapter:

```markdown
---

:::tip Next Chapter
Continue to [Chapter N+1: Title](./filename) to learn about...
:::
```

For the last chapter:

```markdown
---

:::info Module Complete
Congratulations! You have completed Module 1...
:::
```

---

## Quality Checklist Per Chapter

Before marking a chapter complete:

- [ ] Learning objectives at the start (3-5 measurable outcomes)
- [ ] All sections follow Concept → System → Example → Summary
- [ ] Technical terms defined on first use
- [ ] Nervous system analogy used where appropriate
- [ ] Code examples are complete and tested
- [ ] No invented APIs or capabilities
- [ ] Summary covers all learning objectives
- [ ] Navigation link to next chapter
- [ ] Flesch-Kincaid Grade 10-12 readability
- [ ] APA citations for external references

---

## Code Example Requirements

### Python (rclpy) Examples

All Python code must:

1. Be complete and executable (no snippets without context)
2. Include necessary imports
3. Follow ROS 2 conventions (node naming, topic naming)
4. Include comments explaining key lines
5. Be tested in ROS 2 Humble/Iron

**Template**:

```python
#!/usr/bin/env python3
"""
Brief description of what this example demonstrates.

Dependencies:
- rclpy
- std_msgs (or other packages)

Run with:
    ros2 run package_name node_name
    # or
    python3 filename.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ExampleNode(Node):
    """Example node demonstrating [concept]."""

    def __init__(self):
        super().__init__('example_node')
        # Initialization code with comments

    # Methods with docstrings

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### URDF Examples

All URDF must:

1. Be valid XML
2. Include required elements (robot, link, joint)
3. Be testable in RViz/Gazebo
4. Include comments explaining structure

---

## Terminology Reference

Use consistent terminology throughout:

| Term | Definition | First Use |
|------|------------|-----------|
| Node | A process performing computation in ROS 2 | Chapter 1 |
| Topic | Named bus for publish/subscribe messaging | Chapter 1 |
| Service | Request/response communication pattern | Chapter 2 |
| Publisher | Entity that sends messages to a topic | Chapter 2 |
| Subscriber | Entity that receives messages from a topic | Chapter 2 |
| Message | Structured data exchanged via topics/services | Chapter 2 |
| rclpy | ROS 2 Python client library | Chapter 3 |
| URDF | Unified Robot Description Format | Chapter 4 |
| Link | Rigid body in a robot model | Chapter 4 |
| Joint | Connection between links with motion constraints | Chapter 4 |

---

## Running the Dev Server

To preview content as you write:

```bash
cd hackathon-text-book
yarn start
```

Navigate to `http://localhost:3000/docs/module-1-ros2/01-introduction-ros2` to see your content.

---

## Validation Commands

### Check Markdown Syntax

```bash
# If you have markdownlint installed
markdownlint docs/module-1-ros2/*.md
```

### Test Python Examples

```bash
# In a ROS 2 environment
cd examples/module-1
python3 minimal_publisher.py
```

### Build Check

```bash
yarn build
```

This will catch any broken links or syntax errors.

---

## Support

- **Spec questions**: See `specs/001-ros2-module/spec.md`
- **Architecture decisions**: See `specs/001-ros2-module/plan.md`
- **Technical details**: See `specs/001-ros2-module/research.md`
- **Writing standards**: See `.specify/memory/constitution.md`
