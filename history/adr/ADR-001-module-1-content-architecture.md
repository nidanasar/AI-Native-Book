# ADR-001: Module 1 Content Architecture

> **Scope**: This ADR documents the clustered decisions for how ROS 2 educational content is structured, coded, and sequenced in Module 1 of the AI-Native Textbook.

- **Status:** Accepted
- **Date:** 2025-12-13
- **Feature:** 001-ros2-module
- **Context:** Module 1 introduces ROS 2 as the "robotic nervous system" to learners transitioning from software AI to robotics. The module must balance technical accuracy with pedagogical effectiveness, provide executable code examples, and support the project's constitution principles (Accuracy First, AI-Native Pedagogy, Clarity & Accessibility, Embodied Intelligence Focus, Reproducibility & Traceability).

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? ✅ Yes - sets pattern for all future modules
     2) Alternatives: Multiple viable options considered with tradeoffs? ✅ Yes - documented below
     3) Scope: Cross-cutting concern (not an isolated detail)? ✅ Yes - affects all chapter writing and code examples
-->

## Decision

The following integrated decisions define Module 1's content architecture:

### Code Example Design
- **Pattern**: Multiple focused nodes (one per concept) rather than monolithic examples
- **Complexity**: Direct rclpy usage without abstraction wrappers
- **URDF**: Simplified 6-joint humanoid (torso + 2 arms) rather than full 25+ joint model

### Pedagogical Sequencing
- **Flow**: Concept-first with embedded examples (Concept → System → Example → Summary)
- **Structure**: Theory establishes the nervous system analogy before technical implementation
- **Placement**: Code examples appear within each section, not batched at chapter end

### Technology Stack
- **Content**: Markdown for Docusaurus (MDX-compatible)
- **Code**: Python 3.10+ with rclpy targeting ROS 2 Humble/Iron
- **Validation**: Gazebo simulation for URDF testing

## Consequences

### Positive

- **Learning Clarity**: Focused nodes isolate concepts, preventing cognitive overload
- **Professional Relevance**: Direct rclpy teaches patterns learners will use in real projects
- **Transferability**: Constitution-aligned approach creates template for future modules
- **Reproducibility**: All code is executable and testable in standard ROS 2 environments
- **Analogy Strength**: Concept-first allows nervous system analogy to anchor technical details

### Negative

- **More Files**: Multiple focused nodes means more example files to maintain
- **Boilerplate**: Direct rclpy has more setup code than wrapped approaches
- **Simplified Models**: 6-joint URDF is less realistic than production humanoids
- **Delayed Code**: Concept-first may feel slower for learners eager to code immediately

## Alternatives Considered

### Alternative A: Monolithic Examples + Full URDF + Wrappers

**Components**:
- Single comprehensive node demonstrating all concepts
- Full 25+ joint humanoid URDF
- Custom wrapper library hiding rclpy complexity

**Why Rejected**:
- Monolithic nodes obscure individual concepts and make debugging harder
- Full URDF overwhelms learners before they understand fundamentals
- Wrappers create "magic" that violates constitution principle of clarity
- Not aligned with official ROS 2 documentation patterns

### Alternative B: Example-First Pedagogy

**Components**:
- Start with code, explain concepts after seeing them work
- "Learn by doing" immediate gratification approach

**Why Rejected**:
- Encourages copy-paste without understanding
- Nervous system analogy loses impact when presented after technical details
- Constitution mandates Concept → System → Example → Summary pattern
- Higher risk of misconceptions when concepts aren't established first

### Alternative C: ROS 1 Style with Migration Notes

**Components**:
- Include ROS 1 examples alongside ROS 2
- Provide migration guidance

**Why Rejected**:
- Out of scope for Module 1 (spec explicitly excludes ROS 1)
- Adds confusion for learners who don't know ROS 1
- Increases maintenance burden

## References

- Feature Spec: [specs/001-ros2-module/spec.md](../../specs/001-ros2-module/spec.md)
- Implementation Plan: [specs/001-ros2-module/plan.md](../../specs/001-ros2-module/plan.md)
- Research: [specs/001-ros2-module/research.md](../../specs/001-ros2-module/research.md)
- Related ADRs: None (first ADR for this project)
- Constitution: [.specify/memory/constitution.md](../../.specify/memory/constitution.md)
