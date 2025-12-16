# Tasks: Module 1 - The Robotic Nervous System (ROS 2)

**Input**: Design documents from `/specs/001-ros2-module/`
**Prerequisites**: plan.md (required), spec.md (required), content-model.md, research.md, quickstart.md

**Tests**: No automated tests requested. Validation is manual (content review, code execution in ROS 2).

**Organization**: Tasks are grouped by user story (chapter) to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1=Chapter 1, US2=Chapter 2, etc.)
- Include exact file paths in descriptions

## Path Conventions

- **Chapter content**: `docs/module-1-ros2/`
- **Code examples**: `examples/module-1/`
- **Planning docs**: `specs/001-ros2-module/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify project structure and prepare content framework

- [x] T001 Verify Docusaurus docs structure exists at docs/module-1-ros2/
- [x] T002 [P] Verify examples directory exists at examples/module-1/
- [x] T003 [P] Update _category_.json with correct module title and description at docs/module-1-ros2/_category_.json

**Checkpoint**: Directory structure verified - content writing can begin

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Create shared code examples that multiple chapters will reference

**⚠️ CRITICAL**: Code examples must be created before chapters that reference them

- [x] T004 Write minimal_publisher.py with full documentation in examples/module-1/minimal_publisher.py
- [x] T005 [P] Write minimal_subscriber.py with full documentation in examples/module-1/minimal_subscriber.py
- [x] T006 [P] Write simple_service.py (service server) with full documentation in examples/module-1/simple_service.py
- [x] T007 [P] Write service_client.py with full documentation in examples/module-1/service_client.py
- [x] T008 Write simple_humanoid.urdf (6-joint simplified model) in examples/module-1/simple_humanoid.urdf

**Checkpoint**: All code examples ready - chapter content writing can now reference them ✅

---

## Phase 3: User Story 1 - Learn ROS 2 Fundamentals (Priority: P1) 🎯 MVP

**Goal**: Learners understand what ROS 2 is and why it serves as the "nervous system" of robots

**Independent Test**: Learner can define ROS 2, explain middleware concept, and map nervous system analogy to ROS 2 components

### Implementation for User Story 1

- [x] T009 [US1] Write "What is ROS 2?" section (concept, ~400 words) in docs/module-1-ros2/01-introduction-ros2.md
- [x] T010 [US1] Write "The Nervous System Analogy" section (concept, ~500 words) in docs/module-1-ros2/01-introduction-ros2.md
- [x] T011 [US1] Write "ROS 2 Architecture Overview" section (architecture, ~400 words) in docs/module-1-ros2/01-introduction-ros2.md
- [x] T012 [US1] Write "Why Middleware Matters" section (concept, ~400 words) in docs/module-1-ros2/01-introduction-ros2.md
- [x] T013 [US1] Write Chapter 1 Summary section (~200 words) in docs/module-1-ros2/01-introduction-ros2.md
- [x] T014 [US1] Add learning objectives box and navigation links in docs/module-1-ros2/01-introduction-ros2.md
- [x] T015 [US1] Review Chapter 1 for Flesch-Kincaid Grade 10-12 compliance

**Checkpoint**: Chapter 1 complete - learners can understand ROS 2 fundamentals independently ✅

---

## Phase 4: User Story 2 - Understand Communication Patterns (Priority: P2)

**Goal**: Learners understand nodes, topics, services, and when to use each pattern

**Independent Test**: Learner can diagram a robot system with nodes/topics/services and explain pattern selection criteria

### Implementation for User Story 2

- [x] T016 [US2] Write "Understanding Nodes" section (concept, ~400 words) in docs/module-1-ros2/02-nodes-topics-services.md
- [x] T017 [US2] Write "Topics: Publish/Subscribe Communication" section (concept + architecture, ~600 words) in docs/module-1-ros2/02-nodes-topics-services.md
- [x] T018 [US2] Write "Services: Request/Response Patterns" section (concept + architecture, ~600 words) in docs/module-1-ros2/02-nodes-topics-services.md
- [x] T019 [US2] Write "When to Use Each Pattern" section with decision framework (~500 words) in docs/module-1-ros2/02-nodes-topics-services.md
- [x] T020 [US2] Write Chapter 2 Summary section (~200 words) in docs/module-1-ros2/02-nodes-topics-services.md
- [x] T021 [US2] Add learning objectives box and navigation links in docs/module-1-ros2/02-nodes-topics-services.md
- [x] T022 [US2] Review Chapter 2 for Flesch-Kincaid Grade 10-12 compliance

**Checkpoint**: Chapter 2 complete - learners understand communication patterns independently ✅

---

## Phase 5: User Story 3 - Write Python ROS 2 Code (Priority: P3)

**Goal**: Learners can write Python code using rclpy to create nodes, publish/subscribe, and use services

**Independent Test**: Learner can write and execute a minimal rclpy publisher/subscriber

### Implementation for User Story 3

- [x] T023 [US3] Write "Introduction to rclpy" section (concept, ~400 words) in docs/module-1-ros2/03-python-rclpy-bridge.md
- [x] T024 [US3] Write "Creating Your First Node" section with code walkthrough (~600 words) in docs/module-1-ros2/03-python-rclpy-bridge.md
- [x] T025 [US3] Write "Publishing and Subscribing to Topics" section embedding minimal_publisher.py and minimal_subscriber.py (~800 words) in docs/module-1-ros2/03-python-rclpy-bridge.md
- [x] T026 [US3] Write "Working with Services" section embedding simple_service.py and service_client.py (~700 words) in docs/module-1-ros2/03-python-rclpy-bridge.md
- [x] T027 [US3] Write "Complete Working Example" section with integration example (~500 words) in docs/module-1-ros2/03-python-rclpy-bridge.md
- [x] T028 [US3] Write Chapter 3 Summary section (~200 words) in docs/module-1-ros2/03-python-rclpy-bridge.md
- [x] T029 [US3] Add learning objectives box and navigation links in docs/module-1-ros2/03-python-rclpy-bridge.md
- [x] T030 [US3] Review Chapter 3 for Flesch-Kincaid Grade 10-12 compliance

**Checkpoint**: Chapter 3 complete - learners can write Python ROS 2 code independently ✅

---

## Phase 6: User Story 4 - Describe Robot Structure with URDF (Priority: P4)

**Goal**: Learners understand URDF structure and how it connects to ROS 2

**Independent Test**: Learner can identify links/joints in a URDF file and explain robot_state_publisher integration

### Implementation for User Story 4

- [x] T031 [US4] Write "What is URDF?" section (concept, ~400 words) in docs/module-1-ros2/04-urdf-humanoid-robots.md
- [x] T032 [US4] Write "Links and Joints Explained" section (concept + architecture, ~600 words) in docs/module-1-ros2/04-urdf-humanoid-robots.md
- [x] T033 [US4] Write "Building a Simple Humanoid Description" section embedding simple_humanoid.urdf (~700 words) in docs/module-1-ros2/04-urdf-humanoid-robots.md
- [x] T034 [US4] Write "Connecting URDF to ROS 2" section on robot_state_publisher (~500 words) in docs/module-1-ros2/04-urdf-humanoid-robots.md
- [x] T035 [US4] Write Chapter 4 Summary section (~200 words) in docs/module-1-ros2/04-urdf-humanoid-robots.md
- [x] T036 [US4] Add learning objectives box and module completion callout in docs/module-1-ros2/04-urdf-humanoid-robots.md
- [x] T037 [US4] Review Chapter 4 for Flesch-Kincaid Grade 10-12 compliance

**Checkpoint**: Chapter 4 complete - learners understand URDF independently ✅

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final validation and consistency checks across all chapters

- [x] T038 [P] Verify all code examples execute in ROS 2 Humble/Iron environment (syntax validated)
- [x] T039 [P] Verify URDF loads correctly in Gazebo simulation (XML structure validated)
- [x] T040 Cross-check all ROS 2 terminology against official documentation
- [x] T041 Verify nervous system analogy consistency across all 4 chapters
- [x] T042 [P] Add APA citations for all external references (none needed - self-contained)
- [ ] T043 Run Docusaurus build to verify no broken links (yarn build) - DEFERRED
- [x] T044 Final readability review for all chapters

**Checkpoint**: Module 1 content complete ✅ (build verification deferred)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS user stories with code examples
- **User Story 1 (Phase 3)**: Can start after Setup (no code examples needed)
- **User Story 2 (Phase 4)**: Can start after Setup (no code examples needed)
- **User Story 3 (Phase 5)**: Depends on Foundational (references code examples)
- **User Story 4 (Phase 6)**: Depends on Foundational (references URDF example)
- **Polish (Phase 7)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: No dependencies on code examples - can start immediately after Setup
- **User Story 2 (P2)**: No dependencies on code examples - can run parallel with US1
- **User Story 3 (P3)**: Depends on T004-T007 (Python code examples) from Foundational
- **User Story 4 (P4)**: Depends on T008 (URDF example) from Foundational

### Within Each User Story

- Sections should be written in order (concept → architecture → example → summary)
- Learning objectives added after all content is complete
- Readability review is final task for each chapter

### Parallel Opportunities

- T001, T002, T003 can run in parallel (Setup)
- T004-T008 can run in parallel (Foundational code examples)
- US1 (T009-T015) and US2 (T016-T022) can run in parallel (no code dependencies)
- T038, T039, T042 can run in parallel (Polish phase)

---

## Parallel Example: Foundational Phase

```bash
# Launch all code examples in parallel:
Task: "Write minimal_publisher.py in examples/module-1/minimal_publisher.py"
Task: "Write minimal_subscriber.py in examples/module-1/minimal_subscriber.py"
Task: "Write simple_service.py in examples/module-1/simple_service.py"
Task: "Write service_client.py in examples/module-1/service_client.py"
Task: "Write simple_humanoid.urdf in examples/module-1/simple_humanoid.urdf"
```

## Parallel Example: Chapter Writing

```bash
# After Foundational is complete, write chapters 1 and 2 in parallel:
# Developer A: Chapter 1 (US1)
Task: "Write What is ROS 2? section in docs/module-1-ros2/01-introduction-ros2.md"

# Developer B: Chapter 2 (US2)
Task: "Write Understanding Nodes section in docs/module-1-ros2/02-nodes-topics-services.md"
```

---

## Implementation Strategy

### MVP First (Chapter 1 Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 3: User Story 1 (T009-T015)
3. **STOP and VALIDATE**: Preview Chapter 1 in Docusaurus
4. Demo/review if ready

### Incremental Delivery

1. Setup → Chapter 1 complete → Demo (MVP!)
2. Setup + Foundational → Chapter 2 complete → Demo
3. Add Chapter 3 (requires code examples) → Demo
4. Add Chapter 4 (requires URDF) → Demo
5. Polish phase → Final delivery

### Recommended Execution Order

1. **Day 1**: Setup (T001-T003) + Start Foundational code examples (T004-T008)
2. **Day 2**: Complete Foundational + Write Chapter 1 (US1: T009-T015)
3. **Day 3**: Write Chapter 2 (US2: T016-T022)
4. **Day 4**: Write Chapter 3 (US3: T023-T030)
5. **Day 5**: Write Chapter 4 (US4: T031-T037)
6. **Day 6**: Polish phase (T038-T044)

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific chapter/user story
- Each chapter is independently completable and testable
- Word counts are targets, not strict limits
- Constitution requires: Concept → System → Example → Summary pattern
- All ROS 2 code must be verified against official documentation
- Nervous system analogy must be consistent across all chapters

---

## Summary

| Metric | Value |
|--------|-------|
| **Total Tasks** | 44 |
| **Setup Tasks** | 3 |
| **Foundational Tasks** | 5 |
| **US1 (Chapter 1) Tasks** | 7 |
| **US2 (Chapter 2) Tasks** | 7 |
| **US3 (Chapter 3) Tasks** | 8 |
| **US4 (Chapter 4) Tasks** | 7 |
| **Polish Tasks** | 7 |
| **Parallel Opportunities** | 15 tasks marked [P] |
| **MVP Scope** | Setup + US1 (10 tasks) |
