# Implementation Plan: Module 2 - The Digital Twin (Gazebo & Unity)

**Branch**: `002-digital-twin-module` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-digital-twin-module/spec.md`

## Summary

Create educational content teaching learners how to build and use digital twins for humanoid robots. The module covers physics simulation with Gazebo (Fortress/Harmonic), sensor simulation (LiDAR, depth cameras, IMUs), and Unity integration for high-fidelity visualization. Content follows the "virtual rehearsal/imagination" analogy extending Module 1's nervous system metaphor.

## Technical Context

**Content Type**: Educational textbook module (Docusaurus markdown)
**Primary Technologies**:
- New Gazebo (Fortress for ROS 2 Humble, Harmonic for Iron)
- Unity 2021+ with ROS-TCP-Connector
- ROS 2 (Humble/Iron)

**Prerequisites**: Module 1 completion (ROS 2, nodes, topics, URDF)
**Target Platform**: Docusaurus static site, future AI agent augmentation
**Testing**: Manual content review, code syntax validation, reproducibility checks
**Performance Goals**: Flesch-Kincaid Grade 10-12 readability
**Constraints**: No hallucinated APIs, verified against official documentation

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| I. Accuracy First | ✅ PASS | Content based on official Gazebo/Unity docs; no speculative claims |
| II. AI-Native Pedagogy | ✅ PASS | Uses "virtual rehearsal" analogy; system-level thinking |
| III. Clarity & Accessibility | ✅ PASS | Assumes CS background, not robotics; defines terms |
| IV. Embodied Intelligence Focus | ✅ PASS | Explicitly addresses sim-to-real gap, physics constraints |
| V. Reproducibility & Traceability | ✅ PASS | Includes practical Gazebo/Unity exercises |

**Writing Standards**:
- Tone: Academic-professional, approachable ✅
- Structure: Concept → System → Example → Summary ✅
- Chapter structure: Learning objectives, core concepts, practical mapping, summary ✅

**Technical Standards**:
- ROS 2: Humble/Iron, accurate terminology ✅
- Simulation: Gazebo vs Unity roles clearly distinguished ✅
- Sim-to-real challenges addressed ✅

## Project Structure

### Documentation (this feature)

```text
specs/002-digital-twin-module/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── content-model.md     # Phase 1 output (replaces data-model for content)
├── quickstart.md        # Phase 1 output
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Content Structure (docs folder)

```text
docs/module-2-digital-twin/
├── _category_.json
├── 01-introduction-digital-twins.md
├── 02-physics-simulation-gazebo.md
├── 03-simulating-sensors.md
└── 04-unity-visualization-hri.md

examples/module-2/
├── gazebo/
│   ├── simple_world.sdf           # Basic Gazebo world
│   ├── spawn_robot.launch.py      # Robot spawning launch file
│   └── physics_demo.py            # Physics interaction demo
├── sensors/
│   ├── lidar_config.yaml          # LiDAR sensor configuration
│   ├── depth_camera_config.yaml   # Depth camera configuration
│   └── imu_config.yaml            # IMU configuration
└── unity/
    └── ros_connection_example.cs  # Unity-ROS 2 connection (conceptual)
```

**Structure Decision**: Content-focused structure with docs/ for chapters and examples/ for code samples, following Module 1 pattern.

## Architecture Overview

### Digital Twin Simulation Stack

```
┌─────────────────────────────────────────────────────────────────┐
│                    HIGH-FIDELITY VISUALIZATION                   │
│                         (Unity + ROS-TCP)                        │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │  3D Render  │  │   HRI UI    │  │  Photorealistic Scene   │  │
│  └─────────────┘  └─────────────┘  └─────────────────────────┘  │
└────────────────────────────┬────────────────────────────────────┘
                             │ ROS-TCP-Connector
┌────────────────────────────┴────────────────────────────────────┐
│                      ROS 2 MIDDLEWARE                            │
│  Topics: /joint_states, /tf, /camera/image, /scan, /imu         │
└────────────────────────────┬────────────────────────────────────┘
                             │
┌────────────────────────────┴────────────────────────────────────┐
│                    PHYSICS SIMULATION                            │
│                    (Gazebo Fortress/Harmonic)                    │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │   Physics   │  │   Sensors   │  │    World/Environment    │  │
│  │   Engine    │  │   Plugins   │  │       (SDF Files)       │  │
│  └─────────────┘  └─────────────┘  └─────────────────────────┘  │
└────────────────────────────┬────────────────────────────────────┘
                             │
┌────────────────────────────┴────────────────────────────────────┐
│                     ROBOT MODEL (URDF)                           │
│          (From Module 1: simple_humanoid.urdf)                   │
└─────────────────────────────────────────────────────────────────┘
```

### The Virtual Rehearsal Analogy

```
HUMAN BRAIN                          ROBOT DIGITAL TWIN
─────────────────                    ─────────────────────
Mental simulation                    Gazebo simulation
(imagine before act)                 (simulate before deploy)
       │                                    │
       ▼                                    ▼
Motor planning in                    Physics engine tests
safe "imagination"                   movements virtually
       │                                    │
       ▼                                    ▼
Predicted outcomes                   Sensor data streams
(what will I see/feel?)              (what would robot sense?)
       │                                    │
       ▼                                    ▼
Execute if safe                      Deploy if validated
```

## Key Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Physics Parameters | Default Gazebo values | Simpler for learners; tuning covered conceptually |
| Sensor Detail | Basic suite (LiDAR, depth, IMU) | Covers common sensors without overwhelming |
| Unity Visualization | Conceptual overview | Full implementation out of scope; show bridge |
| Chapter Sequencing | Concept-first | Aligns with constitution pedagogy pattern |
| Gazebo Version | Fortress/Harmonic (new Gazebo) | Clarified in /sp.clarify session |
| Teaching Analogy | Virtual rehearsal/imagination | Clarified in /sp.clarify session |

## Chapter Plan

### Chapter 1: Introduction to Digital Twins in Robotics

**Learning Objectives**:
- Define digital twin and its role in robotics development
- List 3+ benefits of simulation (safety, cost, speed, repeatability)
- Explain the sim-to-real gap challenge

**Sections**:
1. What is a Digital Twin? (~400 words)
   - Definition with virtual rehearsal analogy
   - Industry examples (automotive, aerospace, robotics)
2. Why Simulate Humanoid Robots? (~500 words)
   - Benefits: safety, iteration speed, cost savings
   - Real-world examples: Boston Dynamics, Tesla Optimus
3. The Sim-to-Real Challenge (~400 words)
   - Reality gap concept
   - Domain randomization introduction
   - Transfer learning basics
4. Simulation Tools Landscape (~300 words)
   - Gazebo: physics-focused
   - Unity: visualization-focused
   - When to use each
5. Summary (~200 words)

### Chapter 2: Physics Simulation with Gazebo

**Learning Objectives**:
- Set up Gazebo simulation environment with ROS 2
- Load and spawn URDF robots in Gazebo
- Configure physics properties and observe dynamics

**Sections**:
1. Introduction to Gazebo (~400 words)
   - Gazebo architecture (new Gazebo vs Classic)
   - ROS 2 integration via ros_gz_bridge
2. Creating Simulation Worlds (~600 words)
   - SDF format basics
   - Ground plane, lighting, objects
   - World file structure
3. Spawning Your Robot (~600 words)
   - Using spawn_entity service
   - URDF to SDF conversion
   - Launch file setup
4. Physics Properties and Dynamics (~500 words)
   - Gravity, friction, inertia
   - Collision detection
   - Physics engine options (ODE, DART, Bullet)
5. Controlling Simulated Joints (~500 words)
   - Joint position/velocity controllers
   - ros2_control integration
   - Observing behavior
6. Summary (~200 words)

### Chapter 3: Simulating Sensors for Humanoid Robots

**Learning Objectives**:
- Add simulated sensors to robot models
- Configure LiDAR, depth cameras, and IMUs
- Understand sensor noise models

**Sections**:
1. Why Simulate Sensors? (~400 words)
   - Perception pipeline testing
   - Data generation for ML
   - Virtual rehearsal for sensing
2. LiDAR Simulation (~600 words)
   - Gazebo GPU LiDAR plugin
   - Configuration parameters
   - Visualizing in RViz
3. Depth Camera Simulation (~600 words)
   - RGB-D camera plugin
   - Point cloud generation
   - Integration with perception
4. IMU Simulation (~500 words)
   - IMU sensor plugin
   - Orientation and acceleration data
   - Noise characteristics
5. Sensor Noise and Realism (~400 words)
   - Gaussian noise models
   - Why noise matters for real deployment
   - Domain randomization preview
6. Summary (~200 words)

### Chapter 4: High-Fidelity Visualization and Interaction with Unity

**Learning Objectives**:
- Explain Gazebo vs Unity use cases
- Understand ROS-TCP-Connector architecture
- Appreciate Unity's role in HRI research

**Sections**:
1. Gazebo vs Unity: When to Use Each (~400 words)
   - Gazebo: physics accuracy
   - Unity: visual fidelity, HRI
   - Complementary roles
2. Introduction to Unity for Robotics (~500 words)
   - Unity Robotics Hub
   - URDF Importer
   - Basic scene setup
3. Connecting Unity to ROS 2 (~600 words)
   - ROS-TCP-Connector architecture
   - Message types and serialization
   - Connection setup
4. Visualizing Robot State (~500 words)
   - Subscribing to /joint_states
   - TF visualization
   - Real-time updates
5. Human-Robot Interaction Scenarios (~400 words)
   - Why Unity for HRI
   - Virtual human models
   - Interaction testing
6. Summary (~200 words)
   - Module completion callout

## Validation Approach

### Content Quality
- [ ] All code examples syntactically valid
- [ ] Gazebo commands verified against official docs
- [ ] Unity concepts verified against Robotics Hub docs
- [ ] Virtual rehearsal analogy consistent across chapters

### Technical Accuracy
- [ ] New Gazebo (Fortress/Harmonic) terminology used correctly
- [ ] ROS 2 ros_gz_bridge usage accurate
- [ ] Sensor plugin configurations valid
- [ ] ROS-TCP-Connector description accurate

### Reproducibility
- [ ] World files can be loaded in Gazebo
- [ ] Launch files are complete and runnable
- [ ] Sensor configurations produce expected topics

## Phase Outputs

- **Phase 0**: research.md (Gazebo docs, Unity Robotics Hub, sensor plugins)
- **Phase 1**: content-model.md (section breakdown), quickstart.md (reader setup)
- **Phase 2**: tasks.md (via /sp.tasks command)

## Dependencies

- Module 1 completion (URDF, ROS 2 basics)
- Gazebo Fortress/Harmonic documentation
- Unity Robotics Hub documentation
- ros_gz_bridge documentation
