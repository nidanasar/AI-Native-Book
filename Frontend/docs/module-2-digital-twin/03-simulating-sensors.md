---
sidebar_position: 3
title: "Chapter 3: Simulating Sensors for Humanoid Robots"
description: "Add and configure virtual sensors including LiDAR, depth cameras, and IMUs"
---

# Simulating Sensors for Humanoid Robots

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Add simulated sensors to your robot model in Gazebo
- Configure LiDAR sensors and visualize laser scan data in RViz
- Set up depth cameras and view point cloud output
- Implement IMU sensors for orientation and acceleration data
- Understand sensor noise models and their importance for sim-to-real transfer
:::

## Why Simulate Sensors?

In Chapter 2, you gave your robot a body that obeys physics—it can move, fall, and collide. But a body without senses is useless. Your robot needs to *perceive* its environment to act intelligently. This is where **sensor simulation** completes the digital twin.

### The Perception-Action Loop

Every intelligent robot operates through a perception-action loop:

```
┌──────────────────────────────────────────────────────────┐
│                    THE ROBOT LOOP                         │
│                                                           │
│   Sense ──► Process ──► Decide ──► Act ──► (repeat)      │
│     │                                  │                  │
│     │         Environment              │                  │
│     └───────────◄──────────────────────┘                  │
└──────────────────────────────────────────────────────────┘
```

Without simulated sensors, you can only test the "Act" part of this loop. Sensor simulation lets you develop and test the entire perception pipeline:

- **Computer vision** algorithms processing camera data
- **SLAM** (Simultaneous Localization and Mapping) using LiDAR
- **State estimation** combining IMU with other sensors
- **Obstacle detection** for navigation and safety

### Virtual Rehearsal for Sensing

Continuing our virtual rehearsal analogy: when you imagine catching a ball, you don't just simulate your arm movement—you also imagine *seeing* the ball approach and *feeling* it hit your hand. Your mental rehearsal includes predicted sensory feedback.

Similarly, a complete digital twin needs simulated sensors:

| Human Mental Rehearsal | Robot Sensor Simulation |
|----------------------|-------------------------|
| Imagined sight | Simulated camera/LiDAR |
| Imagined proprioception | Simulated joint encoders |
| Imagined balance | Simulated IMU |
| Imagined touch | Simulated contact sensors |

### Benefits of Sensor Simulation

Simulated sensors provide unique advantages:

1. **Unlimited data generation**: Train ML models on millions of images without manual labeling
2. **Ground truth access**: Know the exact position of every object (impossible in reality)
3. **Controllable conditions**: Test perception in rain, fog, darkness—on demand
4. **Safe failure testing**: What happens if a sensor fails mid-operation?
5. **Rapid iteration**: Adjust sensor placement instantly without hardware changes

:::warning Reality Check
Simulated sensors produce "perfect" data with optional artificial noise. Real sensors have complex, correlated noise patterns that are hard to model. We'll discuss bridging this gap in the final section.
:::

## LiDAR Simulation

**LiDAR** (Light Detection and Ranging) measures distances by bouncing laser beams off surfaces. It's essential for humanoid robots navigating complex environments, providing 360° awareness that cameras can't match.

### How LiDAR Works

A LiDAR sensor:
1. Emits laser pulses in multiple directions
2. Measures the time for each pulse to return
3. Calculates distance: `distance = (time × speed_of_light) / 2`
4. Produces a point cloud or laser scan

For humanoid robots, 2D LiDAR (horizontal plane) is common for navigation, while 3D LiDAR provides richer environmental understanding.

### Adding LiDAR to Your Robot

In Gazebo, add a LiDAR sensor using the GPU-accelerated plugin:

```xml
<!-- Add inside a link in your URDF (Gazebo-specific section) -->
<gazebo reference="head_link">
  <sensor name="head_lidar" type="gpu_lidar">
    <pose>0 0 0.1 0 0 0</pose>  <!-- Position relative to head -->
    <topic>scan</topic>
    <update_rate>10</update_rate>

    <lidar>
      <scan>
        <horizontal>
          <samples>640</samples>
          <resolution>1</resolution>
          <min_angle>-1.5708</min_angle>   <!-- -90 degrees -->
          <max_angle>1.5708</max_angle>    <!-- +90 degrees -->
        </horizontal>
        <vertical>
          <samples>1</samples>   <!-- 2D LiDAR: single plane -->
          <resolution>1</resolution>
          <min_angle>0</min_angle>
          <max_angle>0</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.1</min>    <!-- Minimum detection range (meters) -->
        <max>30.0</max>   <!-- Maximum detection range (meters) -->
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>  <!-- 1cm standard deviation -->
      </noise>
    </lidar>

    <always_on>true</always_on>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

### LiDAR Configuration Parameters

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `samples` | Number of beams per scan | 360-1080 |
| `min_angle`/`max_angle` | Angular coverage | -π to +π for 360° |
| `min`/`max` range | Detection distance | 0.1m - 30m |
| `update_rate` | Scans per second | 10-40 Hz |
| `stddev` (noise) | Range measurement error | 0.01-0.05m |

### Bridging LiDAR Data to ROS 2

Configure ros_gz_bridge to publish LiDAR data:

```bash
ros2 run ros_gz_bridge parameter_bridge \
    /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
```

Or in a launch file:

```python
Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
)
```

### Visualizing in RViz

View LiDAR data in RViz:

```bash
ros2 run rviz2 rviz2
```

Then:
1. Set **Fixed Frame** to your LiDAR link (e.g., `head_link`)
2. Click **Add** → **By topic** → **/scan** → **LaserScan**
3. Adjust **Size** for point visibility

You should see red dots showing where the laser beams hit surfaces.

## Depth Camera Simulation

**Depth cameras** (RGB-D cameras) provide both color images and per-pixel depth measurements. Common real-world examples include Intel RealSense and Microsoft Kinect.

### How Depth Cameras Work

Depth cameras typically use:
- **Structured light**: Project known patterns, analyze distortion
- **Time-of-flight**: Measure light travel time per pixel
- **Stereo vision**: Triangulate depth from two cameras

In simulation, Gazebo computes depth directly from the 3D scene—no need to simulate the underlying technology.

### Adding a Depth Camera

```xml
<gazebo reference="head_link">
  <sensor name="rgbd_camera" type="rgbd_camera">
    <pose>0.05 0 0 0 0 0</pose>  <!-- 5cm forward on head -->
    <topic>camera</topic>
    <update_rate>30</update_rate>

    <camera>
      <horizontal_fov>1.047</horizontal_fov>  <!-- ~60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
      <depth_camera>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </depth_camera>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>

    <always_on>true</always_on>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

### Depth Camera Topics

A depth camera publishes multiple topics:

| Topic | Type | Content |
|-------|------|---------|
| `/camera/image` | `sensor_msgs/Image` | Color image |
| `/camera/depth` | `sensor_msgs/Image` | Depth image (float32) |
| `/camera/points` | `sensor_msgs/PointCloud2` | 3D point cloud |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Calibration data |

### Bridge Configuration

```python
Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/depth@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
    ],
)
```

### Viewing Point Clouds

In RViz:
1. Add **PointCloud2** display
2. Set topic to `/camera/points`
3. Set **Fixed Frame** to camera frame
4. Adjust **Size** and **Color Transformer**

Point clouds show the 3D structure of the scene as your robot's camera perceives it—essential for obstacle avoidance and manipulation.

## IMU Simulation

The **IMU** (Inertial Measurement Unit) measures acceleration and angular velocity—essential for balance control in humanoid robots.

### What an IMU Measures

An IMU typically contains:

| Component | Measures | Units |
|-----------|----------|-------|
| Accelerometer | Linear acceleration | m/s² |
| Gyroscope | Angular velocity | rad/s |
| Magnetometer | Magnetic field | (optional, for heading) |

For a humanoid robot, the IMU answers critical questions:
- Am I falling? (sudden acceleration change)
- How fast am I rotating? (gyroscope)
- Which way is up? (gravity direction)

### Adding an IMU Sensor

```xml
<gazebo reference="base_link">
  <sensor name="imu_sensor" type="imu">
    <pose>0 0 0 0 0 0</pose>
    <topic>imu</topic>
    <update_rate>200</update_rate>  <!-- High rate for balance control -->

    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>

    <always_on>true</always_on>
    <visualize>false</visualize>
  </sensor>
</gazebo>
```

### IMU Data in ROS 2

Bridge the IMU data:

```bash
ros2 run ros_gz_bridge parameter_bridge \
    /imu@sensor_msgs/msg/Imu[gz.msgs.IMU
```

The `sensor_msgs/msg/Imu` message contains:

```
orientation: quaternion (x, y, z, w)
angular_velocity: vector (x, y, z) in rad/s
linear_acceleration: vector (x, y, z) in m/s²
covariance matrices for each
```

### Using IMU for Balance

In a humanoid controller, IMU data feeds into the balance system:

```python
def balance_callback(self, imu_msg):
    # Extract orientation (quaternion -> euler)
    orientation = imu_msg.orientation
    roll, pitch, yaw = quaternion_to_euler(orientation)

    # Check if robot is tilting too much
    if abs(pitch) > 0.1:  # ~6 degrees
        self.get_logger().warn(f'Robot tilting! Pitch: {pitch:.2f} rad')
        self.apply_correction(pitch)

    # Use angular velocity for rate feedback
    pitch_rate = imu_msg.angular_velocity.y
    # ... feed into PD controller for balance
```

## Sensor Noise and Realism

Perfect sensors don't exist. Real sensors have noise, drift, and systematic errors. To prepare your algorithms for reality, you must add realistic noise to simulated sensors.

### Types of Sensor Noise

| Noise Type | Description | Effect |
|------------|-------------|--------|
| Gaussian | Random variations around true value | Most common, easy to model |
| Bias | Constant offset from true value | Causes drift over time |
| Scale factor | Multiplicative error | Distances consistently over/under-estimated |
| Quantization | Discrete resolution steps | Visible in low-resolution sensors |
| Temporal | Changes over time (drift) | IMU orientation error accumulates |

### Configuring Noise in Gazebo

Gazebo supports Gaussian noise on most sensors:

```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>      <!-- Bias: usually 0 for unbiased sensors -->
  <stddev>0.01</stddev> <!-- Standard deviation of noise -->
</noise>
```

Typical noise values:

| Sensor | Parameter | Typical Noise |
|--------|-----------|---------------|
| LiDAR | Range | 0.01 - 0.03 m |
| Camera | Pixel intensity | 0.005 - 0.02 |
| IMU (gyro) | Angular velocity | 0.0001 - 0.001 rad/s |
| IMU (accel) | Linear acceleration | 0.01 - 0.05 m/s² |

### Domain Randomization for Sensors

Beyond fixed noise, domain randomization varies sensor parameters during training:

```python
# Pseudocode for sensor domain randomization
for each training episode:
    lidar_noise_stddev = random.uniform(0.005, 0.05)
    camera_brightness = random.uniform(0.8, 1.2)
    imu_bias = random.uniform(-0.01, 0.01)

    # Configure simulation with randomized sensor parameters
    update_sensor_config(lidar_noise=lidar_noise_stddev, ...)

    # Train with this configuration
    run_episode()
```

This teaches your perception algorithms to handle sensor variations they'll encounter in the real world.

### The Sim-to-Real Sensor Gap

Simulated sensors differ from real ones in several ways:

| Aspect | Simulation | Reality |
|--------|------------|---------|
| Noise distribution | Perfect Gaussian | Complex, correlated |
| Environmental effects | Often ignored | Reflections, glare, fog |
| Sensor artifacts | Clean data | Motion blur, rolling shutter |
| Calibration | Perfect | Requires careful procedure |

Strategies to bridge this gap:
1. **Add realistic noise models** based on sensor datasheets
2. **Use domain randomization** to build robustness
3. **Fine-tune on real data** after simulation training
4. **Test with sensor-realistic synthetic data** (photorealistic renderers)

:::tip Key Insight
The goal isn't to perfectly match reality in simulation—that's impossible. The goal is to build algorithms robust enough to handle the *range* of conditions they might encounter, including real-world sensor imperfections.
:::

## Summary

In this chapter, you learned how to add simulated sensors to your digital twin:

**Key Concepts:**

- **Sensor simulation completes the perception-action loop**
  - Without sensors, you can only test actuation
  - With sensors, you can develop full perception pipelines

- **LiDAR sensors** provide distance measurements
  - Configure with samples, angle range, noise
  - Visualize as LaserScan in RViz
  - Essential for navigation and obstacle avoidance

- **Depth cameras** provide RGB + depth data
  - Output: images, depth maps, point clouds
  - Configure resolution, field of view, noise
  - Essential for manipulation and scene understanding

- **IMU sensors** measure acceleration and rotation
  - Critical for humanoid balance control
  - High update rate (100-400 Hz typical)
  - Prone to drift—often fused with other sensors

- **Sensor noise** bridges simulation to reality
  - Add Gaussian noise to all sensors
  - Use domain randomization for robustness
  - Real sensors have complex noise patterns

**Virtual Rehearsal Extended:**

| Human Sense | Robot Sensor | Simulation |
|-------------|--------------|------------|
| Vision | Camera/LiDAR | Image/point cloud plugins |
| Balance | IMU | Acceleration/gyro data |
| Proprioception | Joint encoders | Joint state publisher |
| Touch | Force sensors | Contact plugins |

**What's Next:**

Your digital twin can now move *and* perceive. But Gazebo prioritizes physics accuracy over visual realism. In Chapter 4, we'll explore Unity—a game engine that provides photorealistic rendering for training vision systems and creating human-robot interaction scenarios.

---

:::tip Next Chapter
Continue to [Chapter 4: High-Fidelity Visualization and Interaction with Unity](./unity-visualization-hri) to learn about advanced visualization and human-robot interaction.
:::
