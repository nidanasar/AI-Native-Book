/*
 * ROS Connection Example for Unity
 *
 * This script demonstrates how to connect Unity to ROS 2 using
 * the ROS-TCP-Connector package. It subscribes to /joint_states
 * and updates a robot visualization in real-time.
 *
 * Prerequisites:
 *   1. Unity 2021+ with ROS-TCP-Connector package installed
 *   2. Robot imported via URDF Importer
 *   3. ROS 2 with ros_tcp_endpoint running:
 *      ros2 run ros_tcp_endpoint default_server_endpoint
 *
 * Usage:
 *   1. Attach this script to a GameObject in your Unity scene
 *   2. Assign the robot root Transform to the 'robotRoot' field
 *   3. Configure ROS IP in ROSConnection settings (default: 127.0.0.1:10000)
 *   4. Run ROS 2 simulation and press Play in Unity
 *
 * This is a CONCEPTUAL example - actual implementation requires
 * the Unity ROS-TCP-Connector package to compile.
 */

using System.Collections.Generic;
using UnityEngine;

// NOTE: These namespaces require the ROS-TCP-Connector package
// Uncomment when package is installed:
// using Unity.Robotics.ROSTCPConnector;
// using RosMessageTypes.Sensor;
// using RosMessageTypes.Geometry;

namespace HumanoidTextbook.Module2.Unity
{
    /// <summary>
    /// Subscribes to ROS 2 joint states and updates robot visualization.
    /// Demonstrates basic ROS-Unity integration for humanoid robots.
    /// </summary>
    public class RosConnectionExample : MonoBehaviour
    {
        [Header("Robot Configuration")]
        [Tooltip("Root transform of the imported URDF robot")]
        public Transform robotRoot;

        [Header("ROS Configuration")]
        [Tooltip("Topic name for joint states")]
        public string jointStatesTopic = "/joint_states";

        [Tooltip("Topic name for robot pose")]
        public string poseTopic = "/robot_pose";

        [Header("Visualization Settings")]
        [Tooltip("Smoothing factor for joint interpolation (higher = faster)")]
        [Range(1f, 20f)]
        public float smoothingFactor = 10f;

        [Tooltip("Show debug information in console")]
        public bool debugMode = false;

        // Maps joint names to their Unity ArticulationBody components
        private Dictionary<string, ArticulationBody> jointMap;

        // Target positions for smooth interpolation
        private Dictionary<string, float> targetPositions;

        // Connection status
        private bool isConnected = false;

        void Start()
        {
            InitializeJointMap();
            SetupROSConnection();
        }

        /// <summary>
        /// Builds a dictionary mapping joint names to ArticulationBody components.
        /// This allows quick lookup when processing joint state messages.
        /// </summary>
        void InitializeJointMap()
        {
            jointMap = new Dictionary<string, ArticulationBody>();
            targetPositions = new Dictionary<string, float>();

            if (robotRoot == null)
            {
                Debug.LogError("RosConnectionExample: robotRoot not assigned!");
                return;
            }

            // Find all ArticulationBody components in the robot hierarchy
            var articulations = robotRoot.GetComponentsInChildren<ArticulationBody>();

            foreach (var articulation in articulations)
            {
                // Skip the root body (usually fixed)
                if (articulation.jointType == ArticulationJointType.FixedJoint)
                    continue;

                string jointName = articulation.name;
                jointMap[jointName] = articulation;
                targetPositions[jointName] = 0f;

                if (debugMode)
                {
                    Debug.Log($"Mapped joint: {jointName} ({articulation.jointType})");
                }
            }

            Debug.Log($"RosConnectionExample: Initialized {jointMap.Count} joints");
        }

        /// <summary>
        /// Sets up the ROS-TCP connection and subscribes to topics.
        /// </summary>
        void SetupROSConnection()
        {
            // NOTE: Actual ROS connection code requires ROS-TCP-Connector package
            // The following is pseudocode showing the pattern:

            /*
            // Get or create the ROS connection singleton
            var ros = ROSConnection.GetOrCreateInstance();

            // Subscribe to joint states topic
            ros.Subscribe<JointStateMsg>(jointStatesTopic, OnJointStateReceived);

            // Optionally subscribe to pose for base link position
            ros.Subscribe<PoseMsg>(poseTopic, OnPoseReceived);

            // Register for connection status events
            ros.RegisterRosService<EmptySrvRequest, EmptySrvResponse>(
                "/ping",
                OnPingResponse
            );
            */

            Debug.Log($"RosConnectionExample: Subscribed to {jointStatesTopic}");
            isConnected = true;
        }

        /// <summary>
        /// Called when a JointState message is received from ROS 2.
        /// Updates target positions for all joints.
        /// </summary>
        /// <param name="msg">The joint state message from ROS 2</param>
        void OnJointStateReceived(/* JointStateMsg msg */)
        {
            // NOTE: Actual implementation with ROS-TCP-Connector:
            /*
            for (int i = 0; i < msg.name.Length; i++)
            {
                string jointName = msg.name[i];
                float position = (float)msg.position[i];

                if (targetPositions.ContainsKey(jointName))
                {
                    targetPositions[jointName] = position;

                    if (debugMode)
                    {
                        Debug.Log($"Joint {jointName}: {position:F3} rad");
                    }
                }
            }
            */
        }

        /// <summary>
        /// Called when a Pose message is received for the robot base.
        /// Updates the robot's world position and orientation.
        /// </summary>
        void OnPoseReceived(/* PoseMsg msg */)
        {
            // NOTE: Actual implementation with ROS-TCP-Connector:
            /*
            // Convert ROS pose to Unity coordinates
            // ROS: X-forward, Y-left, Z-up (right-handed)
            // Unity: X-right, Y-up, Z-forward (left-handed)

            Vector3 position = new Vector3(
                -(float)msg.position.y,  // ROS Y -> Unity -X
                (float)msg.position.z,   // ROS Z -> Unity Y
                (float)msg.position.x    // ROS X -> Unity Z
            );

            Quaternion rotation = new Quaternion(
                -(float)msg.orientation.y,
                (float)msg.orientation.z,
                (float)msg.orientation.x,
                (float)msg.orientation.w
            );

            robotRoot.position = position;
            robotRoot.rotation = rotation;
            */
        }

        void Update()
        {
            if (!isConnected || jointMap == null)
                return;

            // Smoothly interpolate joint positions
            UpdateJointVisualization();
        }

        /// <summary>
        /// Smoothly updates joint positions using linear interpolation.
        /// This prevents jerky motion when ROS messages arrive at lower rates.
        /// </summary>
        void UpdateJointVisualization()
        {
            foreach (var kvp in jointMap)
            {
                string jointName = kvp.Key;
                ArticulationBody joint = kvp.Value;

                if (!targetPositions.ContainsKey(jointName))
                    continue;

                float targetRad = targetPositions[jointName];
                float targetDeg = targetRad * Mathf.Rad2Deg;

                // Get current drive settings
                var drive = joint.xDrive;

                // Interpolate toward target
                float currentTarget = drive.target;
                float newTarget = Mathf.Lerp(
                    currentTarget,
                    targetDeg,
                    Time.deltaTime * smoothingFactor
                );

                // Apply new target
                drive.target = newTarget;
                joint.xDrive = drive;
            }
        }

        /// <summary>
        /// Immediately sets all joints to their target positions without interpolation.
        /// Useful for initialization or teleportation.
        /// </summary>
        public void SnapToTargetPositions()
        {
            foreach (var kvp in jointMap)
            {
                string jointName = kvp.Key;
                ArticulationBody joint = kvp.Value;

                if (!targetPositions.ContainsKey(jointName))
                    continue;

                float targetRad = targetPositions[jointName];
                float targetDeg = targetRad * Mathf.Rad2Deg;

                var drive = joint.xDrive;
                drive.target = targetDeg;
                joint.xDrive = drive;
            }
        }

        /// <summary>
        /// Returns the current position of a joint in radians.
        /// </summary>
        public float GetJointPosition(string jointName)
        {
            if (jointMap.TryGetValue(jointName, out var joint))
            {
                return joint.jointPosition[0]; // Returns in radians
            }
            return 0f;
        }

        /// <summary>
        /// Returns all joint names managed by this component.
        /// </summary>
        public IEnumerable<string> GetJointNames()
        {
            return jointMap.Keys;
        }

        void OnGUI()
        {
            if (!debugMode)
                return;

            // Display connection status and joint info
            GUILayout.BeginArea(new Rect(10, 10, 300, 500));
            GUILayout.Label($"ROS Connection: {(isConnected ? "Connected" : "Disconnected")}");
            GUILayout.Label($"Joints: {jointMap.Count}");
            GUILayout.Space(10);

            foreach (var kvp in targetPositions)
            {
                GUILayout.Label($"{kvp.Key}: {kvp.Value:F3} rad");
            }

            GUILayout.EndArea();
        }

        void OnDestroy()
        {
            // Cleanup ROS connection
            /*
            var ros = ROSConnection.GetOrCreateInstance();
            ros.Unsubscribe(jointStatesTopic);
            ros.Unsubscribe(poseTopic);
            */
        }
    }

    /// <summary>
    /// Example: Publishing commands from Unity to ROS 2.
    /// Demonstrates bidirectional communication.
    /// </summary>
    public class RosCommandPublisher : MonoBehaviour
    {
        [Header("Publishing Configuration")]
        public string commandTopic = "/unity_command";
        public float publishRate = 10f; // Hz

        private float lastPublishTime = 0f;

        void Update()
        {
            // Publish at fixed rate
            if (Time.time - lastPublishTime > 1f / publishRate)
            {
                PublishCommand();
                lastPublishTime = Time.time;
            }
        }

        void PublishCommand()
        {
            // NOTE: Actual implementation with ROS-TCP-Connector:
            /*
            var msg = new Float64Msg
            {
                data = ComputeCommandValue()
            };

            ROSConnection.GetOrCreateInstance().Publish(commandTopic, msg);
            */
        }

        float ComputeCommandValue()
        {
            // Example: Send a sinusoidal command
            return Mathf.Sin(Time.time * 2f * Mathf.PI * 0.5f);
        }
    }
}
