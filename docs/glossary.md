---
sidebar_label: 'Glossary'
sidebar_position: 100
---

# Glossary

This glossary defines key terms used throughout the AI Robotics Textbook, organized by topic area.

## General Robotics Terms

**Actuator**: A component of a robot that converts control signals into physical motion or action.

**Degrees of Freedom (DOF)**: The number of independent parameters that define the configuration or state of a mechanical system.

**End Effector**: The device at the end of a robotic arm that interacts with the environment, such as a gripper or tool.

**Forward Kinematics**: The process of determining the position and orientation of the end effector given the joint angles.

**Inverse Kinematics**: The process of determining the joint angles required to achieve a desired end effector position and orientation.

**Jacobian**: A matrix that describes the relationship between joint velocities and end-effector velocities.

**Manipulator**: A robotic device used to position and orient objects in a workspace.

**Mobile Robot**: A robot with the ability to move around in its environment using wheels, tracks, legs, or other locomotion methods.

**Robot Operating System (ROS)**: A flexible framework for writing robot software that provides services designed for a heterogeneous computer cluster.

**Servo**: A rotary actuator that allows for precise control of angular position, velocity, and acceleration.

## AI and Machine Learning Terms

**Artificial Intelligence (AI)**: The simulation of human intelligence processes by machines, especially computer systems.

**Convolutional Neural Network (CNN)**: A class of deep neural networks, most commonly applied to analyzing visual imagery.

**Deep Learning**: A subset of machine learning that uses neural networks with multiple layers to extract high-level features from raw input.

**Large Language Model (LLM)**: An AI model based on transformer architecture with billions of parameters, trained on large text corpora.

**Machine Learning**: A type of artificial intelligence that provides systems the ability to automatically learn and improve from experience.

**Neural Network**: A series of algorithms that endeavors to recognize underlying relationships in a set of data through a process that mimics how the human brain operates.

**Reinforcement Learning**: A type of machine learning where an agent learns to make decisions by taking actions in an environment to maximize cumulative reward.

**Supervised Learning**: A type of machine learning where the model is trained on labeled data.

**Transformer**: A deep learning model that adopts the mechanism of attention, differentially weighing the significance of each part of the input data.

**Unsupervised Learning**: A type of machine learning where the model is trained on unlabeled data to find patterns or structures.

## Computer Vision Terms

**Bounding Box**: A rectangle used to identify and locate an object in an image.

**Depth Map**: A type of image that contains information relating to the distance of objects in a scene from a viewpoint.

**Image Segmentation**: The process of partitioning a digital image into multiple segments to simplify and change the representation.

**Object Detection**: The computer technology related to identifying objects in images or videos.

**Optical Flow**: The pattern of apparent motion of objects, surfaces, and edges in a visual scene caused by the relative motion.

**Point Cloud**: A set of data points in space that represent the external surface of an object or environment.

**Semantic Segmentation**: The task of associating each pixel in an image with a class label.

**Stereo Vision**: A technique that extracts 3D information from digital images using two or more cameras.

**Visual SLAM**: Simultaneous Localization and Mapping using visual sensors as the primary input.

## Navigation and Mapping Terms

**A* Algorithm**: A graph traversal and path search algorithm that is often used in many fields of computer science.

**Costmap**: A 2D or 3D representation of the environment that indicates the cost of navigating through each cell.

**Dijkstra's Algorithm**: An algorithm for finding the shortest paths between nodes in a graph.

**Global Planner**: A component of a navigation system that creates a path from the start location to the goal location.

**Grid Map**: A discretized representation of the environment where each cell represents the occupancy status.

**Local Planner**: A component of a navigation system that follows a path while avoiding obstacles in real-time.

**Occupancy Grid**: A probabilistic 2D representation of space which divides the world into discrete grid cells.

**Path Planning**: The computational problem of finding a sequence of valid configurations that moves an object.

**Potential Field**: A method for path planning that models the environment as an artificial force field.

**Simultaneous Localization and Mapping (SLAM)**: The computational problem of constructing or updating a map of an unknown environment.

## Control Systems Terms

**Controller**: A device or algorithm that manages and regulates the behavior of other devices or systems.

**Feedback Control**: A control system that uses the difference between the desired and actual output to adjust the system.

**PID Controller**: A control loop mechanism employing feedback that is widely used in industrial control systems.

**Proportional-Integral-Derivative (PID)**: A control algorithm that calculates an error value as the difference between desired and actual output.

**State Space**: A mathematical representation of a physical system as a set of input, output and state variables.

**Trajectory**: A time-ordered sequence of desired position, velocity, and acceleration commands.

**Velocity Controller**: A controller that adjusts the robot's velocity to follow a desired path.

## ROS 2 Specific Terms

**Action**: A communication pattern in ROS 2 that enables sending goals to a server and receiving results and feedback.

**Launch File**: An XML or Python file that defines how to start a set of nodes with specific configurations.

**Message**: A data structure used for communication between ROS nodes.

**Node**: A process that performs computation in ROS.

**Package**: A reusable software module in ROS that contains nodes, libraries, and other resources.

**Parameter**: A way to configure a node at runtime in ROS.

**Publisher**: A node that sends messages on a specific topic.

**Service**: A synchronous communication pattern in ROS where a client sends a request and receives a response.

**Subscriber**: A node that receives messages on a specific topic.

**Topic**: A communication channel over which messages are sent and received in ROS.

**URDF (Unified Robot Description Format)**: An XML format for representing a robot model in ROS.

## Vision-Language-Action Terms

**Cognitive Planning**: High-level reasoning that determines appropriate sequences of actions to achieve goals.

**Domain Randomization**: A technique that randomizes simulation parameters to improve sim-to-real transfer.

**Multi-Modal Integration**: The process of combining information from different sensory modalities.

**Natural Language Processing (NLP)**: A field of AI focused on the interaction between computers and humans through natural language.

**Sim-to-Real Transfer**: The process of transferring skills or behaviors learned in simulation to real robots.

**Vision-Language-Action (VLA)**: A system that integrates visual perception, language understanding, and robotic action.

**Voice Activity Detection (VAD)**: A technique to detect the presence or absence of human speech in an audio signal.

**Whisper**: OpenAI's automatic speech recognition system.

## Ethics and Safety Terms

**Autonomous System**: A system that can operate independently without human intervention.

**Human-Robot Interaction (HRI)**: The field of study focusing on the design, development, and evaluation of robots for human interaction.

**Responsible AI**: A framework for the ethical development and deployment of artificial intelligence systems.

**Safety-Critical System**: A system whose failure could result in human injury or death.

**Trustworthy AI**: AI systems that are lawful, ethical, and robust.

## Simulation Terms

**Digital Twin**: A virtual representation of a physical object or system that spans its lifecycle.

**Gazebo**: An open-source 3D robotics simulator.

**Isaac Sim**: NVIDIA's reference application for simulating autonomous robots.

**PhysX**: NVIDIA's physics engine used for real-time simulation.

**Synthetic Data**: Artificially generated data that mimics real-world observations.

**Unity Robotics**: The integration of Unity game engine with robotics simulation.

## Hardware Terms

**LiDAR**: Light Detection and Ranging - a remote sensing method that uses light in the form of a pulsed laser.

**IMU (Inertial Measurement Unit)**: An electronic device that measures and reports a body's specific force, angular rate, and sometimes magnetic field.

**RGB-D Camera**: A camera that captures both color (RGB) and depth (D) information.

**ROS Bridge**: A component that enables communication between ROS and other systems like Unity or simulation engines.

**Time-of-Flight (ToF)**: A method for measuring the distance between a sensor and an object based on the time light takes to travel.