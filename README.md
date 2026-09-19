# ROS2 Autonomous Mobile Robot

An autonomous mobile robotics system developed using **ROS2, Python and OpenCV**, capable of navigating an environment, exploring unknown areas and detecting coloured visual beacons using computer vision.

The project was developed as part of a team robotics project at the **University of Sheffield** and was deployed and tested on a physical mobile robot.

## Demo

### Autonomous Exploration & Beacon Detection

The robot autonomously explores an environment containing obstacles while searching for coloured beacons.

▶️ [Watch the Task 3 autonomous robot demonstration](demo/task3-demo.mp4)

### Robot Motion

The initial task involved controlling the physical robot's movement while monitoring its position and orientation through ROS2.

▶️ [Watch the Task 1 robot demonstration](demo/task1-demo.mp4)

## Features

* Autonomous mobile robot navigation
* Frontier-based environment exploration
* Obstacle-aware movement
* Computer-vision-based beacon detection
* HSV colour segmentation
* Detection of red, green, blue and yellow beacons
* ROS2 node-based architecture
* Robot position and orientation tracking
* Environment mapping
* Deployment and testing on physical robotics hardware

## System Overview

The system combines autonomous navigation with computer vision to allow the robot to explore its environment and identify visual targets.

ROS2 nodes are used to separate different responsibilities within the system.

The exploration component determines where the robot should navigate next, while the vision component processes camera data to identify coloured beacons.

### Exploration

The robot explores previously unknown areas of the map using frontier-based exploration.

Frontiers represent boundaries between explored and unexplored regions. By identifying and navigating towards these areas, the robot can progressively explore the environment without requiring a predefined route.

### Beacon Detection

Camera images are processed using **OpenCV** and **CvBridge**.

Images received through ROS2 are converted into a format that can be processed with OpenCV. HSV colour segmentation is then used to isolate beacon colours.

The system detects:

* Red
* Green
* Blue
* Yellow

Additional filtering is used to reduce false detections, including minimum contour-area requirements and checks around the edges of the camera frame.

## Project Structure

```text
ros2-autonomous-robot/
│
├── com2009_team09_2026_modules/
├── launch/
├── maps/
├── msg/
│
├── scripts/
│   ├── task1/
│   │   └── task1.py
│   │
│   ├── task2/
│   │
│   └── task3/
│       ├── task3_beacon.py
│       ├── task3_explorer.py
│       └── task3_map_saver.py
│
├── snaps/
├── demo/
│   ├── task1-demo.mp4
│   └── task3-demo.mp4
│
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Task 1 – Robot Motion

The first stage focused on controlling the movement of the physical robot using ROS2.

The robot's position and orientation were monitored during execution, including its:

* X position
* Y position
* Yaw

This provided experience working with ROS2 topics, robot movement and coordinate-based control on physical hardware.

## Task 3 – Autonomous Exploration

The final stage combined several components into a larger autonomous robotics system.

The robot was required to navigate through an environment containing obstacles while exploring unknown areas and searching for coloured beacons.

The system combines:

```text
Camera Input
     │
     ▼
ROS2 Image Topic
     │
     ▼
CvBridge
     │
     ▼
OpenCV Image Processing
     │
     ▼
HSV Colour Segmentation
     │
     ▼
Beacon Detection
```

Alongside the vision pipeline, the exploration system continuously searches for unexplored areas of the environment and determines new navigation targets.

## Technologies

**Languages**

* Python

**Robotics**

* ROS2
* ROS2 Nodes
* ROS2 Topics
* Robot navigation and mapping

**Computer Vision**

* OpenCV
* CvBridge
* HSV colour segmentation
* Contour detection

**Other**

* NumPy
* Git
* GitHub
* Linux development environment

## My Contribution

This project was developed collaboratively as part of a **six-person university team**.

My work focused particularly on the autonomous exploration and beacon-detection components of the project.

This included working with:

* ROS2 Python nodes
* Camera image processing
* CvBridge
* OpenCV
* HSV colour thresholding
* Beacon detection logic
* Contour filtering
* Autonomous exploration behaviour
* Integration and testing of the robot's autonomous behaviour

Working on the project also involved debugging and testing the system on physical robotics hardware and collaborating with other team members through Git.

## What I Learned

This project provided practical experience building software that interacts with physical hardware rather than running entirely within a conventional application environment.

Key areas of experience included:

* Designing software using ROS2 nodes
* Processing real-time camera data
* Applying computer vision techniques with OpenCV
* Developing autonomous robot behaviour
* Debugging software on physical hardware
* Integrating independently developed components
* Using Git collaboratively within a development team

## Contributors

This project was completed collaboratively by a six-person team as part of the University of Sheffield COM2009 robotics coursework.

The original Git commit history and contributor information have been preserved in this repository.


## Build the code
```
colcon build --packages-select com2009_team09_2026
```

## Resource `bashrc`
```
source ~/.bashrc
```

## Run the code with launch file
```
ros2 launch com2009_team09_2026 taskX.launch.py
```

## Drive the robot with Keyboard
```
ros2 run turtlebot3_teleop teleop_keyboard
```

> [!NOTE]
> Run all these commands from `~/ros2_ws`

## Gazebo Simulation Environments

### Empty World
```
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### Task1: Velocity Control
```
ros2 launch tuos_task_sims fig_of_eight.launch.py
```

### Task2: Avoiding Obstacles
```
ros2 launch tuos_task_sims obstacle_avoidance.launch.py
```

### Task3: Exploration & Search
```
ros2 launch tuos_task_sims explore.launch.py
```