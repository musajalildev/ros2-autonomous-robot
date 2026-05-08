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