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

> [!NOTE]
> Run all these commands from `~/ros2_ws`

## Gazebo Simulation Environments

### Empty World
```
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### Task1: Figure of Eight
```
ros2 launch tuos_task_sims fig_of_eight.launch.py
```

### Task2: Obstacle Avoidance
```
ros2 launch tuos_task_sims obstacle_avoidance.launch.py
```