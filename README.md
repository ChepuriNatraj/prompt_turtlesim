# Prompt Turtlesim

Controlling the Turtlesim in ROS2 using simple prompts to control the robot.

## Description

This ROS2 package allows you to control the turtlesim robot using simple text prompts. You can move the turtle forward, backward, turn left or right, draw circles, and stop.

## Installation

Clone this repository into your ROS2 workspace's `src` directory:

```bash
cd ~/ros2_ws/src
git clone <repository-url> prompt_turtlesim
```

Then build the workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select prompt_turtlesim
source install/setup.bash
```

## Usage

First, start the turtlesim node:

```bash
ros2 run turtlesim turtlesim_node
```

Then, run the prompt turtle node:

```bash
ros2 run prompt_turtlesim prompt_turtle
```

Enter commands like:
- `forward 2` - Move forward 2 units
- `backward 1.5` - Move backward 1.5 units
- `left 90` - Turn left 90 degrees
- `right 45` - Turn right 45 degrees
- `circle` - Draw a circle
- `stop` - Stop the turtle
- `quit` - Exit the program

## Dependencies

- rclpy
- geometry_msgs

## License

TODO: License declaration# prompt_turtlesim
