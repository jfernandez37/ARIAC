# moveit_py

## Overview

Moveit_py allows the robots in the ARIAC simulation to be controlled using moveit programmed in Python. In this example, the package is called "moveit_py_test".

## Basic setup

To start, we must first get the floor and ceiling robots as planning components. To do this, we can use this code:

```python
robot = MoveItPy(node_name="moveit_py_test")

floor_robot = robot.get_planning_component("floor_robot")
ceiling_robot = robot.get_planning_component("ceiling_robot")
```

This will allow us to set the start and goal states for movements and conduct those movements.

## Moving to named positions

Moving to named posiitons allow us to always move to the same positions at the start of the simulation. To do this, we can use the planning components made in the first section to set the start state and set the goal state