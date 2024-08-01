# IR Final Project
Intelligent Robotics Team 22 Final Project

## Installation

- `git clone` the
  [`rl-texplore-ros-pkg`](https://github.com/toddhester/rl-texplore-ros-pkg)
  into `<catkin_ws>/src`
- `git clone` this repository into `<catkin_ws>/src`
- remove the `static` on line 1129 of
  `rl-texplore-ros-pkg/src/rl_common/include/rl_common/Random.h`
  	- source: [this GitHub issue](https://github.com/toddhester/rl-texplore-ros-pkg/issues/7)
	- this removes the errors during `catkin_make`
- run `catkin_make` to compile the packages

## Running

```bash
roslaunch stealth_robot [map_name]_[difficulty].launch
```

Current maps:
- `lgfloor`: lower ground floor map from Assignment 1
- `simpler-maze`: a simple maze drawn by hand and converted to Stage map
- `floorplan1`: floor plan map generated with map creation script
- `map1`: map generated with map creation script

`difficulty` is used to determine which goal point to use; options are `easy`,
`medium`, and `hard`.

Searchlights (green spheres) should appear and reinforcement learning should
start right away- they are initialised on a per-map basis along with AMCL
initial poses, and reinforcement learning goal points.

## Builtins (for learning purposes)

Instructions on how to run the built-in agents and environments from
`rl-texplore-ros-pkg`

- Start `roscore`:
	```
	roscore &
	```

- Run the Q-learning agent:
	```
	rosrun rl_agent agent --agent qlearner
	```

- Running an `rl_env` environment (here, the two-room grid world, see
  [the documentation](http://wiki.ros.org/rl_env) for all environment types)
	```bash
	rosrun rl_env env --env tworooms
	```
	- run in a new terminal
	- **important:** make sure you start running the `rl_agent` before the
	  `rl_env`!

## Useful Resources
- [ROS reinforcement learning
  tutorial](http://wiki.ros.org/reinforcement_learning/Tutorials/Reinforcement%20Learning%20Tutorial)
- [ROS reinforcement learning
  documentation](http://wiki.ros.org/reinforcement_learning)
- [ROS Stage
  customisation](http://wiki.ros.org/turtlebot_stage/Tutorials/indigo/Customizing%20the%20Stage%20Simulator)
  (useful for defining new maps)
- [ROS actionlib documentation](http://wiki.ros.org/actionlib) (useful for
  defining robot actions)
- [ROS Publisher and Subscriber
  Tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
