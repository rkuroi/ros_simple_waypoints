# ros_ros_simple_waypoints

This is a work in progress to manipulate sequences of waypoints for use with the [ROS navigation stack](http://wiki.ros.org/navigation?distro=melodic),
(namely move_base).

As of now it contains a single node which can be run using

```bash
rosrun ros_simple_waypoints waypoints_callback_client
```

Topics can be renamed using the ROS remapping tool.

## Subscribed topics

- `/simple_waypoints_definition` : topic on which PoseStamped messages are published by another node (i.e. rviz). When a message is received, the waypoint defined in it is added at the end of the current waypoints list.

## Published topics

- `/current_waypoints_array` : topic on which a PoseArray message representing the current waypoints list is published when requested using the `publish_waypoints_sequence` service

## Services

- `cancel_current_waypoint` : **one arg: *skip***, *cancels the current goal and either tries to send the next (*skip=true*) or cancels the whole sequence*

- `clear_waypoints_sequence` : **no args**, erases the current list of waypoints

- `load_waypoints_sequence` : **one arg: *filepath***, loads a sequence of waypoints from a YAML file designated by *filepath*

- `save_waypoints_sequence` : **one arg: *filepath***, saves the current sequence of waypoints to a YAML file at the location designated by *filepath*

- `start_waypoints_sequence` : **no args**, sends the first goal to the *move_base* server and automatically continues until the end of the sequence

- `publish_waypoints_sequence` : **no args**, publishes the current sequence of waypoints as a PoseArray on the topic `/current_waypoints_array` (for visualization on rviz)

### TODO

- implement a way to modify the waypoints without having to clear and begin anew

- rviz plugin (?) or a separate GUI(?)
