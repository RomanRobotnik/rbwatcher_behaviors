# rbwatcher_behaviors

`rbwatcher_behaviors` provides a reusable BehaviorTree.CPP-based action server that orchestrates RBWatcher missions through Nav2 navigation commands, boolean topic checks, and auxiliary ROS interfaces. The package exposes a ROS 2 action (`ExecuteBehaviorTree`) that accepts either inline XML or a path to a behavior tree definition and executes it using custom tree nodes.

## Requirements

- ROS 2 Jazzy (or compatible distro providing BehaviorTree.CPP v4.7.1)
- `behaviortree_cpp`, `nav2_msgs`, `geometry_msgs`, `std_msgs`, `std_srvs`
- `colcon` and the usual ROS 2 build dependencies

## Build

From a ROS 2 workspace (e.g., `workshop_ws`):

```bash
cd ~/workspaces/workshop_ws
colcon build --packages-select rbwatcher_behaviors
source install/setup.bash
```

## Features

- **BehaviorTree.CPP integration** with ready-to-use custom nodes.
- **Action server** (`execute_behavior_tree`) that receives targets and tree definitions at runtime.
- **Nav2 bridge** through the `NavigateToPose` and `FollowWaypoints` behavior tree action nodes.
- **Topic guard** using `BoolTopicCondition` to react to boolean status topics.
- **Service trigger** node to interact with `std_srvs/srv/Trigger` servers.
- **PTZ tracker bridge** with `PTZTrackerAction` to start the `ptz_tracker` action server and capture the resulting camera pose.
- **Launch file** for quick bring-up with the provided example tree.

## Package Content

- `action/ExecuteBehaviorTree.action`: ROS 2 action used by the server.
- `config/default_tree.xml`: Example behavior tree wiring the provided nodes.
- `launch/behavior_tree_action_server.launch.py`: Launch file that starts the action server.
- `src/` and `include/`: Implementation of the action server and custom tree nodes.

## Usage

```bash
ros2 launch rbwatcher_behaviors behavior_tree_action_server.launch.py
```

Send an action goal using `ros2 action send_goal`:

```bash
ros2 action send_goal /execute_behavior_tree rbwatcher_behaviors/action/ExecuteBehaviorTree "{target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}, target_waypoints: [{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}], waypoint_loops: 1, waypoint_start_index: 0, tree_xml: '', tree_path: 'config/default_tree.xml'}"
```

Provide your own behavior tree by setting `tree_xml` (inline XML) or `tree_path` (relative to the package share or an absolute path).

The goal interface also accepts `target_waypoints`, `waypoint_loops`, and `waypoint_start_index` so the default tree can drive Nav2's `FollowWaypoints` action. When these fields are omitted, the single `target_pose` is used as a one-waypoint mission.

## Custom Tree Nodes

- `FollowWaypoints`: Sends a Nav2 `FollowWaypoints` goal with the waypoint array supplied via port `waypoints` and optional loop/start parameters.
- `NavigateToPose`: Sends a Nav2 `NavigateToPose` goal using the pose supplied via port `goal`.
- `TriggerService`: Calls a `std_srvs/Trigger` service, exposing the response string through the `response_message` output port.
- `BoolTopicCondition`: Subscribes to a `std_msgs/Bool` topic and succeeds or fails depending on the observed value.
- `PTZTrackerAction`: Sends a `ptz_tracker_interfaces/TrackTarget` goal and succeeds regardless of the server result (logs warnings when the server reports an error).

These nodes are registered automatically by the action server; reference them directly inside your BehaviorTree XML.

## Extending

1. Implement additional nodes under `src/actions` or `src/conditions` and register them in `BehaviorTreeActionServer::createFactory()`.
2. Add new parameters or blackboard variables in `BehaviorTreeActionServer::setInitialBlackboardValues()`.
3. Update or provide extra BT XML files under `config/` for different mission profiles.

## Testing

While no dedicated test suite ships with the package yet, consider adding BehaviorTree XML regression tests or action-level integration tests to validate future extensions.
