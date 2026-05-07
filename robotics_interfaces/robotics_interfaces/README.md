# robotics_interfaces

Shared ROS 2 interfaces for the Humble warehouse and office course scenarios.

### Messages

| Type | Description |
| --- | --- |
| `msg/TopologyMap.msg` | Topology graph metadata consumed by the scenario and navigation stack. |
| `msg/Vertex.msg` | Named semantic waypoint with pose, floor, and status. |

### Services

| Type | Description |
| --- | --- |
| `srv/SetNode.srv` | Updates logical topology-node state for scenario coordination. |

### Actions

| Type | Description |
| --- | --- |
| `action/Delivery.action` | Generic task envelope used for delivery and patrol routes. |
