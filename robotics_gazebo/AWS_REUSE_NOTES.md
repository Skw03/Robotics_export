# AWS Scene Reuse Notes

This package vendors the `small_warehouse.world` assets from
`aws-robotics/aws-robomaker-small-warehouse-world` under the repository's
Apache-2.0 license.

What is reused here:

- The original AWS small warehouse Gazebo models.
- The original AWS small warehouse world layout.
- The rotated occupancy map used for AMCL/Nav2 in the ROS 2 Humble course stack.

What is intentionally not reused:

- The ROS 1 launch and catkin package structure from the upstream repository.
- Any ROS 1 navigation or SLAM assumptions from the upstream example.

In this course workspace the AWS world is treated purely as a reusable map/world
asset and is launched through the local ROS 2 Humble bringup chain.
