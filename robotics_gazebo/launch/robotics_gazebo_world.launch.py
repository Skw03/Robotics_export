import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui", default="true")
    world = LaunchConfiguration("world")
    gazebo_models_path = PathJoinSubstitution(
        [FindPackageShare("robotics_gazebo"), "models"]
    )
    default_world = PathJoinSubstitution(
        [FindPackageShare("robotics_gazebo"), "worlds", "small_city.world"]
    )

    return LaunchDescription([
        SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=gazebo_models_path),
        SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1"),
        SetEnvironmentVariable(name="QT_X11_NO_MITSHM", value="1"),
        SetEnvironmentVariable(name="MESA_LOADER_DRIVER_OVERRIDE", value="llvmpipe"),
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Start robot in Gazebo simulation",
        ),
        DeclareLaunchArgument(
            "use_gazebo_gui",
            default_value="true",
            description="Start Gazebo client GUI when true, otherwise run gzserver only",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=default_world,
            description="Absolute path to the Gazebo world file",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Flag to enable use_sim_time",
        ),
        launch.actions.ExecuteProcess(
            cmd=[
                "gazebo",
                "--verbose",
                "-s",
                "libgazebo_ros_init.so",
                "-s",
                "libgazebo_ros_factory.so",
                world,
            ],
            output="screen",
            condition=IfCondition(use_gazebo_gui),
        ),
        launch.actions.ExecuteProcess(
            cmd=[
                "gzserver",
                "--verbose",
                "-s",
                "libgazebo_ros_init.so",
                "-s",
                "libgazebo_ros_factory.so",
                world,
            ],
            output="screen",
            condition=UnlessCondition(use_gazebo_gui),
        ),
    ])
