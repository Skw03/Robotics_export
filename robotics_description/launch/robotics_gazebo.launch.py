import launch


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
from launch_ros.descriptions import ParameterValue

# from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
import os

from ament_index_python.packages import get_package_share_directory
from scripts import GazeboRosPaths


def _normalize_model_path(path_value, fallback_path):
    if not path_value:
        return fallback_path

    normalized_paths = []
    for raw_path in path_value.split(os.pathsep):
        candidate = raw_path.strip()
        if not candidate:
            continue

        candidate_for_match = candidate.replace("\\", "/")
        if candidate_for_match.endswith("/robotics_gazebo/models") and "ros2_ws" in candidate_for_match:
            candidate = fallback_path

        if candidate not in normalized_paths:
            normalized_paths.append(candidate)

    if fallback_path not in normalized_paths:
        normalized_paths.insert(0, fallback_path)

    return os.pathsep.join(normalized_paths)


def generate_launch_description():
    # 환경 변수 설정
    model, plugin, media = GazeboRosPaths.get_paths()

    gazebo_model_path = os.getenv("GAZEBO_MODEL_PATH", "")
    robotics_gazebo_model_path = os.path.join(
        get_package_share_directory("robotics_gazebo"), "models"
    )
    normalized_model_path = _normalize_model_path(model, robotics_gazebo_model_path)
    combined_gazebo_model_path = os.pathsep.join(
        path
        for path in [
            normalized_model_path,
            gazebo_model_path,
            robotics_gazebo_model_path,
        ]
        if path
    )

    gazebo_resource_path = os.getenv("GAZEBO_RESOURCE_PATH", "")
    combined_gazebo_resource_path = (
        f"{gazebo_resource_path}:{media}" if gazebo_resource_path else media
    )

    # GAZEBO_MODEL_PATH 환경 변수 설정
    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH", value=combined_gazebo_model_path
    )
    # GAZEBO_PLUGIN_PATH 환경 변수 설정
    set_gazebo_plugin_path = SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH", value=plugin
    )
    # GAZEBO_MODEL_PATH 환경 변수 설정
    set_gazebo_resource_path = SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH", value=combined_gazebo_resource_path
    )
    set_qt_no_mitshm = SetEnvironmentVariable(
        name="QT_X11_NO_MITSHM", value="1"
    )
    force_software_rendering = LaunchConfiguration(
        "force_software_rendering", default="false"
    )
    set_software_rendering = SetEnvironmentVariable(
        name="LIBGL_ALWAYS_SOFTWARE",
        value="1",
        condition=IfCondition(force_software_rendering),
    )
    set_mesa_driver = SetEnvironmentVariable(
        name="MESA_LOADER_DRIVER_OVERRIDE",
        value="llvmpipe",
        condition=IfCondition(force_software_rendering),
    )

    start_rviz = LaunchConfiguration("start_rviz")
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui", default="True")
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # robot urdf 파일의 경로를 설정합니다.
    default_model_dir = PathJoinSubstitution(
        [FindPackageShare("robotics_description"), "urdf", "robotics.urdf.xacro"]
    )

    # rviz 파일의 경로를 설정합니다.
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robotics_description"), "rviz", "display.rviz"]
    )
    # world 파일의 경로를 설정합니다.
    world_dir = PathJoinSubstitution(
        [FindPackageShare("robotics_gazebo"), "worlds", "small_city.world"]
    )
    selected_world = LaunchConfiguration("world", default=world_dir)
    spawn_x = LaunchConfiguration("spawn_x", default="0.5")
    spawn_y = LaunchConfiguration("spawn_y", default="0.5")
    spawn_z = LaunchConfiguration("spawn_z", default="0.01")

    # robot_state_publisher를 실행하는 노드를 설정합니다.

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_description": ParameterValue(
                    Command(["xacro ", LaunchConfiguration("model")]), value_type=str
                ),
            }
        ],
    )

    # joint_state_publisher를 실행하는 노드를 설정합니다.
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=launch.conditions.IfCondition(LaunchConfiguration("gui")),
    )

    # rviz를 실행하는 노드를 설정합니다.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        #output="screen",
        condition=IfCondition(start_rviz),
    )

    # gazebo를 실행하여 월드를 불러옵니다.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "robotics",
            "-topic",
            "robot_description",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            set_gazebo_model_path,
            set_gazebo_plugin_path,
            set_gazebo_resource_path,
            set_software_rendering,
            set_qt_no_mitshm,
            set_mesa_driver,
            # 런치 파일에 사용할 인자들을 정의합니다.
            DeclareLaunchArgument(
                "start_rviz", default_value="true", description="Whether execute rviz2"
            ),
            DeclareLaunchArgument(
                "use_sim",
                default_value="true",
                description="Start robot in Gazebo simulation",
            ),
            DeclareLaunchArgument(
                name="gui",
                default_value="False",
                description="Flag to enable joint_state_publisher_gui",
            ),
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_dir,
                description="Absolute path to robot urdf file",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=rviz_config_file,
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            DeclareLaunchArgument(
                name="force_software_rendering",
                default_value="false",
                description="Use CPU software rendering for Gazebo when GPU rendering is unstable",
            ),
            DeclareLaunchArgument(
                name="world",
                default_value=world_dir,
                description="Absolute path to Gazebo world file",
            ),
            DeclareLaunchArgument(
                name="spawn_x",
                default_value="0.5",
                description="Robot spawn x position",
            ),
            DeclareLaunchArgument(
                name="spawn_y",
                default_value="0.5",
                description="Robot spawn y position",
            ),
            DeclareLaunchArgument(
                name="spawn_z",
                default_value="0.01",
                description="Robot spawn z position",
            ),
            # gazebo를 실행합니다.
            launch.actions.ExecuteProcess(
                cmd=[
                    "gazebo",
                    "--verbose",
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                    selected_world,
                ],
                output="screen",
                condition=IfCondition(use_gazebo_gui),
            ),
            # gazebo를 실행합니다.
            launch.actions.ExecuteProcess(
                cmd=[
                    "gzserver",
                    "--verbose",
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                    selected_world,
                ],
                output="screen",
                condition=UnlessCondition(use_gazebo_gui),
            ),
            # 위에서 정의한 노드들을 실행합니다.
            # 로봇의 상태를 퍼블리시하는 노드
            # joint_state_publisher_node,
            robot_state_publisher_node,
            # joint_state_publisher_gui_node,
            # 로봇을 gazebo에 스폰하는 노드
            spawn_entity,
            # rviz_node,
        ]
    )
