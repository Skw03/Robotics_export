from pathlib import Path

path = Path("/home/qzb/ros2_ws/src/Robotics/robotics_description/launch/robotics_gazebo.launch.py")
text = path.read_text()

if "plugin = os.pathsep.join" not in text:
    insert = (
        "    plugin = os.pathsep.join(\n"
        "        path\n"
        "        for path in [plugin, os.getenv(\\\"GAZEBO_PLUGIN_PATH\\\", \\\"\\\"), \\\"/opt/ros/humble/lib\\\"]\n"
        "        if path\n"
        "    )\n"
    )
    text = text.replace(
        "    model, plugin, media = GazeboRosPaths.get_paths()\n",
        "    model, plugin, media = GazeboRosPaths.get_paths()\n" + insert,
        1,
    )

if '"-timeout"' not in text:
    marker = '            "robot_description",\n'
    text = text.replace(
        marker,
        marker + '            "-timeout",\n            "120",\n',
        1,
    )

path.write_text(text)
