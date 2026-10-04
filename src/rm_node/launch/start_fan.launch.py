from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    shootflag_test = DeclareLaunchArgument("shootflag_test", default_value="f")
    camp_test = DeclareLaunchArgument("camp_test", default_value="r")
    sim_mode = DeclareLaunchArgument("sim_mode", default_value="false")
    buff_model_path = DeclareLaunchArgument(
        "buff_model_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rm_tracker"), "models", "fu_moxi", "buff_repvgg.xml"]
        ),
    )
    fu_config_path = DeclareLaunchArgument(
        "fu_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rm_tracker"), "config", "rm_fu.yaml"]
        ),
    )

    camera_params = PathJoinSubstitution(
        [FindPackageShare("rm_node"), "config", "camera_ros2.yaml"]
    )
    fu_params = PathJoinSubstitution(
        [FindPackageShare("rm_tracker"), "config", "fu_ros2.yaml"]
    )

    camera = Node(
        package="rm_hikcamera",
        executable="hikcamera",
        name="hikcamera",
        parameters=[camera_params],
        output="screen",
        respawn=True,
    )
    serial = Node(
        package="rm_serial",
        executable="rm_serial_node",
        name="serial",
        parameters=[
            {
                "shootflag_test": LaunchConfiguration("shootflag_test"),
                "camp_test": LaunchConfiguration("camp_test"),
                "sim_mode": LaunchConfiguration("sim_mode"),
            }
        ],
        output="screen",
        respawn=True,
    )
    fu = Node(
        package="rm_tracker",
        executable="Fu",
        name="Fu",
        parameters=[
            fu_params,
            {
                "buff_model_path": LaunchConfiguration("buff_model_path"),
                "fu_config_path": LaunchConfiguration("fu_config_path"),
            },
        ],
        output="screen",
        respawn=True,
    )

    return LaunchDescription(
        [shootflag_test, camp_test, sim_mode, buff_model_path, fu_config_path,
         camera, serial, fu]
    )
