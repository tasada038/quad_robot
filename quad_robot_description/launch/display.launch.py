from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    packages_name = "quad_robot_description"
    xacro_file_name = "quad_robot.xacro"
    rviz_file_name = "urdf.rviz"

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(packages_name), "urdf", xacro_file_name]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(packages_name), "rviz", rviz_file_name]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )
    joint_state_pub_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes = [
        rviz_node,
        robot_state_pub_node,
        joint_state_pub_gui_node,
    ]

    return LaunchDescription(nodes)