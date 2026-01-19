from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument,OpaqueFunction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit


ARGUMENTS = [
    DeclareLaunchArgument("namespace", default_value='x500_1', description="Robot #1 namespace"),
    DeclareLaunchArgument(
        'user_config_file',
        default_value = PathJoinSubstitution([FindPackageShare("husarion_bag_recorder"), "config", "params.yaml"]),
        description="Path to config file"
    )]

def generate_launch_description():
    bag_recorder_params_file = LaunchConfiguration("user_config_file")
    namespace = LaunchConfiguration("namespace")

    bag_recorder_node = Node(
        package="husarion_bag_recorder",
        executable="bag_recorder",
        name="husarion_bag_recorder_node",
        parameters=[bag_recorder_params_file],
        namespace=namespace)
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(bag_recorder_node)
    return ld
    