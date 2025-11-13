from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ld = LaunchDescription([
        DeclareLaunchArgument('background_r', default_value='0'),
        DeclareLaunchArgument('background_g', default_value='122'), 
        DeclareLaunchArgument('background_b', default_value='84'),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            parameters=[{
                'background_r': LaunchConfiguration('background_r'),
                'background_g': LaunchConfiguration('background_g'),
                'background_b': LaunchConfiguration('background_b'),
            }]
        )
    ])
    ld.add_action(ComposableNodeContainer(
        
        name='custom_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='turtle_custom',
                plugin='custom::TurtlePlugin',
                name='pose_monitor'
            )
        ]
    ))
    return ld