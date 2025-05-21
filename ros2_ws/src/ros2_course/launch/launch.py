from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    turtle_arg = DeclareLaunchArgument(
        'turtle',
        default_value='turtle1',
        description='Name of the turtle to draw'
    )
    order_arg = DeclareLaunchArgument(
        'order',
        default_value='3',
        description='Order of the Koch snowflake'
    )
    side_length_arg = DeclareLaunchArgument(
        'side_length',
        default_value='5.0',
        description='Length of each side'
    )
    start_x_arg = DeclareLaunchArgument(
        'start_x',
        default_value='2.0',
        description='Starting X position'
    )
    start_y_arg = DeclareLaunchArgument(
        'start_y',
        default_value='7.0',
        description='Starting Y position'
    )
    theta_arg = DeclareLaunchArgument(
        'theta',
        default_value='0.0',
        description='Starting orientation (radians)'
    )
    pen_r_arg = DeclareLaunchArgument(
        'pen_r',
        default_value='255',
        description='Pen color red channel (0-255)'
    )
    pen_g_arg = DeclareLaunchArgument(
        'pen_g',
        default_value='0',
        description='Pen color green channel (0-255)'
    )
    pen_b_arg = DeclareLaunchArgument(
        'pen_b',
        default_value='0',
        description='Pen color blue channel (0-255)'
    )
    pen_width_arg = DeclareLaunchArgument(
        'pen_width',
        default_value='2',
        description='Pen width'
    )
    turtle = LaunchConfiguration('turtle')
    order = LaunchConfiguration('order')
    side_length = LaunchConfiguration('side_length')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    theta = LaunchConfiguration('theta')
    pen_r = LaunchConfiguration('pen_r')
    pen_g = LaunchConfiguration('pen_g')
    pen_b = LaunchConfiguration('pen_b')
    pen_width = LaunchConfiguration('pen_width')
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim'
    )

    koch_node = Node(
        package='ros2_course',
        executable='koch_snowflake',
        name='koch_snowflake',
        parameters=[{
            'turtle': turtle,
            'order': order,
            'side_length': side_length,
            'start_x': start_x,
            'start_y': start_y,
            'theta': theta,
            'pen_r': pen_r,
            'pen_g': pen_g,
            'pen_b': pen_b,
            'pen_width': pen_width,
        }]
    )

    delayed_koch = TimerAction(
        period=2.0,
        actions=[koch_node]
    )

    return LaunchDescription([
        turtle_arg,
        order_arg,
        side_length_arg,
        start_x_arg,
        start_y_arg,
        theta_arg,
        pen_r_arg,
        pen_g_arg,
        pen_b_arg,
        pen_width_arg,
        turtlesim_node,
        delayed_koch,
    ])
