from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    turtle_arg = DeclareLaunchArgument('turtle', default_value='turtle1')
    order_arg = DeclareLaunchArgument('order', default_value='3')
    side_length_arg = DeclareLaunchArgument('side_length', default_value='5.0')
    start_x_arg = DeclareLaunchArgument('start_x', default_value='2.0')
    start_y_arg = DeclareLaunchArgument('start_y', default_value='7.0')
    theta_arg = DeclareLaunchArgument('theta', default_value='0.0')
    pen_r_arg = DeclareLaunchArgument('pen_r', default_value='255')
    pen_g_arg = DeclareLaunchArgument('pen_g', default_value='128')
    pen_b_arg = DeclareLaunchArgument('pen_b', default_value='128')
    pen_width_arg = DeclareLaunchArgument('pen_width', default_value='2')

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

    initial_pen_off = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/turtle1/set_pen', 'turtlesim/srv/SetPen',
            '"{r:255, g:0, b:0, width:2, off:1}"'
        ],
        shell=True
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

    delayed_koch = TimerAction(period=1.0, actions=[koch_node])

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
        TimerAction(period=0.5, actions=[initial_pen_off]),
        delayed_koch,
    ])
