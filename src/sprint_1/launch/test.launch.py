from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    mcap_file = str('src/sprint_1/data/rosbags/rosbags2_yolo.mcap')

    return LaunchDescription([
        # Nodo 1: unzipper
        Node(
            package='sprint_1',
            executable='unzipper',
            name='unzipper_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', mcap_file, '--rate', '0.1', '--loop'],
            output='screen'
        ),

        # Nodo 3: ollama ayuda en caso de fallar
        # Node(
        #     package='turtlesim',
        #     executable='turtlesim_node',
        #     name='simulador'
        # ),
        LogInfo(msg="¡Lanzando sistema YOLO + Graph + Stanza + Ollama!"),

    ])