from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jt_dynamic_controller',
            executable='dynamic_compensation_node',
            name='dynamic_compensation_node',
            output='screen',
            emulate_tty=True, # Esto permite que los logs (RCLCPP_INFO) salgan con color en la terminal
            # parameters=[
            #     {'ejemplo_parametro': 1.0} 
            # ]
        )
    ])