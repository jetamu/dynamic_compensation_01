import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo 1: El estimador adaptativo de parámetros
        Node(
            package='jt_dynamic_controller',
            executable='parameter_estimator_node',
            name='parameter_estimator_node',
            output='screen',
            # parameters=[{'k_u': 2.0, 'k_w': 2.0}] # Ejemplo de cómo podrías pasar parámetros desde aquí en el futuro
        )
    ])