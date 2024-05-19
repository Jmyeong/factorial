from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='qr_localization',
            executable='qr_camera',
            name='qr_camera',
            output='screen',
            #parameters=[
                #{'parameter_name': 'parameter_value'}
            #]
        ),
        
        Node(
            package='qr_localization',
            executable='z_coord_estimation',
            name='gaussian_mean_calculator',
            output='screen'
        ),
    ])
