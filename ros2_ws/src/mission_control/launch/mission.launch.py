from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    ws_path = '/home/kristianhans/Documents/Personal/ai4good/ros2_ws'
    return LaunchDescription([
        ExecuteProcess(
            cmd=[os.path.join(ws_path, 'install/sensing/bin/fruit_detector')],
            name='fruit_detector'
        ),
        ExecuteProcess(
            cmd=[os.path.join(ws_path, 'install/sensing/bin/color_sensor')],
            name='color_sensor'
        ),
        ExecuteProcess(
            cmd=[os.path.join(ws_path, 'install/navigation/bin/navigator')],
            name='navigator'
        ),
        ExecuteProcess(
            cmd=[os.path.join(ws_path, 'install/navigation/bin/odometry')],
            name='odometry'
        ),
        ExecuteProcess(
            cmd=[os.path.join(ws_path, 'install/manipulation/bin/seed_planter')],
            name='seed_planter'
        ),
        ExecuteProcess(
            cmd=[os.path.join(ws_path, 'install/manipulation/bin/fruit_picker')],
            name='fruit_picker'
        ),
        ExecuteProcess(
            cmd=[os.path.join(ws_path, 'install/mission_control/bin/mission_controller')],
            name='mission_controller'
        ),
    ])