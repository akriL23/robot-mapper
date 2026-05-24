import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_mapper')

    return LaunchDescription([
        # ── 1. Core: Lidar, Motors, Odom, TF ────────────────────────
        Node(
            package='robot_mapper',
            executable='robot_ros_node',
            name='robot_ros_node',
            output='screen'
        ),

        # ── 2. Multiplexer: Prioritizes Manual over Auto ────────────
        Node(
            package='robot_mapper',
            executable='cmd_vel_mux',
            name='cmd_vel_mux',
            output='screen',
            parameters=[{
                'manual_timeout': 0.8,  # Match web_control.py keepalive
                'auto_timeout': 1.2,
                'loop_hz': 40.0
            }]
        ),

        # ── 3. Ultrasonic Sensors (4 sensors) ───────────────────────
        Node(
            package='robot_mapper',
            executable='ultrasonic_node',
            name='ultrasonic_node',
            output='screen'
        ),

        # ── 4. Joystick Hardware Driver (reads /dev/input/js0) ──────
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',
                'autorepeat_rate': 20.0  # Send data even when idle (for UI)
            }]
        ),

        # ── 5. Joystick Logic: /joy -> /cmd_vel/manual ──────────────
        Node(
            package='robot_mapper',
            executable='joystick_control',
            name='joystick_control',
            output='screen',
            parameters=[{
                'linear_scale': 0.5,
                'angular_scale': 1.5
            }]
        ),

        # ── 6. Camera (USB /dev/video0) ─────────────────────────────
        Node(
            package='robot_mapper',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen'
        ),

        # ── 7. Web Control UI (Flask Server + Status Node) ──────────
        Node(
            package='robot_mapper',
            executable='web_control',
            name='web_control',
            output='screen'
        ),

        # ── 8. Foxglove Bridge (WebSocket) ──────────────────────────
        # ИСПРАВЛЕНО: executable='foxglove_bridge' (без _node)
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge', 
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',       # Слушать все интерфейсы
                'send_buffer_limit': 10000000 # Буфер для лидара/камеры
            }]
        ),
    ])
