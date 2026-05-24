import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_mapper')
    
    # Параметры SLAM Toolbox (Оптимизированы под Raspberry Pi 4)
    slam_params = {
        'use_odom': False,
        'mode': 'mapping',
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_frame': 'base_link',
        'scan_topic': '/scan',
        
        # === ОПТИМИЗАЦИЯ ===
        'transform_timeout': 1.0,            # Увеличенный допуск для TF
        'transform_publish_period': 0.2,     # Реже публикуем TF
        'scan_buffer_size': 3,               # Минимальный буфер
        'minimum_travel_distance': 0.1,      # Игнорируем микро-дрожание
        'minimum_travel_heading': 0.15,
        'map_update_interval': 2.0,          # Обновляем карту раз в 2 сек
        'do_loop_closing': False,            # Отключаем замыкание петель (экономит CPU)
        'correlative_search_enabled': True,
        'correlative_search_linear_search_distance': 0.3,
        'correlative_search_angular_search_distance': 0.35,
        'solver_plugin': 'solver_plugins::CeresSolver',
        'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
        'ceres_preconditioner': 'SCHUR_JACOBI',
        'use_sim_time': False
    }

    return LaunchDescription([
        # 1. Основная нода (Драйверы)
        Node(
            package='robot_mapper',
            executable='robot_ros_node',
            name='robot_node',
            output='screen',
            emulate_tty=True
        ),
        # 2. Веб-управление
        Node(
            package='robot_mapper',
            executable='web_control',
            name='web_node',
            output='screen',
            respawn=True,
            emulate_tty=True
        ),
        # 3. Мультиплексор скоростей
        Node(
            package='robot_mapper',
            executable='cmd_vel_mux',
            name='cmd_mux',
            output='screen',
            respawn=True,
            emulate_tty=True
        ),
        # 4. SLAM (Картографирование)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            respawn=True,
            emulate_tty=True,
            parameters=[slam_params]
        ),
        # 4. Камера (V1.3)
        Node(
            package='robot_mapper',
            executable='camera_publisher',
            name='camera_node',
            output='screen',
            respawn=True,
            emulate_tty=True
        ),
        # 5. Мост Foxglove
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='fg_bridge',
            output='screen',
            respawn=True,
            emulate_tty=True,
            parameters=[{'port': 8765, 'max_qos_depth': 10}]
        )
    ])
