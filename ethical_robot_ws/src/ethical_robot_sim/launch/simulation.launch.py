from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare(package='ethical_robot_sim').find('ethical_robot_sim')
    
    # Set the path to the URDF file
    urdf_file_name = 'ethical_robot.urdf.xacro'
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name)
    
    # Set the path to the world file
    world_file_name = 'dilemma_world.sdf'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    
    # Set the path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_share, 'config', 'nav.rviz')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    ethical_framework = LaunchConfiguration('ethical_framework')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Open RViz if true')
    
    declare_ethical_framework_cmd = DeclareLaunchArgument(
        name='ethical_framework',
        default_value='UTILITARIAN',
        description='Ethical framework to use (UTILITARIAN, KANTIAN, CARE_ETHICS, VIRTUE_ETHICS, CUSTOM)')
    
    # Start Gazebo with the dilemma world
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen')
    
    # Publish the robot state to tf
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', urdf_path])}])
    
    # Spawn the robot in Gazebo
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', 'ethical_robot',
                   '-topic', 'robot_description',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.1',
                   '-Y', '0.0'])
    
    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')
    
    # Start the perception node
    perception_node_cmd = Node(
        package='ethical_robot_sim',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'area1_center_x': 12.0},
            {'area1_center_y': 6.0},
            {'area2_center_x': 10.0},
            {'area2_center_y': -5.0},
            {'area1_radius': 3.0},
            {'area2_radius': 3.0},
            {'decision_point_x': 5.0},
            {'decision_point_y': 0.0},
            {'decision_radius': 1.0}
        ])
    
    # Start the decision engine
    decision_engine_cmd = Node(
        package='ethical_robot_sim',
        executable='decision_engine',
        name='decision_engine',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'ethical_framework': ethical_framework},
            {'utilitarian_people_weight': 0.7},
            {'utilitarian_time_weight': 0.3},
            {'decision_threshold': 0.1},
            {'uncertainty_tolerance': 0.2}
        ])
    
    # Start the robot controller
    robot_controller_cmd = Node(
        package='ethical_robot_sim',
        executable='extended_robot_controller',
        name='robot_controller',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'linear_speed': 0.3},
            {'angular_speed': 0.5},
            {'obstacle_threshold': 0.5},
            {'goal_tolerance': 0.2},
            {'decision_point_x': 5.0},
            {'decision_point_y': 0.0},
            {'area1_x': 12.0},
            {'area1_y': 6.0},
            {'area2_x': 10.0},
            {'area2_y': -5.0}
        ])
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_ethical_framework_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(perception_node_cmd)
    ld.add_action(decision_engine_cmd)
    ld.add_action(robot_controller_cmd)
    
    return ld