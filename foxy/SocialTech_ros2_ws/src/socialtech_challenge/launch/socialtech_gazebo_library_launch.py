import os

import launch
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
import launch_ros.descriptions
import launch_ros.substitutions
from launch_ros.descriptions import ParameterValue
 
def generate_launch_description():

  # Pose where we want to spawn the robot
  spawn_x_val = '1.0'
  spawn_y_val = '1.0'
  spawn_z_val = '0.0'
  spawn_yaw_val = '0.00'
  urdf_file_path = 'urdf/tracer2.xacro'
  robot_name_in_model = 'tracer2'
  
  mundo= launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [get_package_share_directory(
                    'aws_robomaker_bookstore_world'), '/launch/bookstore.launch.py']
            ),
            launch_arguments={
                'gui': 'true'
            }.items()
        )
    
  # Set the path to this package.
  pkg_share = FindPackageShare(package='socialtech_challenge').find('socialtech_challenge')
  default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
  
  ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############  
  # Launch configuration variables specific to simulation
  use_sim_time = LaunchConfiguration('use_sim_time')

  urdf_model = LaunchConfiguration('urdf_model')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  gui = LaunchConfiguration('gui')
 
  robot_description = xacro.process_file(default_urdf_model_path).toxml()
 
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')
  
  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=default_urdf_model_path, 
    description='Absolute path to robot urdf file')
  
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')
  
  declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
    name='gui',
    default_value='True',
    description='Flag to enable joint_state_publisher_gui')
    
  # Specify the actions

  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.    
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
     parameters=[{
        'use_sim_time': use_sim_time,
        'robot_description': robot_description
      }]
    )
 
  # Publish the joint states of the robot
  start_joint_state_publisher_cmd = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
  )
   
  
  spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", robot_name_in_model, "-x", "-2.1", "-y", "5.6", "-z", "0.2"],
        output="screen")
 
  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Add any actions
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(mundo)
  ld.add_action(spawn_entity_cmd)
  ld.add_action(declare_urdf_model_path_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_joint_state_publisher_cmd)


  return ld