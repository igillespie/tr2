import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Set the path to different files and folders.
  #pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_share = FindPackageShare(package='two_wheeled_robot').find('two_wheeled_robot')
  
  default_launch_dir = os.path.join(pkg_share, 'launch')
  default_model_path = os.path.join(pkg_share, 'urdf/two_wheeled_robot.urdf')
  robot_localization_file_path = os.path.join(pkg_share, 'config/tr2_ekf.yaml')
  teleop_joy_params_path = os.path.join(pkg_share, 'config/teleop_twist_joy_node.yaml')
  joy_params_path = os.path.join(pkg_share, 'config/joy_node.yaml')
  lidar_params_path = os.path.join(pkg_share, 'config/lidar.yaml')
  
  robot_name_in_urdf = 'two_wheeled_robot'
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/tr2.rviz')
  #world_file_name = 'two_wheeled_robot_world/smalltown.world'
  #world_path = os.path.join(pkg_share, 'worlds', world_file_name)
  
  
  
  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  model = LaunchConfiguration('model')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  use_lidar = LaunchConfiguration('use_lidar')
  
  #world = LaunchConfiguration('world')
  
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  declare_use_sim_time_cmd = DeclareLaunchArgument(
	name='use_sim_time',
   	default_value='False',
   	description='Use simulation (Gazebo) clock if true')

  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='False',
    description='Whether to start the simulator')
      
  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
    
  declare_use_lidar_cmd = DeclareLaunchArgument(
    name='use_lidar',
    default_value='True',
    description='Whether to start LiDAR')
    
  # Declare the launch arguments  
  declare_model_path_cmd = DeclareLaunchArgument(
   	name='model', 
    default_value=default_model_path, 
    description='Absolute path to robot urdf file')
    
  
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')
        
  # Start robot localization using an Extended Kalman filter
  start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path, 
    {'use_sim_time': use_sim_time}])

  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': use_sim_time, 
    'robot_description': Command(['xacro ', model])}],
    arguments=[default_model_path])

  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])    
   
  #micro ros agent
  micro_ros_agent = Node(
  	package="micro_ros_agent",
  	name='micro_ros_agent',
  	executable="micro_ros_agent",
  	output="screen",
  	arguments=['serial','--dev', '/dev/ttyACM0'])
  	
  lidar_node = Node(package='ydlidar_ros2_driver',
  	condition=IfCondition(use_lidar),
	executable='ydlidar_ros2_driver_node',
	name='ydlidar_ros2_driver_node',
	output='screen',
	emulate_tty=True,
	parameters=[lidar_params_path])
		        
  tf2_node = Node(package='tf2_ros',
	executable='static_transform_publisher',
	name='static_tf_pub_laser',
	arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'])
  
  #Joy stick control
  joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
        parameters=[joy_params_path])
        
  
  joy_teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[teleop_joy_params_path])
        
  odometry = Node(
        package="ekf_odom_pub",
        name="ekf_odom_pub",
        executable="ekf_odom_pub",
        output="screen")
        
  imu = Node(
        package="bno055_publisher",
        name="bno055_publisher",
        executable="bno055",
        output="screen")

  
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_model_path_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_use_lidar_cmd)
  #ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  #ld.add_action(declare_world_cmd)

  # Add any actions
  
  ld.add_action(micro_ros_agent)
  ld.add_action(lidar_node)
  ld.add_action(tf2_node)
  ld.add_action(joy_node)
  ld.add_action(joy_teleop_node)
  ld.add_action(odometry)
  ld.add_action(imu)
  ld.add_action(start_robot_localization_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_rviz_cmd)

  return ld
