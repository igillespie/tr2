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
  nav_params_path = os.path.join(pkg_share, 'config/slam.yaml')
  map_server_params_path = os.path.join(pkg_share, 'config/map_server.yaml')
  #amcl_params = os.path.join(pkg_share, '/config/amcl.yaml')
  
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
  use_slam = LaunchConfiguration('use_slam')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  use_lidar = LaunchConfiguration('use_lidar')
  use_voice = LaunchConfiguration('use_voice')
  
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
    
  declare_use_slam_cmd = DeclareLaunchArgument(
    name='use_slam',
    default_value='False',
    description='Whether to start SLAM toolbox')
    
  declare_use_lidar_cmd = DeclareLaunchArgument(
    name='use_lidar',
    default_value='True',
    description='Whether to start LiDAR')
    
  declare_use_voice_cmd = DeclareLaunchArgument(
    name='use_voice',
    default_value='True',
    description='Whether to use voice recognition')
    
  # Declare the launch arguments  
  declare_model_path_cmd = DeclareLaunchArgument(
   	name='model', 
    default_value=default_model_path, 
    description='Absolute path to robot urdf file')
    
  
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')
    
    
  declare_map_command = DeclareLaunchArgument(
     'map',
     default_value=os.path.join(pkg_share, 'maps', 'map.yaml'),
     description='Full path to map yaml file to load')
    
  # Map fully qualified names to relative ones so the node's namespace can be prepended.
  # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
  # https://github.com/ros/geometry2/issues/32
  # https://github.com/ros/robot_state_publisher/pull/30
  # TODO(orduno) Substitute with `PushNodeRemapping`
  #              https://github.com/ros2/launch_ros/issues/56
  
  remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]
        
  # Start robot localization using an Extended Kalman filter
  start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    #output='screen',
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
    
    
  start_sync_slam_toolbox_node = Node(
    condition=IfCondition(use_slam),
    parameters=[
          nav_params_path,
          {'use_sim_time': use_sim_time}
        ],
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    output='screen')
    
  #amcl = Node(
    #condition=IfCondition(use_slam),
    #package='nav2_amcl',
    #executable='amcl',
    #name='amcl',
    #output='screen',
    #parameters=[amcl_params],
    #remappings=remappings)
    
    
  map_server = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    output='screen',
    #parameters=[map_server_params_path],
    remappings=remappings)

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
	#output='screen',
	emulate_tty=True,
	parameters=[lidar_params_path])
	
  #base_link_tf = Node(package='tf2_ros',
	#executable='static_transform_publisher',
	#name='base_link_tf_broadcaster',
	#output='screen',
	#arguments=['0.0', '0.0', '0.0','0', '0', '0', '1','base_footprint','base_link'])
		        
  #lidar_link_tf = Node(package='tf2_ros',
	#executable='static_transform_publisher',
	#name='laser_tf_broadcaster',
	#arguments=['0.18', '0.0', '0.33','0', '0', '0', '1','base_link','lidar_link'])
  
  
  
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
        package="diff_drive_controller",
        name="diff_drive_controller",
        executable="diff_drive_controller",
        parameters=[{"broadcast_transform" : False}],
        output="screen")
        
  imu = Node(
        package="bno055_publisher",
        name="bno055_publisher",
        namespace="imu",
        executable="bno055",
        output="screen")

  voice_recog = Node(
        condition=IfCondition(use_voice),
        package="voice_recognition",
        name="voice_recognition",
        namespace="speech",
        executable="voice_recognition",
        output="screen")
        
  voice_tts = Node(
        condition=IfCondition(use_voice),
        package="voice_tts",
        name="voice_tts",
        namespace="speech",
        executable="voice_tts",
        output="screen")
        
  voice_interpreter = Node(
        condition=IfCondition(use_voice),
        package="voice_interpreter",
        name="voice_interpreter",
        namespace="speech",
        executable="voice_interpreter",
        output="screen")
        
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_model_path_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_use_lidar_cmd)
  ld.add_action(declare_use_voice_cmd)
  ld.add_action(declare_use_slam_cmd)
  #ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  #ld.add_action(declare_map_command)
  #ld.add_action(declare_world_cmd)

  # Add any actions
  ld.add_action(micro_ros_agent)
  ld.add_action(lidar_node)
  #ld.add_action(base_link_tf)
  #ld.add_action(lidar_link_tf)
  ld.add_action(joy_node)
  ld.add_action(joy_teleop_node)
  ld.add_action(odometry)
  ld.add_action(imu)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_robot_localization_cmd)
  ld.add_action(start_sync_slam_toolbox_node)
  ld.add_action(voice_recog)
  ld.add_action(voice_tts)
  ld.add_action(voice_interpreter)
  #ld.add_action(map_server)
  #ld.add_action(amcl)

  ld.add_action(start_rviz_cmd)

  return ld
