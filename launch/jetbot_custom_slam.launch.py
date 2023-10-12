from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
import launch


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    tf_map_odom = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"])  
        
              
    tf_baselink_laser = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0", "3.14", "0", "0", "base_link", "laser"])   
    tf_baselink_odom = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0", "0", "0", "0", "base_link", "map"])                

    laser_odometry = Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom',
                    'publish_tf' : True,
                    'base_frame_id' : 'base_link',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 2.0}],
            )

    start_sync_slam_toolbox_node = Node(
        parameters=[
          get_package_share_directory("jetbot_slam") + '/config/mapper_params_online_sync.yaml',
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        node_executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
        
    rviz =Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
        )

    ld = LaunchDescription()
    ld.add_action(laser_odometry)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(start_sync_slam_toolbox_node)
    ld.add_action(tf_map_odom)
    ld.add_action(tf_baselink_laser)
    ld.add_action(tf_baselink_odom)
    bag=launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a' ,'-o', 'demo_bag'],
            output='screen'
        )
    bag_remove=launch.actions.ExecuteProcess(
            cmd=['rm', '-r', 'demo_bag'],
            output='screen'
        )
    ld.add_action(bag_remove)
    ld.add_action(rviz)
    ld.add_action(bag)
    
    
    return ld