from launch import LaunchDescription 
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
        
    rviz_cfg_file = os.path.join(get_package_share_directory('nswr_ouster'), 'nswr_ouster.rviz')
    print(rviz_cfg_file)     
    
    return LaunchDescription([

        Node(
            package="nswr_ouster",
            executable="nswr_ouster",
            output="screen",
        ),
        
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d" + rviz_cfg_file]
        ),    
         
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            name="link_map_velodyne",
            arguments=["0", "0", "0", "0", "0", "0", "map", "os_sensor"]         
        ),  
    
    ])