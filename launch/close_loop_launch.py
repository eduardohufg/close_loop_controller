from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    node1 = Node(package='turtlesim',
                       executable='turtlesim_node',
                       name="turtlesim",
                       )
    
    node2 = Node(package='close_loop_controller',
                          executable='odometry',
                            name="odometry",
                          )
        
    node3 = Node(package='close_loop_controller',
                       executable='path_generator',
                       name="path_generator",
                       )
    
    node4 = Node(package='close_loop_controller',
                       executable='close_loop_controller',
                       name="close_loop_controller",
                       )
    
    node5 = Node(package='close_loop_controller',
                        executable='color_detection',
                        name="color_detection",
                        )
    

    
    
    
    l_d = LaunchDescription([node1, node2, node3, node4, node5])

    return l_d