import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_path = "/home/tzz/ros2maze_ws/src/maze_control/worlds/z_maze.world"

    # 启动Gazebo，加载自定义world
    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            world_path,
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so"
        ],
        output="screen"
    )

    # 生成TurtleBot3 Burger小车
    spawn_tb3 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "turtlebot3_burger",
            "-x", "-2.0", "-y", "-1.6", "-z", "0.01"
        ],
        output="screen"
    )

    # robot_state_publisher，发布机器人URDF
    tb3_model_dir = get_package_share_directory("turtlebot3_gazebo")
    urdf_path = os.path.join(tb3_model_dir, "models", "turtlebot3_burger", "model.sdf")
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": open(urdf_path).read()}]
    )

    wall_follower = Node(
        package="maze_control",            
        executable="tb3_wall_follower",     
        name="tb3_wall_follower",
        output="screen",
    )
    delayed_wall_follower = TimerAction(
        period=10.0,                          
        actions=[wall_follower]
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )
    delayed_rviz2 = TimerAction(period=3.0, actions=[rviz2])

    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn_tb3,
        delayed_rviz2,
        delayed_wall_follower,
    ])