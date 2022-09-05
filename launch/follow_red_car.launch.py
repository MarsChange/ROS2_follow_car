from os import environ
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        # ExecuteProcess可执行ign gazebo xx.sdf的shell命令
        # 参考资料https://blog.csdn.net/lgh1231/article/details/123811089
        ExecuteProcess(
            cmd=['ign gazebo',
                 '/home/martin/Desktop/gazebo_ws/src/follow_red_car/src/my_robot.sdf',
                 ],
            output='screen',
            shell=True
        ),
        # 创建ros与ign的桥接节点
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=['/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                       '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
                       '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
            output = 'screen',
            name = 'imu'
        ),
        # 创建ign与ros之间图像传输的节点
        Node(
            package='ros_ign_image',
            executable='image_bridge',
            arguments=['camera'],
            output='screen',
            name='camera'
        ),
        # 创建可执行文件的节点
        Node(
            package='follow_red_car',
            executable='follow_red_car',
            output='screen',
            name='follow_red_car'
        )
    ])