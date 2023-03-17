import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# use this script to fly with two CFs and Joystick

def generate_launch_description():
    # load crazyflies
    crazyflies_yaml = os.path.join(
        get_package_share_directory('cvmrs_ros'),
        'config',
        'crazyflies.yaml')

    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)

    server_params = crazyflies

    # construct motion_capture_configuration
    motion_capture_yaml = os.path.join(
        get_package_share_directory('cvmrs_ros'),
        'config',
        'motion_capture.yaml')

    with open(motion_capture_yaml, 'r') as ymlfile:
        motion_capture = yaml.safe_load(ymlfile)

    motion_capture_params = motion_capture["/motion_capture_tracking"]["ros__parameters"]
    motion_capture_params["rigid_bodies"] = dict()
    for key, value in crazyflies["robots"].items():
        type = crazyflies["robot_types"][value["type"]]
        if value["enabled"] and type["motion_capture"]["enabled"]:
            motion_capture_params["rigid_bodies"][key] =  {
                    "initial_position": value["initial_position"],
                    "marker": type["motion_capture"]["marker"],
                    "dynamics": type["motion_capture"]["dynamics"],
                }

    # teleop params
    teleop_yaml = os.path.join(
        get_package_share_directory('cvmrs_ros'),
        'config',
        'teleop.yaml')
    teleop_5_yaml = os.path.join(
        get_package_share_directory('cvmrs_ros'),
        'config',
        'teleop_5.yaml')
    
    return LaunchDescription([
        Node(
            package='motion_capture_tracking',
            executable='motion_capture_tracking_node',
            name='motion_capture_tracking',
            output='screen',
            parameters=[motion_capture_params]
        ),
        Node(
            package='crazyflie',
            executable='teleop',
            name='teleop',
            remappings=[
                ('emergency', 'all/emergency'),
                ('takeoff', 'all/takeoff'),
                ('land', 'all/land'),
                ('cmd_vel', 'cf231/cmd_vel'),
                ('cmd_full_state', 'cf231/cmd_full_state'),
                ('notify_setpoints_stop', 'cf231/notify_setpoints_stop'),
                ('joy', 'cf231/joy'),
            ],
            parameters=[teleop_yaml]
        ),
        Node(
            package='crazyflie',
            executable='teleop',
            name='teleop',
            remappings=[
                ('emergency', 'all/emergency'),
                ('takeoff', 'all/takeoff'),
                ('land', 'all/land'),
                ('cmd_vel', 'cf3/cmd_vel'),
                ('cmd_full_state', 'cf3/cmd_full_state'),
                ('notify_setpoints_stop', 'cf3/notify_setpoints_stop'),
                # ('joy', 'cf3/joy'),
            ],
            parameters=[teleop_5_yaml]
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            remappings=[('joy', 'cf231/joy')],
            output='screen',
            parameters=[{'device_id':0}] # new joystick
        ),
        # Node(
        #     package='joy',
        #     executable='joy_node',
        #     name='joy_node',
        #     remappings=[('joy', 'cf3/joy')],
        #     parameters=[{'device_id':1}] # old joystick
        # ),  
        Node(
            package='crazyflie',
            executable='crazyflie_server',
            name='crazyflie_server',
            output='screen',
            parameters=[server_params]
        ),
    ])

