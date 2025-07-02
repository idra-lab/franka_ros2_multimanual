import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo,
    Shutdown
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile

import xacro


def robot_description_dependent_nodes_spawner(
        context: LaunchContext,
        robot_ip,
        arm_id,
        use_fake_hardware,
        fake_sensor_commands,
        load_gripper,
        arm_prefix,
        left_ip,
        right_ip
    ):

    robot_ip_str = context.perform_substitution(robot_ip)
    arm_id_str = context.perform_substitution(arm_id)
    arm_prefix_str = context.perform_substitution(arm_prefix)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(
        fake_sensor_commands)
    load_gripper_str = context.perform_substitution(load_gripper)

    franka_xacro_filepath = os.path.join(
        get_package_share_directory('idra_franka_launch'), 'urdf', 'bimanual.urdf.xacro'
    )

    p = '\t' # Padding
    robot_description = xacro.process_file(
        franka_xacro_filepath,
        mappings = {
            'ros2_control': 'true',
            'arm_id': arm_id_str,
            'arm_prefix': arm_prefix_str,
            'robot_ip': robot_ip_str,
            'hand': load_gripper_str,
            'use_fake_hardware': use_fake_hardware_str,
            'fake_sensor_commands': fake_sensor_commands_str,

            'gazebo': 'true',
            'franka1': right_ip,
            'franka2': left_ip
        }
    ).toprettyxml(p)

    franka_controllers = PathJoinSubstitution(
        [FindPackageShare('idra_franka_launch'), 'config', 'basic_controllers.yaml']
    )

    for i in ['franka1', 'franka2']:
        robot_description = robot_description.replace(
            f'{p}<link name="world"/>\n{p}<joint name="world_joint" type="fixed">\n{p}{p}<origin rpy="0 0 0" xyz="0 0 0"/>\n{p}{p}<parent link="world"/>\n{p}{p}<child link="fr3_link0"/>\n{p}</joint>',
            f'{p}<link name="{i}_base"/>\n{p}<joint name="{i}_fr3_{i}_base_joint" type="fixed">\n{p}{p}<origin rpy="0 0 0" xyz="0 0 0"/>\n{p}{p}<parent link="{i}_base"/>\n{p}{p}<child link="{i}_fr3_link0"/>\n{p}</joint>',
            1
        )
    
    robot_description = re.sub(
        f'{p}<gazebo>\n{p}{p}<plugin filename="franka_ign_ros2_control-system" name="ign_ros2_control::IgnitionROS2ControlPlugin">\n{p}{p}{p}<parameters>[\w\/]+\/franka_gazebo_controllers\.yaml</parameters>\n{p}{p}</plugin>\n{p}</gazebo>',
        '',
        robot_description
    )

    print(robot_description)

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),        
        
        # Node(
        #     package='controller_manager',
        #     executable='ros2_control_node',
        #     parameters = [franka_controllers],
        #     #parameters=[
        #     #    ParameterFile(franka_controllers, allow_substs=True),
        #     #            {'robot_description': robot_description},
        #     #            {'arm_id': arm_id},
        #     #            {'load_gripper': load_gripper},
        #     #            {'arm_prefix': arm_prefix},
        #     #],
        #     remappings=[
        #         ('joint_states', 'franka/joint_states'),
        #         #('motion_control_handle/target_frame', 'cartesian_impedance_controller/target_frame'),
        #     ],
        #     output={
        #         'stdout': 'screen',
        #         'stderr': 'screen',
        #     },
        #     on_exit=Shutdown(),
        # ),
    ]


def generate_launch_description():
    arm_id_parameter_name = 'arm_id'
    arm_prefix_parameter_name = 'arm_prefix'
    robot_ip_parameter_name = 'robot_ip'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'

    robot_left_ip_parameter_name = 'left_ip'
    robot_right_ip_parameter_name = 'right_ip'

    arm_id = LaunchConfiguration(arm_id_parameter_name)
    arm_prefix = LaunchConfiguration(arm_prefix_parameter_name)
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name
    )
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    left_ip = LaunchConfiguration(robot_left_ip_parameter_name)
    right_ip = LaunchConfiguration(robot_right_ip_parameter_name)

    rviz_file = os.path.join(get_package_share_directory('idra_franka_launch'), 'rviz', 'bimanual.rviz')

    robot_description_dependent_nodes_spawner_opaque_function = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[
            robot_ip,
            arm_id,
            use_fake_hardware,
            fake_sensor_commands,
            load_gripper,
            arm_prefix,

            left_ip,
            right_ip
        ]
    )
    
    # Gazebo specific configurations
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    controller_left = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_position_controller_left', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    controller_right = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_position_controller_right', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(msg='Shutting down cleanly...'),
            ]
        )
    )

    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            description='Hostname or IP address of the robot.',
            default_value="192.160.100.11"
        ),
        DeclareLaunchArgument(
            arm_id_parameter_name,
            description='ID of the type of arm used. Supported values: fer, fr3, fp3',
            default_value='fr3'
        ),
        DeclareLaunchArgument(
            arm_prefix_parameter_name,
            description='Name of the robot arm. Used for multiple robot configurations.',
            default_value='arm1'
        ),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='true',
            description='Visualize the robot in Rviz'
        ),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware'
        ),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description='Fake sensor commands. Only valid when "{}" is true'.format(
                use_fake_hardware_parameter_name
            )
        ),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'
        ),

        robot_description_dependent_nodes_spawner_opaque_function,
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['joint_state_broadcaster'],
        #     output='screen',
        # ),
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['franka_robot_state_broadcaster'],
        #     parameters=[{'arm_id': 'fr3'}],
        #     output='screen',
        #     condition=UnlessCondition(use_fake_hardware),
        # ),
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['motion_control_handle'],
        #     output='screen',
        #     condition=UnlessCondition(use_fake_hardware),
        # ),
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['cartesian_impedance_controller'],
        #     output='screen',
        #     condition=UnlessCondition(use_fake_hardware),
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([PathJoinSubstitution(
        #         [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
        #     launch_arguments={robot_ip_parameter_name: robot_ip,
        #                       use_fake_hardware_parameter_name: use_fake_hardware}.items(),
        #     condition=IfCondition(load_gripper)
        # ),

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': 'empty.sdf -r --render-engine ogre'}.items(),
        ),

        # Launch Rviz
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file],
             condition=IfCondition(use_rviz)
        ),

        # Spawn
        spawn,

        # Controllers
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_joint_state_broadcaster],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[controller_left],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[controller_right],
            )
        ),

        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     parameters=[{
        #         'source_list': ['franka/joint_states', 'franka_gripper/joint_states'],
        #         'rate': 30
        #     }],
        # ), 

        on_shutdown
    ])

    return launch_description
