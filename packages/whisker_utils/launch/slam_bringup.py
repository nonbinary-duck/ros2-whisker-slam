import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

#
# Launch file based on the substitutions tutorial: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html
# And my tidybot assignment from autonomous mobile robotics
#

def generate_launch_description():

    # Line of code adapted from the nav2_bringup slam_launch.py file
    package_dir = get_package_share_directory('whisker_utils');

    # Initialise a launch description object to add to
    ld = LaunchDescription();

    #
    # Params
    #
    slam_for_real_lidar   = DeclareLaunchArgument(
        "slam_for_real_lidar",
        default_value="False",
        description="Launch the SLAM stack for the ground-truth"
    );

    launch_coppelia       = DeclareLaunchArgument(
        "launch_coppelia",
        default_value="False",
        description="If Coppelia should be launched (requires $COPPELIASIM_ROOT_DIR to be exported)"
    );

    launch_nav2          = DeclareLaunchArgument(
        "launch_nav2",
        default_value="False",
        description="If the navigation2 stack should be launched, taking the SLAM map as input"
    );

    run_rviz              = DeclareLaunchArgument(
        "run_rviz",
        default_value="True",
        description="If rviz should be launched using the rviz config in /rviz"
    );

    run_simple_controller = DeclareLaunchArgument(
        "run_simple_controller",
        default_value="False",
        description="If the simple reactive controller should be launched"
    );

    # Add our arguments to the launch description
    ld.add_action(slam_for_real_lidar);
    ld.add_action(launch_coppelia);
    ld.add_action(launch_nav2);
    ld.add_action(run_rviz);
    ld.add_action(run_simple_controller);


    # Get the path to coppelia simulate
    coppelia_path = os.getenv("COPPELIASIM_ROOT_DIR");

    # If we're using coppelia, we obviously need to know where it is
    if ((coppelia_path == None) and PythonExpression([ launch_coppelia ])):
        raise RuntimeError("Cannot launch Coppelia without the environment variable $COPPELIASIM_ROOT_DIR");

    # Launch SLAM
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("slam_toolbox"),
                    "launch",
                    "online_async_launch.py"
                ])
            ]),
            launch_arguments={
                "use_sim_time": "True",
                "slam_params_file": os.path.join( package_dir, "params", "mapper_params_online_sync.yaml" )
            #                                      (
            #         # If we're using the real lidar we have a different set of params
            #         "real_lidar_mapper_params_online_sync.yaml" if (PythonExpression([slam_for_real_lidar])) else "mapper_params_online_sync.yaml" )
            }.items()
        )
    );

    # Conditionally include the Nav2 stack
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py"
                ])
            ]),
            launch_arguments={
                "use_sim_time": "True",
                "params_file": os.path.join( package_dir, "params", "nav2_params.yaml" )
            }.items(),
            # Only run according to the param
            condition=LaunchConfiguration(launch_nav2)
            # condition=IfCondition(
            #     PythonExpression([ launch_nav2 ])
            # )
        )
    );

    # Conditionally run rviz
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                os.path.join( package_dir, "rviz", "mapping.rviz" )
            ],
            # Only run according to the param
            condition=IfCondition(
                PythonExpression([ run_rviz ])
            )
        )
    );

    # Conditionally run Coppelia
    ld.add_action(
        ExecuteProcess(
            cmd= [ os.path.join( coppelia_path, "coppeliaSim" ), "-f", os.path.join( package_dir, "coppelia", "whisker_scene.ttt" ) ],
            # Only run according to the param
            condition=IfCondition(
                PythonExpression([ launch_coppelia ])
            )
        )
    );

    # Conditionally run the reactive controller
    ld.add_action(
        Node(
            package="whisker_utils",
            executable="reactive_controller",
            # Only run according to the param
            condition=IfCondition(
                PythonExpression([ run_simple_controller ])
            )
        )
    );

    # Return our launch description we've generated
    return ld;

