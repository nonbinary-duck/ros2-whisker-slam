import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

#
# Launch file based on the substitutions tutorial: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html
# And my tidybot assignment from autonomous mobile robotics
#

def generate_launch_description():

    # Line of code adapted from the nav2_bringup slam_launch.py file
    package_dir = get_package_share_directory('whisker_utils');

    # Initialise a launch description object to add to
    ld = LaunchDescription();

    # Use different SLAM setup for ground-truth LiDAR and whisker
    use_whisker_params_arg = DeclareLaunchArgument(
        "use_whisker_params",
        default_value="True",
        description="If the parameters for slam_toolbox should use the whisker ones or not"
    );
    ld.add_action(use_whisker_params_arg);

    # Let us use this value
    use_whisker_params = LaunchConfiguration("use_whisker_params");


    # Get the path to coppelia simulate
    coppelia_path = os.getenv("COPPELIASIM_ROOT_DIR");

    # If we're using coppelia, we obviously need to know where it is
    if (coppelia_path == None):
        raise RuntimeError("Cannot launch Coppelia without the environment variable $COPPELIASIM_ROOT_DIR");

    # Run our 4 differential drive controller
    ld.add_action(
        Node(
            package="whisker_utils",
            executable="cmdvel_controller",
        )
    );

    # Launch SLAM (if using whisker)
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
                "slam_params_file": os.path.join( package_dir, "params", "mapper_params_online_async.yaml" )
            }.items(),
            # Use this launcher if we're using the whisker
            condition=IfCondition(
                PythonExpression([ use_whisker_params ])
            )
        )
    );

    # Launch SLAM (if NOT using whisker)
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
                "slam_params_file": os.path.join( package_dir, "params", "real_lidar_mapper_params_online_async.yaml" )
            }.items(),
            # Use this launcher if we're NOT using the whisker
            condition=UnlessCondition(
                PythonExpression([ use_whisker_params ])
            )
        )
    );

    # Run the Nav2 stack
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
            }.items()
        )
    );

    # Run rviz
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                os.path.join( package_dir, "rviz", "mapping.rviz" )
            ]
        )
    );

    # Run Coppelia
    ld.add_action(
        ExecuteProcess(
            cmd= [ os.path.join( coppelia_path, "coppeliaSim" ), "-f", os.path.join( package_dir, "coppelia", "whisker_scene.ttt" ) ]
        )
    );

    # Return our launch description we've generated
    return ld;

