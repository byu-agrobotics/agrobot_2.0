import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # launch_ros.actions.Node(
        #     package='agrobot_navigation',
        #     executable='roboclaw_wrapper',
        # ),
        launch_ros.actions.Node(
            package='agrobot_navigation',
            executable='drive_controller',
            output='screen',
        ),
        # launch_ros.actions.Node(
        #     package='agrobot_navigation',
        #     executable='test_drive_controller',
        #     output='screen'
        # ),
        launch_ros.actions.Node(
            package='agrobot_navigation',
            executable='test_center',
            output='screen'
        ),
        # launch_ros.actions.Node(
        #     package='agrobot_navigation',
        #     executable='test_turn',
        #     output='screen'
        # ),
    ])
