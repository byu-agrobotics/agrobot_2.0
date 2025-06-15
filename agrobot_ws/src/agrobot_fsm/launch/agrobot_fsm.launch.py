import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='agrobot_fsm',
            executable='collect_fsm',
        ),
        launch_ros.actions.Node(
            package='agrobot_fsm',
            executable='handle_fsm',
        ),
        launch_ros.actions.Node(
            package='agrobot_fsm',
            executable='navigate_fsm',
        ),
        launch_ros.actions.Node(
            package='agrobot_fsm',
            executable='sort_fsm',
        ),
    ])