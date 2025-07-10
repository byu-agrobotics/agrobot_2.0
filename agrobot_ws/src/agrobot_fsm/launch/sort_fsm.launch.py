import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Launch sort_fsm node from agrobot_fsm package
        launch_ros.actions.Node(
            package='agrobot_fsm',
            executable='sort_fsm',
            output='screen'
        ),

        # Launch egg_id node from agrobot_perception package
        launch_ros.actions.Node(
            package='agrobot_perception',
            executable='egg_id',
            output='screen'
        ),
    ])