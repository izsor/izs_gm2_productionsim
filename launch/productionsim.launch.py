from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rate_sec_arg = DeclareLaunchArgument('rate_sec', default_value='1.0')
    defect_prob_arg = DeclareLaunchArgument('defect_prob', default_value='0.25')
    noise_arg = DeclareLaunchArgument('noise', default_value='0.08')
    diff_threshold_arg = DeclareLaunchArgument('diff_threshold', default_value='1.0')
    log_pairs_arg = DeclareLaunchArgument('log_pairs', default_value='true')

    rate_sec = LaunchConfiguration('rate_sec')
    defect_prob = LaunchConfiguration('defect_prob')
    noise = LaunchConfiguration('noise')
    diff_threshold = LaunchConfiguration('diff_threshold')
    log_pairs = LaunchConfiguration('log_pairs')

    f32 = lambda x: ParameterValue(x, value_type=float)
    bval = lambda x: ParameterValue(x, value_type=bool)

    line = Node(
        package='izs_gm2_productionsim',
        executable='line',
        name='line',
        output='screen',
        parameters=[{
            'rate_sec': f32(rate_sec),
            'defect_prob': f32(defect_prob),
        }],
    )

    device = Node(
        package='izs_gm2_productionsim',
        executable='device',
        name='device',
        output='screen',
        parameters=[{
            'noise': f32(noise),
        }],
    )

    quality = Node(
        package='izs_gm2_productionsim',
        executable='quality',
        name='quality',
        output='screen',
        parameters=[{
            'diff_threshold': f32(diff_threshold),
            'log_pairs': bval(log_pairs),
        }],
    )

    stats = Node(
        package='izs_gm2_productionsim',
        executable='stats',
        name='stats',
        output='screen',
    )

    viz = Node(package='izs_gm2_productionsim', 
    executable='viz', 
    name='viz', 
    output='screen', 
    parameters=[{'frame_id': 'map'}]
    ),

    return LaunchDescription([
        rate_sec_arg, defect_prob_arg, noise_arg, diff_threshold_arg, log_pairs_arg,
        line, device, quality, stats, viz
    ])
