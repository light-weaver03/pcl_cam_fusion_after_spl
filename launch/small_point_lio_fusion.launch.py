from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    run_lio = LaunchConfiguration("run_lio")

    declare_run_lio = DeclareLaunchArgument(
        "run_lio", default_value="true",
        description="Whether to launch small_point_lio and static TF"
    )
    small_point_lio_node = Node(
        package="small_point_lio",
        executable="small_point_lio_node",
        name="small_point_lio",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("small_point_lio"),
                    "config",
                    "mid360.yaml",
                ]
            )
        ],
        condition=IfCondition(run_lio),
    )

    static_base_link_to_livox_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.2",
            "--roll",
            "-0.785398", 
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "livox_frame",
        ],
        condition=IfCondition(run_lio),
        # arguments=[
        #     "--x",
        #     "0.0",
        #     "--y",
        #     "0.0",
        #     "--z",
        #     "0.0",
        #     "--roll",
        #     "3.14159265", 
        #     "--pitch",
        #     "0.0",
        #     "--yaw",
        #     "0.0",
        #     "--frame-id",
        #     "base_link",
        #     "--child-frame-id",
        #     "livox_frame",
        # ],
    )


    picture_filter_node = Node(
        package="small_point_lio",
        executable="picture_filter_node",
        name="picture_filter",
        output="screen",
        parameters=[{
            "image_in": "/image_raw",
            "image_out": "/image_nearest",
            "max_dt": 0.3,
            "buffer_size": 200,
        }],
    )

    cloud_to_lidar_colorize_node = Node(
        package="small_point_lio",
        executable="cloud_to_lidar_colorize_node",
        name="cloud_to_lidar_colorize",
        output="screen",
        parameters=[{
            "image_topic": "/image_nearest",
            "lidar_frame": "livox_frame",
            # 用包内 config 路径更稳（不要用 ../ 相对路径）
            "calib_file": PathJoinSubstitution(
                [FindPackageShare("small_point_lio"), "config", "single_calib_result.txt"]
            ),
            "interp": "bilinear",
            "tf_timeout": 0.2,
            # 相机内参/畸变（可按需放这里）
            "fx":  17389.74446324433, "fy": 17137.88266951942,
            "cx": 1009.440472967610, "cy": 1538.804619553634,
            "k1": 2.076393990914652, "k2": -149.3987088187306, "p1": 0.0, "p2": 0.0,
        }],
    )


    return LaunchDescription([
    	declare_run_lio,
    	small_point_lio_node, 
    	static_base_link_to_livox_frame,
    	picture_filter_node,
    	cloud_to_lidar_colorize_node,
    ])
