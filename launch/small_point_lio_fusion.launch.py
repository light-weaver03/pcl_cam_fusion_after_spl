from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

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
            "cloud_in": "/cloud_registered",
            "image_out": "/image_nearest",
            "max_dt": 0.05,
            "buffer_size": 200,
        }],
    )

    cloud_to_lidar_colorize_node = Node(
        package="small_point_lio",
        executable="cloud_to_lidar_colorize_node",
        name="cloud_to_lidar_colorize",
        output="screen",
        parameters=[{
            "cloud_topic": "/cloud_registered",
            "image_topic": "/image_nearest",
            "lidar_frame": "livox_frame",
            # 用包内 config 路径更稳（不要用 ../ 相对路径）
            "calib_file": PathJoinSubstitution(
                [FindPackageShare("small_point_lio"), "config", "single_calib_result.txt"]
            ),
            "interp": "bilinear",
            "tf_timeout": 0.2,
            # 相机内参/畸变（可按需放这里）
            "fx": 13800.0, "fy": 13800.0,
            "cx": 743.785, "cy": 566.278,
            "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0,
        }],
    )


    return LaunchDescription([
    	small_point_lio_node, 
    	static_base_link_to_livox_frame,
    	picture_filter_node,
    	cloud_to_lidar_colorize_node,
    ])
