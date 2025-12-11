from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import yaml
import os


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, "r") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    
    # Get the URDF via xacro
    ur_type = "ur5e"
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]
            ),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            PathJoinSubstitution(
                [FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"]
            ),
            " ",
            "kinematics_params:=",
            PathJoinSubstitution(
                [FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"]
            ),
            " ",
            "physical_params:=",
            PathJoinSubstitution(
                [FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"]
            ),
            " ",
            "visual_params:=",
            PathJoinSubstitution(
                [FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"]
            ),
            " ",
            "name:=",
            "ur",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    
    # Get the SRDF
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]
            ),
            " ",
            "name:=ur",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }
    
    # Get kinematics yaml
    kinematics_yaml = load_yaml("ur_moveit_config", "config/kinematics.yaml")
    
    # Load servo config and wrap it properly
    servo_yaml = load_yaml("my_ur_station", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml["servo_node"]["ros__parameters"]}
    
    # MoveIt Servo Node with robot description and semantic description
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
    )
    
    return LaunchDescription([servo_node])
