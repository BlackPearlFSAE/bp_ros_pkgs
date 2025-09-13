from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  timestamper = Node(
    package="bp_timestamp",
    executable="sub_timestamper",
    name="bp_stamp",
  )
  return LaunchDescription([
    timestamper
  ])