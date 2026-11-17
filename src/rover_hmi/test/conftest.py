# Keep test nodes off the real ROS network: the PC and the rover share domain 0, so a test node
# there could talk to (or drive) the real rover. Must be set before rclpy.init().
import os

os.environ["ROS_DOMAIN_ID"] = "42"
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
