import math 
import rclpy 
from time import sleep 
from rclpy.node import Node
from std_msgs.msg import String 
from geometry_msgs.msg import Twist     
from sensor_msgs.msg import LaserScan    
from geometry_msgs.msg import Pose 
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data 
import numpy as np 
import argparse

 
class Controller(Node):
  """
  Create a Controller class, which is a subclass of the Node 
  class for ROS2.
  """
  def __init__(self,name):
    """
    Class constructor to set up the node
    """
    ##################### ROS SETUP ####################################################
    # Initiate the Node class's constructor and give it a name
    super().__init__(node_name ='Controller',namespace=name)
 
    # Create a subscriber
    # This node subscribes to messages of type Float64MultiArray  
    # over a topic named: /b_rover/state_est
    # The message represents the current estimated state:
    #   [x, y, yaw]
    # The callback function is called as soon as a message 
    # is received.
    # The maximum number of queued messages is 10.
    self.subscription = self.create_subscription(
                        Float64MultiArray,
                        '/'+name+'/state_est',
                        self.state_estimate_callback,
                        10)
    self.subscription  # prevent unused variable warning
 
    # Create a subscriber
    # This node subscribes to messages of type 
    # sensor_msgs/LaserScan     
    self.scan_subscriber = self.create_subscription(
                           LaserScan,
                           '/'+name+'/gazebo_ros_range/out',
                           self.scan_callback,
                           qos_profile=qos_profile_sensor_data)
                            
    # Create a publisher
    # This node publishes the desired linear and angular velocity of the robot (in the
    # robot chassis coordinate frame) to the /demo/cmd_vel topic. Using the diff_drive
    # plugin enables the robot model to read this /demo/cmd_vel topic and execute
    # the motion accordingly.
    self.publisher_ = self.create_publisher(
                      Twist, 
                      '/'+name+'/cmd_vel', 
                      10)
 
    # Initialize the LaserScan sensor readings to some large value
    # Values are in meters.
    # self.left_dist = 999999.9 # Left
    # self.leftfront_dist = 999999.9 # Left-front
    # self.front_dist = 999999.9 # Front
    # self.rightfront_dist = 999999.9 # Right-front
    # self.right_dist = 999999.9 # Right
    self.obstacle_dist = 99999.9
 
     ################### ROBOT CONTROL PARAMETERS ##################
    # Maximum forward speed of the robot in meters per second
    # Any faster than this and the robot risks falling over.
    self.forward_speed = 0.02
 
    # Current position and orientation of the robot in the global 
    # reference frame
    self.current_x = 0.0
    self.current_y = 0.0
    self.current_yaw = 0.0
 
    ############# WALL FOLLOWING PARAMETERS #######################     
    # Finite states for the wall following mode
    #  "turn left": Robot turns towards the left
    #  "search for wall": Robot tries to locate the wall        
    #  "follow wall": Robot moves parallel to the wall
    self.wall_following_state = "turn left"
         
    # Set turning speeds (to the left) in rad/s 
    # These values were determined by trial and error.
    self.turning_speed_wf_fast = 0.1  # Fast turn
    self.turning_speed_wf_slow = 0.08 # Slow turn
 
    # Wall following distance threshold.
    # We want to try to keep within this distance from the wall.
    self.dist_thresh_wf = 0.6 # in meters  
 
    # We don't want to get too close to the wall though.
    self.dist_too_close_to_wall = 0.4 # in meters
 
  def state_estimate_callback(self, msg):
    """
    Extract the position and orientation data. 
    This callback is called each time
    a new message is received on the '/demo/state_est' topic
    """
    # Update the current estimated state in the global reference frame
    curr_state = msg.data
    self.current_x = curr_state[0]
    self.current_y = curr_state[1]
    self.current_yaw = curr_state[2]
 
    # Command the robot to keep following the wall      
    self.follow_wall()
 
  def scan_callback(self, msg):
    """
    This method gets called every time a LaserScan message is 
    received on the '/demo/laser/out' topic 
    """
    # Read the laser scan data that indicates distances
    # to obstacles (e.g. wall) in meters and extract
    # 5 distinct laser readings to work with.
    # Each reading is separated by 45 degrees.
    # Assumes 181 laser readings, separated by 1 degree. 
    # (e.g. -90 degrees to 90 degrees....0 to 180 degrees)
 
    #number_of_laser_beams = str(len(msg.ranges)) 
    self.obstacle_dist =  min(msg.ranges)     
    # self.left_dist = msg.ranges[29]
    # self.leftfront_dist = msg.ranges[22]
    # self.front_dist = msg.ranges[15]
    # self.rightfront_dist = msg.ranges[7]
    # self.right_dist = msg.ranges[0]

             
  def follow_wall(self):
    """
    This method causes the robot to follow the boundary of a wall.
    """
    # Create a geometry_msgs/Twist message
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0        
 
    # Logic for following the wall
    # >d means no wall detected by that laser beam
    # <d means an wall was detected by that laser beam
    d = self.dist_thresh_wf
    self.state="forward"
     
    if self.obstacle_dist >= d and self.state == "forward" :
      msg.linear.x = self.forward_speed

    elif self.obstacle_dist < d:
      if self.state == "forward":
        self.state = "left"
        msg.angular.z = self.turning_speed_wf_slow

      elif self.state == "left":
        self.state = "forward"
        msg.angular.z = -2*self.turning_speed_wf_slow
 
    else:
      pass

    # Send velocity command to the robot
    self.publisher_.publish(msg)    
 
def main(args=None):
 
    # Initialize rclpy library
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Spawn Robot into Gazebo')
    parser.add_argument('-n', '--robot_name', type=str, default='b_rover',
                        help='Name of the robot to spawn')
    args, unknown = parser.parse_known_args()
     
    # Create the node
    controller = Controller(args.robot_name)
 
    # Spin the node so the callback function is called
    # Pull messages from any topics this node is subscribed to
    # Publish any pending messages to the topics
    rclpy.spin(controller)
 
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
     
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
