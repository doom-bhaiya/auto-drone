#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty
from drone_msgs.msg import PropellerVelocity
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
import numpy as np

class Environment:

    def __init__(self, sub_topic, pub_topic):
        rospy.init_node("RL", anonymous=True)

        rospy.Subscriber(sub_topic, ModelStates, self.callback)
        rospy.loginfo(f"Subscribed to {sub_topic}")

        self.pub = rospy.Publisher(pub_topic, PropellerVelocity, queue_size=10)
        rospy.loginfo(f"Publishing to {pub_topic}")

    def reset_environment(self):

        rospy.wait_for_service('/gazebo/reset_world')
        try:
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_world()
            rospy.loginfo("Gazebo world has been reset.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def callback(self, msg):
        drone_name = "my_robot"  # Change this to match your drone's name in Gazebo
        try:
            index = msg.name.index(drone_name)  # Get index of the drone
            pos = msg.pose[index].position
            ori = msg.pose[index].orientation
            
            # Convert quaternion to Euler angles
            euler = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
            theta, phi, rho = euler  # Roll, Pitch, Yaw

            rospy.loginfo(f"Drone Position: x={pos.x}, y={pos.y}, z={pos.z}")
            rospy.loginfo(f"Drone Orientation: theta={theta}, phi={phi}, rho={rho}")

        except ValueError:
            rospy.logwarn("Drone not found in model states!")

    def publish(self, prop_speeds: np.ndarray):
        if prop_speeds.shape != (4,):
            rospy.logerr("Input array must be of size 4")
            return
        
        msg = PropellerVelocity()
        msg.prop1 = prop_speeds[0]
        msg.prop2 = prop_speeds[1]
        msg.prop3 = prop_speeds[2]
        msg.prop4 = prop_speeds[3]
        self.pub.publish(msg)
        rospy.loginfo(f"Published propeller speeds: {prop_speeds}")
