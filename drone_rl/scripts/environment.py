#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty
from drone_msgs.msg import PropellerVelocity
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
import numpy as np

from utils import calculate_distance_cost

class Environment:

    def __init__(self, sub_topic, pub_topic):
        rospy.init_node("RL", anonymous=True)

        rospy.Subscriber(sub_topic, ModelStates, self.callback)
        rospy.loginfo(f"Subscribed to {sub_topic}")

        self.pub = rospy.Publisher(pub_topic, PropellerVelocity, queue_size=10)
        rospy.loginfo(f"Publishing to {pub_topic}")

        self.drone_name = "my_robot"

        self.prev_state = np.zeros(12)
        self.current_state = np.zeros(12)

    def reset_environment(self):

        rospy.wait_for_service('/gazebo/reset_world')
        try:
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_world()
            rospy.loginfo("Gazebo world has been reset.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def callback(self, msg):
        try:
            index = msg.name.index(self.drone_name)

            pos = msg.pose[index].position
            self.current_state[0:3] = [pos.x, pos.y, pos.z]

            ori = msg.pose[index].orientation
            roll, pitch, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
            self.current_state[6:9] = [roll, pitch, yaw]

            twist = msg.twist[index]
            self.current_state[3:6] = [twist.linear.x, twist.linear.y, twist.linear.z]
            self.current_state[9:12] = [twist.angular.x, twist.angular.y, twist.angular.z]

            # rospy.loginfo(f"Drone State: {self.current_state}")

        except ValueError:
            rospy.logwarn("Drone not found in model states")

    def step(self, prop_speeds: np.ndarray):

        if prop_speeds.shape != (4,):
            rospy.logerr("Input array must be of size 4")
            return
        
        prop_speeds *= 1000
        prop_speeds += 1000

        msg = PropellerVelocity()
        msg.prop1 = int(prop_speeds[0])
        msg.prop2 = int(prop_speeds[1])
        msg.prop3 = int(prop_speeds[2])
        msg.prop4 = int(prop_speeds[3])
        self.pub.publish(msg)

        reward = calculate_distance_cost(self.goal_state, self.current_state) - calculate_distance_cost(self.goal_state, self.prev_state)
        self.prev_state = self.current_state
        
        done = False
        # rospy.loginfo(f"Published propeller speeds: {prop_speeds}")

        if self.current_state[2] < 0.01:
            done = True
        print(self.current_state[2])

        return reward, done

    def get_state(self):
        print(self.current_state.shape)
        return self.current_state
    
    def set_goal_state(self, goal):
        self.goal_state = goal
