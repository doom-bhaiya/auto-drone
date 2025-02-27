#!/usr/bin/env python3
from environment import Environment
import rospy
import numpy as np
import time

SUB_TOPIC = "/gazebo/model_states"
PUB_TOPIC = "cmd_vel_three"

if __name__ == "__main__":

    env = Environment(SUB_TOPIC, PUB_TOPIC)
    env.reset_environment()
    for i in range(100):
        speed = np.array([1600,1600,1600,1600])
        env.publish(speed)
        time.sleep(0.1)