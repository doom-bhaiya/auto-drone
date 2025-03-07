from utils import ddpg
from agent import Agent

from config import GAMMA, NUM_EPISODES, MAX_LENGTH

if __name__ == "__main__":

    sub_topic = "/gazebo/model_states"
    pub_topic = "/cmd_vel_three"

    state_size = 24
    action_size = 4

    agent = Agent(sub_topic, pub_topic, state_size, action_size)
    scores = ddpg(agent, num_episodes=NUM_EPISODES, max_steps=MAX_LENGTH, gamma=GAMMA, noise_decay=0.995)