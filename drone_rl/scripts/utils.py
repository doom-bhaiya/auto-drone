import numpy as np
import random
from collections import deque

from config import TAU

class ReplayBuffer:
    def __init__(self, capacity):
        """
        Basic Experience Replay Buffer.

        :param capacity: Maximum number of experiences to store.
        """
        self.buffer = deque(maxlen=capacity)

    def add(self, state, action, reward, next_state, done):
        """
        Add a new experience to the buffer.

        :param state: Current state.
        :param action: Action taken.
        :param reward: Reward received.
        :param next_state: Resulting state.
        :param done: Boolean flag if the episode ended.
        """
        self.buffer.append((state, action, reward, next_state, done))

    def sample(self, batch_size):
        """
        Sample a random batch of experiences.

        :param batch_size: Number of experiences to sample.
        :return: Batch of (states, actions, rewards, next_states, dones)
        """
        experiences = random.sample(self.buffer, batch_size)

        states, actions, rewards, next_states, dones = zip(*experiences)
        # return states, actions, rewards, next_states, dones

        return (np.array(states, dtype=np.float32),
                np.array(actions, dtype=np.float32),
                np.array(rewards, dtype=np.float32),
                np.array(next_states, dtype=np.float32),
                np.array(dones, dtype=np.float32))

    def __len__(self):
        """
        Return the current size of the buffer.
        """
        return len(self.buffer)
    

class OUNoise:
    def __init__(self, size, mu=0.0, theta=0.15, sigma=0.2):
        self.mu = mu * np.ones(size)
        self.theta = theta
        self.sigma = sigma
        self.reset()

    def reset(self):
        self.state = np.copy(self.mu)

    def sample(self):
        dx = self.theta * (self.mu - self.state) + self.sigma * np.random.randn(len(self.state))
        self.state += dx
        return self.state


def ddpg(agent, num_episodes=500, max_steps=1000, gamma=0.99, noise_decay=0.995):
    scores = []
    target_location = np.array([[0, 0, 1],
                               [0, 0, 0],
                               [0, 0, 0],
                               [0, 0, 0]])
    agent.set_target(target_location)

    for episode in range(1, num_episodes + 1):
        agent.env.reset_environment()  
        state = agent.env.current_state  
        episode_reward = 0

        for step in range(max_steps):

            input_vector = np.concatenate([state.flatten(), agent.target.flatten()])
            action = agent.act(input_vector)  
            reward, done = agent.step(action)  
            episode_reward += reward

            agent.train(gamma=gamma, Tau = TAU)  

            if not done:
                break

        scores.append(episode_reward / (step + 0.0001))

        print(f"Episode {episode}/{num_episodes}, Score: {episode_reward:.2f}")
        agent.noise.sigma *= noise_decay

    return scores


def calculate_distance_cost(goal_state, current_state, linear_coeff = 1, angular_coeff = 1):

    linear_distance = np.linalg.norm(goal_state[0, :3] - current_state[:3])
    angular_distance = np.linalg.norm(goal_state[2, :3] - current_state[6:9])
    cost = linear_coeff * linear_distance + angular_coeff / linear_distance * angular_distance

    return cost