from modelling import build_deep_actor, build_deep_critic
import tensorflow as tf
from tensorflow.keras.optimizers import Adam
import numpy as np

from config import BUFFER_SIZE, BATCH_SIZE, LR_ACTOR, LR_CRITIC, TAU

from utils import OUNoise, ReplayBuffer

from environment import Environment

class Agent:

    def __init__(self, sub_topic, pub_topic, state_size, action_size):

        self.env = Environment(sub_topic, pub_topic)

        self.prev_state = self.env.get_state()
        self.current_state = self.env.get_state()

        self.actor = build_deep_actor(state_size, action_size)
        self.critic = build_deep_critic(state_size, action_size)

        self.actor_target = build_deep_actor(state_size, action_size)
        self.critic_target = build_deep_critic(state_size, action_size)

        self.actor_target.set_weights(self.actor.get_weights())
        self.critic_target.set_weights(self.critic.get_weights())

        self.actor_optimizer = Adam(learning_rate = LR_ACTOR)
        self.critic_optimizer = Adam(learning_rate = LR_CRITIC)

        self.memory = ReplayBuffer(capacity=BUFFER_SIZE)

        self.noise = OUNoise(action_size)


    def set_target(self, target):

        self.target = target
        self.env.set_goal_state(target)

    def update_target_networks(self, tau=TAU):

        for target_weights, source_weights in zip(self.actor_target.weights, self.actor.weights):
            target_weights.assign(tau * source_weights + (1.0 - tau) * target_weights)

        for target_weights, source_weights in zip(self.critic_target.weights, self.critic.weights):
            target_weights.assign(tau * source_weights + (1.0 - tau) * target_weights)

    def act(self, state):

        state = np.expand_dims(state, axis=0)
        action = self.actor.predict(state)[0]
        action += self.noise.sample()
        return np.clip(action, 0, 1)
    
    def train(self, gamma, Tau):

        if len(self.memory) < BATCH_SIZE:
            return

        states, actions, rewards, next_states, dones = self.memory.sample(batch_size = BATCH_SIZE)

        target_actions = self.actor_target.predict(next_states)
        target_q_values = self.critic_target.predict([next_states, target_actions])

        targets = rewards + gamma * target_q_values

        # Train critic
        with tf.GradientTape() as tape:
            q_values = self.critic([states, actions])
            critic_loss = tf.reduce_mean(tf.square(targets - q_values))

        critic_grads = tape.gradient(critic_loss, self.critic.trainable_variables)
        self.critic_optimizer.apply_gradients(zip(critic_grads, self.critic.trainable_variables))

        # Train actor
        with tf.GradientTape() as tape:
            actions_pred = self.actor(states)
            actor_loss = -tf.reduce_mean(self.critic([states, actions_pred]))

        actor_grads = tape.gradient(actor_loss, self.actor.trainable_variables)
        self.actor_optimizer.apply_gradients(zip(actor_grads, self.actor.trainable_variables))

        self.update_target_networks(tau=Tau)

    def step(self, action):
        """Take action, observe next state, reward, and done from environment."""
        reward, done = self.env.step(action)

        # Save experience in replay buffer
        concatenated_current_state = np.concatenate([self.current_state, self.target.flatten()])
        concatenated_prev_state = np.concatenate([self.prev_state, self.target.flatten()])

        self.memory.add(concatenated_prev_state, action, reward, concatenated_current_state, 1)

        # Update the current and previous state
        self.prev_state = self.current_state

        return reward, 1

