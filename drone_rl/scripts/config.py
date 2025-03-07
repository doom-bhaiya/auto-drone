BUFFER_SIZE = 1440
BATCH_SIZE = 128

LR_ACTOR = 1e-4
LR_CRITIC = 1e-4

TAU = 0.001

GAMMA = 0.99

NUM_EPISODES = 10
MAX_LENGTH = 10















# In ddpg functions instead of resetting the agent, we are resetting only the environment, we need to reset the agents prev position as well
# Need to properly tune the reward functions
# Need to include more constraints on the reward functions
# Need to add discounted rewards