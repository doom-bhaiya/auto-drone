import tensorflow as tf
from tensorflow.keras import layers, models

def build_deep_actor(state_size, action_size):
    """Deep Actor with progressive skip connections and shrinking layer sizes."""

    inputs = layers.Input(shape=(state_size,))

    x1 = layers.Dense(256, activation='relu')(inputs)
    x1 = layers.Dense(256, activation='relu')(x1)

    x2 = layers.Dense(128, activation='relu')(x1)
    x2 = layers.Dense(128, activation='relu')(x2)
    x2 = layers.Add()([x2, layers.Dense(128)(x1)])

    x3 = layers.Dense(64, activation='relu')(x2)
    x3 = layers.Dense(64, activation='relu')(x3)
    x3 = layers.Add()([x3, layers.Dense(64)(x2)])

    x4 = layers.Dense(32, activation='relu')(x3)
    x4 = layers.Dense(32, activation='relu')(x4)
    x4 = layers.Add()([x4, layers.Dense(32)(x3)])

    x5 = layers.Dense(16, activation='relu')(x4)
    x5 = layers.Dense(16, activation='relu')(x5)
    x5 = layers.Add()([x5, layers.Dense(16)(x4)])

    outputs = layers.Dense(action_size, activation='sigmoid')(x5)

    return models.Model(inputs, outputs)


def build_deep_critic(state_size, action_size):
    """Deep Critic with progressive skip connections and shrinking layer sizes."""

    state_input = layers.Input(shape=(state_size,))
    action_input = layers.Input(shape=(action_size,))

    x = layers.Concatenate()([state_input, action_input])

    x1 = layers.Dense(256, activation='relu')(x)
    x1 = layers.Dense(256, activation='relu')(x1)

    x2 = layers.Dense(128, activation='relu')(x1)
    x2 = layers.Dense(128, activation='relu')(x2)
    x2 = layers.Add()([x2, layers.Dense(128)(x1)])

    x3 = layers.Dense(64, activation='relu')(x2)
    x3 = layers.Dense(64, activation='relu')(x3)
    x3 = layers.Add()([x3, layers.Dense(64)(x2)])

    x4 = layers.Dense(32, activation='relu')(x3)
    x4 = layers.Dense(32, activation='relu')(x4)
    x4 = layers.Add()([x4, layers.Dense(32)(x3)])

    x5 = layers.Dense(16, activation='relu')(x4)
    x5 = layers.Dense(16, activation='relu')(x5)
    x5 = layers.Add()([x5, layers.Dense(16)(x4)])

    outputs = layers.Dense(1)(x5)

    return models.Model([state_input, action_input], outputs)

if __name__ == "__main__":
    
    state_size = 24
    action_size = 4

    actor = build_deep_actor(state_size, action_size)
    critic = build_deep_critic(state_size, action_size)

    actor.summary()
    critic.summary()