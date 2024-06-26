import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import random
import math
import posix_ipc
import mmap
import struct


# Memory name (should match with C++ code)
memory_name = "/dronePosAndReset"
memory_send = "/droneAction"
memory_size = 1024

#Receive position of drone through shared memory
def sharedMemoryReceive():
    # Open the shared memory
    memory = posix_ipc.SharedMemory(memory_name, flags=posix_ipc.O_RDWR)

    # Map the shared memory into the address space
    mapped_memory = mmap.mmap(memory.fd, memory.size)

    # Read serialized data from the shared memory
    serialized_data = mapped_memory.read()

    # Deserialize the data to extract individual fields
    reset, posX, posY, posZ, posYaw = struct.unpack('?dddd', serialized_data)

    # Clean up resources when done
    mapped_memory.close()
    memory.close_fd()

    return reset, posX, posY


#Send chosen action through shared memory
def sharedMemorySend(action):

    # Open the shared memory

    memory_s = posix_ipc.SharedMemory(memory_send, posix_ipc.O_CREAT, size=memory_size)

    # Map the shared memory into the address space

    mapped_send = mmap.mmap(memory_s.fd, memory_s.size)


    action_to_send = struct.pack('i', action)

    mapped_send.write(action_to_send)

  # Clean up resources when done
    mapped_send.close()
    memory_s.close_fd()



class DQN(nn.Module):
    def __init__(self, input_size, output_size):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(input_size, 128)
        self.fc2 = nn.Linear(128, 64)
        self.fc3 = nn.Linear(64, output_size)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)
        return x

# Define some hyperparameters
input_size = 2  # x and y positions
output_size = 5  # Number of possible actions
learning_rate = 0.001
gamma = 0.99  # Discount factor
epsilon = 1.0  # Exploration rate
epsilon_decay = 0.995
min_epsilon = 0.01
memory_size = 10000  # Replay memory size
batch_size = 64

# Max distance of drone from target
max_distance = 2
max_steps = 500


# Define the DQN agent
class DQNAgent:
    def __init__(self):
        self.model = DQN(input_size, output_size)
        self.target_model = DQN(input_size, output_size)
        self.target_model.load_state_dict(self.model.state_dict())
        self.memory = []
        self.optimizer = optim.Adam(self.model.parameters(), lr=learning_rate)

    def select_action(self, state):
        if np.random.rand() < epsilon:
            return np.random.choice(output_size)
        with torch.no_grad():
            q_values = self.model(torch.tensor(state, dtype=torch.float32))
            return torch.argmax(q_values).item()

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
        if len(self.memory) > memory_size:
            del self.memory[0]

    def replay(self):
        if len(self.memory) < batch_size:
            return
        batch = random.sample(self.memory, batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)

        states = torch.tensor(states, dtype=torch.float32)
        actions = torch.tensor(actions, dtype=torch.int64)
        rewards = torch.tensor(rewards, dtype=torch.float32)
        next_states = torch.tensor(next_states, dtype=torch.float32)
        dones = torch.tensor(dones, dtype=torch.float32)

        q_values = self.model(states)
        next_q_values = self.target_model(next_states)

        target_q_values = rewards + (1 - dones) * gamma * torch.max(next_q_values, dim=1)[0]

        loss = nn.MSELoss()(q_values.gather(1, actions.unsqueeze(1)), target_q_values.unsqueeze(1))

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

    def update_target_network(self):
        self.target_model.load_state_dict(self.model.state_dict())

# Example usage
agent = DQNAgent()

class Environment:
    def __init__(self):
        self.state_space = 2
        self.action_space = 5
        X_pos = 0
        Y_pos = 0


    def reset(self):
        reset = 1
        while reset:
            reset, X_pos, Y_pos = sharedMemoryReceive()
        
        return X_pos, Y_pos



    def step(self, action, steps):

        #receive from open memory

        done, X_pos, Y_pos = sharedMemoryReceive()

        print(done,X_pos,Y_pos)

        next_state = X_pos, Y_pos

        distance_to_target = np.sqrt(X_pos**2+Y_pos**2)

        reward =  max(0, 1 - distance_to_target / max_distance)  

        if distance_to_target >= max_distance or steps >= max_steps:
            done =  1 
        else:
            done = 0
        return next_state, reward, done, {}

env = Environment()

# Assuming you have an environment with x, y, and z positions
num_episodes = 1000
for episode in range(num_episodes):
    nm_of_steps = 0
    state = env.reset()
    total_reward = 0
    done = False

    while not done:
        action = agent.select_action(state)
        print(action)


        #send action to MAVsdk script through open memory
        sharedMemorySend(action)

        next_state, reward, done, _ = env.step(action,nm_of_steps)

        agent.remember(state, action, reward, next_state, done)
        state = next_state
        total_reward += reward
        agent.replay()
        nm_of_steps = nm_of_steps + 1

    agent.update_target_network()
    epsilon = max(min_epsilon, epsilon * epsilon_decay)
    print(f"Episode: {episode+1}, Total Reward: {total_reward}")

