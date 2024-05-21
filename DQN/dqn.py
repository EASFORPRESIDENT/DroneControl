import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import random
import time
import posix_ipc
import mmap
import struct
import pickle
import matplotlib.pyplot as plt
from IPython import display
import math
import os

# Memory name (should match with C++ code)
memory_name = "/dronePoseAndReset"
memory_send = "/droneAction"
memory_size = 1024
RunLoop = True

#Receive position of drone through shared memory
def sharedMemoryReceive():
    # Open the shared memory
    memory = posix_ipc.SharedMemory(memory_name, flags=posix_ipc.O_RDWR)

    # Map the shared memory into the address space
    mapped_memory = mmap.mmap(memory.fd, memory.size)

    # Read serialized data from the shared memory
    serialized_data = mapped_memory.read()

    # Deserialize the data to extract individual fields
    reset, play, posX, posY, posZ, posYaw, velX, velY, velZ, velYaw = struct.unpack('??dddddddd', serialized_data)

    # Clean up resources when done
    mapped_memory.close()
    memory.close_fd()

    return reset, play, posX, posY, posYaw, posZ, velX, velY, velZ, velYaw

#Send reset through shared memory
def sharedMemorySendReset():

    # Open the shared memory
    memory_s = posix_ipc.SharedMemory(memory_name, flags=posix_ipc.O_RDWR)

    # Map the shared memory into the address space
    mapped_send = mmap.mmap(memory_s.fd, memory_s.size)


    reset_to_send = struct.pack('??dddddddd', True, True, 0,0,getZ(),0,0,0,0,0)
    mapped_send.write(reset_to_send)

    print("Reset Complete")

    # Clean up resources when done
    mapped_send.close()
    memory_s.close_fd()

def sharedMemorySendPlay():

    # Open the shared memory
    memory_s = posix_ipc.SharedMemory(memory_name, flags=posix_ipc.O_RDWR)

    # Map the shared memory into the address space
    mapped_send = mmap.mmap(memory_s.fd, memory_s.size)


    reset_to_send = struct.pack('??dddddddd', False, True, 0,0,getZ(),0,0,0,0,0)
    mapped_send.write(reset_to_send)


    # Clean up resources when done
    mapped_send.close()
    memory_s.close_fd()

#Send chosen action through shared memory
def sharedMemorySend(action):

    # Open the shared memory
    memory_s = posix_ipc.SharedMemory(memory_send, flags=posix_ipc.O_RDWR)

    # Map the shared memory into the address space
    mapped_send = mmap.mmap(memory_s.fd, memory_s.size)

    action_to_send = struct.pack('id?', action, getZ(), RunLoop)
    mapped_send.write(action_to_send)

  # Clean up resources when done
    mapped_send.close()
    memory_s.close_fd()

class DQN(nn.Module):
    def __init__(self, input_size, output_size):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(input_size, 128)
        self.fc2 = nn.Linear(128, 64)
        self.fc3 = nn.Linear(64, 32)
        self.fc4 = nn.Linear(32, output_size)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = torch.relu(self.fc3(x))
        x = self.fc4(x)
        return x
    
# Define some hyperparameters
input_size = 5  # x and y positions
output_size = 5  # Number of possible actions
learning_rate = 0.1
gamma = 0.95  # Discount factor
epsilon = 1.0  # Exploration rate
epsilon_decay = 0.9999
min_epsilon = 0.01
memory_size = 1000000  # Replay memory size
batch_size = 128

# Max distance of drone from target
max_distance = 2
max_steps = 200


# Define the DQN agent
class DQNAgent:
    def __init__(self):
        self.model = DQN(input_size, output_size)
        self.target_model = DQN(input_size, output_size)
        self.target_model.load_state_dict(self.model.state_dict())
        self.memory = []
        self.optimizer = optim.Adam(self.model.parameters(), lr=learning_rate)

    def select_action(self, state):
        #if np.random.rand() < 2:
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

    #def save_memory(self, filename):
     #   with open(filename, 'wb') as f:
       #     pickle.dump(self.memory, f)
       # print("Training data saved to", filename)

    #def load_memory(self, filename):
    #    try:
     #       with open(filename, 'rb') as f:
     #       DQNAgent    self.memory = pickle.load(f)
     #       print("Training data loaded from", filename)
     #   except FileNotFoundError:
     #
     #        print("No existing training data file found.")
    def save(self, filename):
        with open(filename + '_memory.pkl', 'wb') as f:
            pickle.dump(self.memory, f)
        torch.save({
            'model_state_dict': self.model.state_dict(),
            'target_model_state_dict': self.target_model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'epsilon': epsilon,
        }, filename + '_model.pth')
        print("Training data, model, and optimizer state saved to", filename)

    def load(self, filename):
        try:
            with open(filename + '_memory.pkl', 'rb') as f:
                self.memory = pickle.load(f)
            checkpoint = torch.load(filename + '_model.pth')
            self.model.load_state_dict(checkpoint['model_state_dict'])
            self.target_model.load_state_dict(checkpoint['target_model_state_dict'])
            self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
            global epsilon
            epsilon = checkpoint['epsilon']
            print("Training data, model, and optimizer state loaded from", filename)
        except FileNotFoundError:
            print("No existing training data file found.")


# Example usage
agent = DQNAgent()

class Environment:
    def __init__(self):
        self.state_space = 5
        self.action_space = 5
        X_pos = 0
        Y_pos = 0


    def reset(self):
        print("Reseting...")
        reset = 1

        sharedMemorySendReset()
        sharedMemorySend(0)

        while reset:
            reset, play, X_pos, Y_pos, posYaw , Z_pos, X_vel, Y_vel, Z_vel, Yaw_vel= sharedMemoryReceive()
            time.sleep(0.01)
        return X_pos, Y_pos, posYaw, X_vel, Y_vel



    def step(self, action, steps):

        #receive from open memory
        done, play, X_pos, Y_pos, posYaw, Z_pos ,X_vel, Y_vel, Z_vel, Yaw_vel= sharedMemoryReceive()
        while play:
            done, play, X_pos, Y_pos, posYaw, Z_pos ,X_vel, Y_vel, Z_vel, Yaw_vel= sharedMemoryReceive()
            time.sleep(0.01)


        #print(done,X_pos,Y_pos)


        next_state = X_pos, Y_pos, posYaw, X_vel, Y_vel

        distance_to_target = np.sqrt(X_pos**2+Y_pos**2)

        a = 4
        reward = 10 * math.exp(-a * distance_to_target)
        if distance_to_target >= max_distance:
            reward = -10

        #reward =  max(0, 1 - distance_to_target / max_distance)

        #reward_two = max(0,1 - abs((10-posZ))/5)

        #reward = 0.7*reward_one #+ 0.3*reward_two

        if distance_to_target >= max_distance or steps >= max_steps:
            done =  1 
        else:
            done = 0
        return next_state, reward, done, {}

def getZ():
    done, play, X_pos, Y_pos, posYaw, Z_pos, X_vel, Y_vel, Z_vel, Yaw_vel= sharedMemoryReceive()

    return Z_pos

env = Environment()
agent = DQNAgent()
print("Booting AI model...")
#agent.load_memory("training_data.pkl") #tempmem
#agent = DQNAgent() #tempmem
agent.load("/home/andreas/DroneControl/training_data")  # Load training data if exists tempmem
# Check if the model is loaded successfully
print("Model loaded successfully.")

# Print some information about the loaded model
print("Model architecture:")
print(agent.model)  # Print the model architecture
print("Model parameters:")
for name, param in agent.model.named_parameters():
    print(name, param.size())  # Print the name and size of each parameter

save_interval = 100 #tempmem
# Assuming you have an environment with x, y, and z positions
episode_rewards = [] #defining list for ploting
num_episodes = 1000
#for episode in range(num_episodes):
episode = 0
while True:
    nm_of_steps = 0
    state = env.reset()
    total_reward = 0
    done = False

    while not done:
        action = agent.select_action(state)
        #print(action)


        #send action to MAVsdk script through open memory
        sharedMemorySend(action)

        next_state, reward, done, _ = env.step(action,nm_of_steps)

        print(state)
        print(next_state)
        agent.remember(state, action, reward, next_state, done)
        state = next_state
        total_reward += reward
        agent.replay()
        nm_of_steps = nm_of_steps + 1

        sharedMemorySendPlay()
    # Append total reward for this episode to episode_rewards list for ploting
    episode_rewards.append(total_reward)

    agent.update_target_network()
    epsilon = max(min_epsilon, epsilon * epsilon_decay)
    print(f"Episode: {episode+1}, Total Reward: {total_reward}")
    #agent.save_memory("training_data.pkl")#tempmem
    if (episode + 1) % save_interval == 0:
        agent.save("/home/andreas/DroneControl/training_data")
    episode += 1

    
    if episode > 1000:
        episode = 0
        epsilon = 1
    # Plotting
    if episode % 2 == 0:

        plt.figure(1)
        training_reward = torch.tensor(episode_rewards, dtype=torch.float)
        if True:
            plt.title('Result')
        else:
            plt.clf()
            plt.title('Training...')
        plt.xlabel('Episode')
        plt.ylabel('Reward')
        plt.plot(training_reward.numpy())


        # Take 100 episode averages and plot them too
        #if len(durations_t) >= 100:
            #means = durations_t.unfold(0, 100, 1).mean(1).view(-1)
            #means = torch.cat((torch.zeros(99), means))
            #plt.plot(means.numpy())

        plt.pause(0.002)  # pause a bit so that plots are updated

        display.display(plt.gcf())



RunLoop = False
sharedMemorySend(0)
