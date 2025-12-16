#!/usr/bin/env python3

"""
Humanoid Locomotion Reinforcement Learning

This script implements reinforcement learning for humanoid locomotion
using Isaac Gym or Isaac Sim environments.
"""

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.utils.tensorboard import SummaryWriter
import gym
from gym import spaces
import time
import os
from collections import deque
import argparse


class HumanoidLocomotionEnv:
    """
    Simplified Humanoid Locomotion Environment
    This is a placeholder that would be replaced with Isaac Sim integration in practice
    """
    def __init__(self, num_dofs=16):
        super().__init__()

        # Define action and observation spaces
        self.num_dofs = num_dofs
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(self.num_dofs,), dtype=np.float32
        )

        # Observation space: joint positions, velocities, body orientation, target velocity
        obs_dim = 3 * self.num_dofs + 13  # 3*n_dof (pos, vel, eff) + 13 (body info)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )

        # Target velocity for forward locomotion
        self.target_velocity = 1.0  # m/s

        # Initialize state
        self.reset()

    def reset(self):
        """Reset the environment to initial state"""
        # Initialize joint positions near zero
        self.joint_positions = np.random.uniform(-0.1, 0.1, self.num_dofs)
        self.joint_velocities = np.zeros(self.num_dofs)
        self.joint_efforts = np.zeros(self.num_dofs)

        # Initialize body state (position, orientation, linear/angular velocity)
        self.body_pos = np.array([0.0, 0.0, 0.8])  # Start at 0.8m height
        self.body_orn = np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion (x,y,z,w)
        self.body_lin_vel = np.array([0.0, 0.0, 0.0])
        self.body_ang_vel = np.array([0.0, 0.0, 0.0])

        # Initialize other state variables
        self.time_step = 0
        self.episode_length = 1000  # 1000 steps = 10 seconds at 100Hz

        return self.get_observation()

    def step(self, action):
        """Execute one step in the environment"""
        # Apply action (simplified physics simulation)
        clipped_action = np.clip(action, -1.0, 1.0)

        # Update joint positions (simplified integration)
        self.joint_velocities += clipped_action * 0.1  # Apply torque-like action
        self.joint_positions += self.joint_velocities * 0.01  # Integrate velocity

        # Keep joint positions reasonable
        self.joint_positions = np.clip(self.joint_positions, -2.0, 2.0)

        # Simplified body dynamics update
        self.body_lin_vel[0] += 0.01 * clipped_action[0]  # Move forward
        self.body_pos[0] += self.body_lin_vel[0] * 0.01  # Integrate position

        # Update other state variables
        self.time_step += 1

        # Calculate reward
        reward = self.calculate_reward()

        # Check termination conditions
        done = self.check_termination()

        # Additional info (for debugging)
        info = {
            'time_step': self.time_step,
            'x_position': self.body_pos[0],
            'x_velocity': self.body_lin_vel[0]
        }

        return self.get_observation(), reward, done, info

    def get_observation(self):
        """Get current observation from environment"""
        obs = np.concatenate([
            self.joint_positions,
            self.joint_velocities,
            self.joint_efforts,
            self.body_pos,
            self.body_orn,
            self.body_lin_vel,
            self.body_ang_vel,
            [self.target_velocity]
        ])
        return obs

    def calculate_reward(self):
        """Calculate reward based on current state"""
        # Reward for forward velocity matching target
        forward_vel_reward = -abs(self.body_lin_vel[0] - self.target_velocity)

        # Penalty for falling (body height too low)
        height_penalty = max(0, 0.5 - self.body_pos[2]) * -10

        # Penalty for excessive joint velocities
        velocity_penalty = -np.sum(np.square(self.joint_velocities)) * 0.01

        # Penalty for excessive joint torques (actions)
        action_penalty = -np.sum(np.square(self.joint_efforts)) * 0.001

        # Small alive bonus to encourage longer episodes
        alive_bonus = 0.1

        total_reward = forward_vel_reward + height_penalty + velocity_penalty + action_penalty + alive_bonus
        return total_reward

    def check_termination(self):
        """Check if episode should terminate"""
        # Terminate if robot falls (too low)
        if self.body_pos[2] < 0.3:
            return True

        # Terminate if joint limits exceeded
        if np.any(np.abs(self.joint_positions) > 3.0):
            return True

        # Terminate if episode length exceeded
        if self.time_step >= self.episode_length:
            return True

        return False


class ActorCritic(nn.Module):
    """Actor-Critic neural network for humanoid locomotion"""
    def __init__(self, state_dim, action_dim, hidden_dim=512):
        super(ActorCritic, self).__init__()

        # Actor network (policy)
        self.actor = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Tanh()
        )

        # Critic network (value function)
        self.critic = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )

    def forward(self, state):
        action_mean = self.actor(state)
        value = self.critic(state)
        return action_mean, value

    def get_action(self, state):
        """Get action from policy with added noise for exploration"""
        action_mean, value = self.forward(state)

        # For continuous action spaces, we can add Gaussian noise
        action_std = torch.ones_like(action_mean) * 0.1  # Exploration noise
        dist = torch.distributions.Normal(action_mean, action_std)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(dim=-1, keepdim=True)

        return action, log_prob, value


class PPOAgent:
    """Proximal Policy Optimization (PPO) Agent for humanoid locomotion"""
    def __init__(self, state_dim, action_dim, lr=3e-4, gamma=0.99, eps_clip=0.2,
                 K_epochs=8, hidden_dim=512):
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.K_epochs = K_epochs

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.policy = ActorCritic(state_dim, action_dim, hidden_dim).to(self.device)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=lr)

        self.policy_old = ActorCritic(state_dim, action_dim, hidden_dim).to(self.device)
        self.policy_old.load_state_dict(self.policy.state_dict())

        self.MseLoss = nn.MSELoss()

    def update(self, memory):
        """Update policy using PPO"""
        # Monte Carlo estimate of state rewards
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(memory.rewards), reversed(memory.is_terminals)):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)

        # Normalizing the rewards
        rewards = torch.tensor(rewards, dtype=torch.float32).to(self.device)
        rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-5)

        # Convert list to tensor
        old_states = torch.stack(memory.states).to(self.device).detach()
        old_actions = torch.stack(memory.actions).to(self.device).detach()
        old_logprobs = torch.stack(memory.logprobs).to(self.device).detach()

        # Optimize policy for K epochs
        for _ in range(self.K_epochs):
            # Evaluating old actions and values
            logprobs, state_values = self.policy_old.get_action(old_states)

            # Finding the ratio (pi_theta / pi_theta__old)
            ratios = torch.exp(logprobs - old_logprobs.detach())

            # Finding Surrogate Loss
            advantages = rewards - state_values.detach()
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1 - self.eps_clip, 1 + self.eps_clip) * advantages
            loss = -torch.min(surr1, surr2) + 0.5 * self.MseLoss(state_values, rewards)

            # Take gradient step
            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()

        # Copy new weights into old policy
        self.policy_old.load_state_dict(self.policy.state_dict())


class Memory:
    """Memory to store transitions for RL training"""
    def __init__(self):
        self.actions = []
        self.states = []
        self.logprobs = []
        self.rewards = []
        self.is_terminals = []

    def clear_memory(self):
        """Clear memory after updating policy"""
        del self.actions[:]
        del self.states[:]
        del self.logprobs[:]
        del self.rewards[:]
        del self.is_terminals[:]


def train_humanoid_locomotion():
    """Train humanoid locomotion using PPO"""
    # Create environment
    env = HumanoidLocomotionEnv()
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]

    # Create agent
    agent = PPOAgent(state_dim, action_dim)

    # Create memory
    memory = Memory()

    # Create summary writer for TensorBoard
    writer = SummaryWriter(log_dir='runs/humanoid_locomotion_ppo')

    # Training parameters
    max_episodes = 5000
    max_timesteps = 1000
    update_timestep = 2000  # Update policy every n timesteps
    log_interval = 100  # Log average reward every n episodes
    running_reward = 0
    avg_length = 0
    timestep = 0

    print(f"Starting training for humanoid locomotion...")

    for i_episode in range(1, max_episodes+1):
        state = env.reset()
        episode_reward = 0

        for t in range(max_timesteps):
            timestep += 1

            # Running policy_old
            state_tensor = torch.FloatTensor(state).to(agent.device)
            action, log_prob, _ = agent.policy_old.get_action(state_tensor)

            action = action.cpu().data.numpy()

            # Take action in environment
            state, reward, done, _ = env.step(action)

            # Store in memory
            memory.states.append(state_tensor)
            memory.actions.append(torch.FloatTensor(action).to(agent.device))
            memory.logprobs.append(log_prob)
            memory.rewards.append(reward)
            memory.is_terminals.append(done)

            # Update episode reward
            episode_reward += reward

            if done:
                break

        running_reward += episode_reward
        avg_length += t

        # Update policy if enough timesteps have passed
        if timestep % update_timestep == 0:
            agent.update(memory)
            memory.clear_memory()
            timestep = 0

        # Log average reward every log_interval episodes
        if i_episode % log_interval == 0:
            avg_reward = running_reward / log_interval
            avg_length = int(avg_length / log_interval)
            print(f'Episode {i_episode}, Average Reward: {avg_reward:.2f}, Average Length: {avg_length}')

            # Log to TensorBoard
            writer.add_scalar('Reward/Average', avg_reward, i_episode)
            writer.add_scalar('Episode/Length', avg_length, i_episode)

            running_reward = 0
            avg_length = 0

    # Close TensorBoard writer
    writer.close()

    print("Training completed!")

    # Save the trained model
    torch.save(agent.policy.state_dict(), 'humanoid_locomotion_ppo.pth')
    print("Model saved as 'humanoid_locomotion_ppo.pth'")


def test_trained_policy():
    """Test the trained policy"""
    # Create environment
    env = HumanoidLocomotionEnv()
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]

    # Create agent and load trained model
    agent = PPOAgent(state_dim, action_dim)
    agent.policy.load_state_dict(torch.load('humanoid_locomotion_ppo.pth'))
    agent.policy.eval()

    print("Testing trained policy...")

    for episode in range(5):
        state = env.reset()
        episode_reward = 0
        t = 0

        while t < 1000:  # Run for 1000 timesteps
            state_tensor = torch.FloatTensor(state).to(agent.device)
            action, _, _ = agent.policy.get_action(state_tensor)
            action = action.cpu().data.numpy()

            state, reward, done, info = env.step(action)
            episode_reward += reward
            t += 1

            if done:
                break

        print(f'Test Episode {episode+1}: Reward = {episode_reward:.2f}, Steps = {t}, Final X-Pos = {info["x_position"]:.2f}')

    print("Testing completed!")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Humanoid Locomotion RL Training')
    parser.add_argument('--train', action='store_true', help='Train the policy')
    parser.add_argument('--test', action='store_true', help='Test the trained policy')

    args = parser.parse_args()

    if args.train:
        train_humanoid_locomotion()
    elif args.test:
        test_trained_policy()
    else:
        print("Please specify --train or --test")