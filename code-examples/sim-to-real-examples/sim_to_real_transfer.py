#!/usr/bin/env python3

"""
Sim-to-Real Transfer Utilities

This script implements various techniques for transferring policies
trained in simulation to real-world robotic systems.
"""

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from collections import deque
import pickle
import os
from typing import List, Tuple, Dict, Any
import cv2


class DomainRandomization:
    """
    Domain Randomization utilities for making sim-to-real transfer more robust
    """
    def __init__(self, param_ranges: Dict[str, Tuple[float, float]]):
        """
        Initialize domain randomization with parameter ranges

        Args:
            param_ranges: Dictionary mapping parameter names to (min, max) ranges
        """
        self.param_ranges = param_ranges
        self.current_params = {}

    def randomize_parameters(self) -> Dict[str, float]:
        """
        Generate randomized parameters within specified ranges

        Returns:
            Dictionary of randomized parameters
        """
        randomized_params = {}
        for param_name, (min_val, max_val) in self.param_ranges.items():
            randomized_params[param_name] = np.random.uniform(min_val, max_val)

        self.current_params = randomized_params
        return randomized_params

    def get_current_parameters(self) -> Dict[str, float]:
        """Get the current randomized parameters"""
        return self.current_params


class DynamicsRandomization:
    """
    Dynamics Randomization for physics parameters
    """
    def __init__(self, base_mass: float = 1.0, base_friction: float = 0.5):
        self.base_mass = base_mass
        self.base_friction = base_friction

        # Define ranges for randomization
        self.mass_range = (0.8 * base_mass, 1.2 * base_mass)
        self.friction_range = (0.7 * base_friction, 1.3 * base_friction)
        self.damping_range = (0.8, 1.2)

    def randomize_dynamics(self) -> Dict[str, float]:
        """Randomize dynamics parameters"""
        return {
            'mass': np.random.uniform(*self.mass_range),
            'friction': np.random.uniform(*self.friction_range),
            'damping_factor': np.random.uniform(*self.damping_range)
        }


class VisualDomainRandomization:
    """
    Visual Domain Randomization for camera images
    """
    def __init__(self):
        pass

    def apply_randomization(self, image: np.ndarray) -> np.ndarray:
        """
        Apply visual domain randomization to an image

        Args:
            image: Input image as numpy array

        Returns:
            Randomized image
        """
        # Randomize brightness
        brightness_factor = np.random.uniform(0.7, 1.3)
        image = cv2.convertScaleAbs(image, alpha=brightness_factor, beta=0)

        # Randomize contrast
        contrast_factor = np.random.uniform(0.8, 1.2)
        image = cv2.convertScaleAbs(image, alpha=contrast_factor, beta=0)

        # Randomize saturation (convert to HSV and modify S channel)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        saturation_factor = np.random.uniform(0.8, 1.2)
        hsv[:, :, 1] = np.clip(hsv[:, :, 1] * saturation_factor, 0, 255)
        image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        # Add random noise
        noise_factor = np.random.uniform(0, 10)
        noise = np.random.normal(0, noise_factor, image.shape).astype(np.int16)
        image = np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        return image


class AdaptationNetwork(nn.Module):
    """
    Neural network for adaptation from sim to real
    """
    def __init__(self, input_dim: int, output_dim: int, hidden_dim: int = 256):
        super(AdaptationNetwork, self).__init__()

        self.network = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, output_dim)
        )

    def forward(self, x):
        return self.network(x)


class SystemIDAdaptation:
    """
    System Identification based adaptation
    """
    def __init__(self, state_dim: int, action_dim: int):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.dynamics_model = None

        # Buffers for storing data
        self.state_buffer = deque(maxlen=10000)
        self.action_buffer = deque(maxlen=10000)
        self.next_state_buffer = deque(maxlen=10000)

    def collect_data(self, state: np.ndarray, action: np.ndarray, next_state: np.ndarray):
        """Collect transition data for system identification"""
        self.state_buffer.append(state)
        self.action_buffer.append(action)
        self.next_state_buffer.append(next_state)

    def train_dynamics_model(self):
        """Train a dynamics model using collected data"""
        if len(self.state_buffer) < 100:
            return False

        # Convert buffers to tensors
        states = torch.FloatTensor(list(self.state_buffer))
        actions = torch.FloatTensor(list(self.action_buffer))
        next_states = torch.FloatTensor(list(self.next_state_buffer))

        # Create input (state, action) and output (next_state - state) pairs
        inputs = torch.cat([states, actions], dim=1)
        outputs = next_states - states  # Delta state prediction

        # Train dynamics model (simplified)
        self.dynamics_model = nn.Sequential(
            nn.Linear(states.shape[1] + actions.shape[1], 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, states.shape[1])
        )

        optimizer = optim.Adam(self.dynamics_model.parameters())

        # Simple training loop
        for epoch in range(100):
            optimizer.zero_grad()
            pred_deltas = self.dynamics_model(inputs)
            loss = F.mse_loss(pred_deltas, outputs)
            loss.backward()
            optimizer.step()

        print(f"Dynamics model trained with final loss: {loss.item():.4f}")
        return True

    def predict_next_state(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        """Predict next state using the trained dynamics model"""
        if self.dynamics_model is None:
            return state  # Return current state if no model available

        inputs = torch.cat([state, action], dim=-1)
        delta = self.dynamics_model(inputs)
        return state + delta


class SimToRealTransfer:
    """
    Main class for sim-to-real transfer techniques
    """
    def __init__(self, state_dim: int, action_dim: int):
        self.state_dim = state_dim
        self.action_dim = action_dim

        # Initialize components
        self.domain_rand = DomainRandomization({
            'gravity': [9.0, 10.0],
            'friction': [0.1, 1.0],
            'mass_variance': [0.8, 1.2],
            'damping': [0.1, 0.5]
        })

        self.dyn_rand = DynamicsRandomization()
        self.vis_rand = VisualDomainRandomization()
        self.adaptation_net = AdaptationNetwork(action_dim, action_dim)
        self.sys_id = SystemIDAdaptation(state_dim, action_dim)

        # Training parameters
        self.adaptation_optimizer = optim.Adam(self.adaptation_net.parameters(), lr=1e-4)
        self.adaptation_buffer = deque(maxlen=1000)

    def apply_domain_randomization(self):
        """Apply domain randomization to simulation"""
        sim_params = self.domain_rand.randomize_parameters()
        dyn_params = self.dyn_rand.randomize_dynamics()

        # Combine all parameters
        all_params = {**sim_params, **dyn_params}

        return all_params

    def adapt_action(self, sim_action: np.ndarray, obs_diff: np.ndarray = None) -> np.ndarray:
        """
        Adapt action from simulation to real-world using adaptation network

        Args:
            sim_action: Action from simulation policy
            obs_diff: Difference in observations between sim and real (if available)

        Returns:
            Adapted action for real-world execution
        """
        action_tensor = torch.FloatTensor(sim_action).unsqueeze(0)

        # If observation difference is provided, use it for adaptation
        if obs_diff is not None:
            obs_tensor = torch.FloatTensor(obs_diff).unsqueeze(0)
            # Concatenate action and observation difference
            input_tensor = torch.cat([action_tensor, obs_tensor], dim=1)
            # This requires modifying the network to accept this concatenated input
            adapted_action = action_tensor  # Simplified for this example
        else:
            # Apply adaptation network to action directly
            adapted_action = self.adaptation_net(action_tensor)

        return adapted_action.squeeze(0).detach().numpy()

    def collect_adaptation_data(self, sim_obs: np.ndarray, real_obs: np.ndarray,
                               sim_action: np.ndarray, real_action: np.ndarray):
        """
        Collect data for adaptation learning

        Args:
            sim_obs: Observation from simulation
            real_obs: Observation from real robot
            sim_action: Action executed in simulation
            real_action: Action executed on real robot
        """
        data = {
            'sim_obs': sim_obs,
            'real_obs': real_obs,
            'sim_action': sim_action,
            'real_action': real_action
        }
        self.adaptation_buffer.append(data)

    def train_adaptation_network(self):
        """Train the adaptation network using collected data"""
        if len(self.adaptation_buffer) < 32:
            return False

        # Sample batch of data
        batch = np.random.choice(list(self.adaptation_buffer), size=min(32, len(self.adaptation_buffer)))

        sim_actions = torch.stack([torch.FloatTensor(data['sim_action']) for data in batch])
        real_actions = torch.stack([torch.FloatTensor(data['real_action']) for data in batch])

        # Train adaptation network to map sim actions to real actions
        self.adaptation_optimizer.zero_grad()
        pred_real_actions = self.adaptation_net(sim_actions)
        loss = F.mse_loss(pred_real_actions, real_actions)
        loss.backward()
        self.adaptation_optimizer.step()

        print(f"Adaptation network loss: {loss.item():.4f}")
        return True

    def reset_robot_state(self, sim_state: Dict[str, Any], real_state: Dict[str, Any]) -> Dict[str, Any]:
        """
        Reset real robot state to match simulation state as closely as possible

        Args:
            sim_state: State from simulation
            real_state: State from real robot

        Returns:
            Adjusted state to apply to real robot
        """
        # In practice, this would involve more complex state matching
        # For now, we'll just return the real state as is
        return real_state


def run_sim_to_real_pipeline():
    """Run a complete sim-to-real transfer pipeline example"""
    print("Starting Sim-to-Real Transfer Pipeline Example")

    # Initialize transfer system
    transfer_system = SimToRealTransfer(state_dim=32, action_dim=16)  # Example dimensions

    # Simulate domain randomization during training
    print("Applying domain randomization during simulation training...")
    for episode in range(5):  # Simulate 5 episodes with different domains
        params = transfer_system.apply_domain_randomization()
        print(f"Episode {episode+1} parameters: {params}")

    # Simulate data collection for adaptation
    print("\nCollecting adaptation data...")
    for step in range(100):
        # Simulate random data collection
        sim_obs = np.random.randn(32)
        real_obs = sim_obs + np.random.randn(32) * 0.1  # Add small noise
        sim_action = np.random.randn(16)
        real_action = sim_action + np.random.randn(16) * 0.05  # Add small noise

        transfer_system.collect_adaptation_data(sim_obs, real_obs, sim_action, real_action)

    # Train adaptation network
    print("\nTraining adaptation network...")
    success = transfer_system.train_adaptation_network()
    print(f"Adaptation training success: {success}")

    # Test action adaptation
    print("\nTesting action adaptation...")
    original_action = np.random.randn(16)
    adapted_action = transfer_system.adapt_action(original_action)
    print(f"Original action norm: {np.linalg.norm(original_action):.4f}")
    print(f"Adapted action norm: {np.linalg.norm(adapted_action):.4f}")

    # Test system identification
    print("\nTesting system identification...")
    for _ in range(50):  # Collect some data
        state = np.random.randn(32)
        action = np.random.randn(16)
        next_state = state + 0.1 * action + np.random.randn(32) * 0.01  # Simulate dynamics
        transfer_system.sys_id.collect_data(state, action, next_state)

    success = transfer_system.sys_id.train_dynamics_model()
    print(f"System ID training success: {success}")

    # Test dynamics prediction
    if success:
        state_tensor = torch.FloatTensor(np.random.randn(32)).unsqueeze(0)
        action_tensor = torch.FloatTensor(np.random.randn(16)).unsqueeze(0)
        predicted_next_state = transfer_system.sys_id.predict_next_state(state_tensor, action_tensor)
        print(f"Dynamics prediction shape: {predicted_next_state.shape}")

    print("\nSim-to-Real Transfer Pipeline Example Completed")


if __name__ == '__main__':
    run_sim_to_real_pipeline()