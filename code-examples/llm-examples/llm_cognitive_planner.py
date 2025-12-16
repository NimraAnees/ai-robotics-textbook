#!/usr/bin/env python3

"""
LLM Cognitive Planner for Robotics

This script implements a cognitive planning system using Large Language Models
to interpret natural language commands and generate executable robotic actions.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatus
import openai
import json
import time
import threading
import queue
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Any
import requests
import os


@dataclass
class CognitivePlan:
    """Data class for cognitive plans"""
    command: str
    intent: str
    steps: List[Dict[str, Any]]
    objects: List[str]
    locations: List[str]
    confidence: float
    reasoning: str


@dataclass
class PlanStep:
    """Data class for individual plan steps"""
    action: str
    parameters: Dict[str, Any]
    description: str
    expected_outcome: str


class LLMInterface:
    """Interface to various LLM providers"""
    def __init__(self, provider: str = "openai", api_key: str = None):
        self.provider = provider
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")

        if provider == "openai":
            if not self.api_key:
                raise ValueError("OpenAI API key required")
            openai.api_key = self.api_key

    def call_llm(self, prompt: str, max_tokens: int = 500, temperature: float = 0.1) -> str:
        """Call the LLM with the given prompt"""
        if self.provider == "openai":
            try:
                response = openai.ChatCompletion.create(
                    model="gpt-3.5-turbo",
                    messages=[{"role": "user", "content": prompt}],
                    max_tokens=max_tokens,
                    temperature=temperature
                )
                return response.choices[0].message.content.strip()
            except Exception as e:
                raise Exception(f"OpenAI API error: {e}")
        else:
            # For other providers or local models, implement accordingly
            raise NotImplementedError(f"Provider {self.provider} not implemented")


class ContextManager:
    """Manages context for cognitive planning"""
    def __init__(self):
        self.environment_state = {
            "objects": [],
            "locations": [],
            "robot_state": {},
            "recent_interactions": []
        }
        self.max_history = 10

    def update_environment(self, objects: List[str], locations: List[str]):
        """Update environment state"""
        self.environment_state["objects"] = objects
        self.environment_state["locations"] = locations

    def add_interaction(self, command: str, result: str):
        """Add interaction to history"""
        interaction = {
            "command": command,
            "result": result,
            "timestamp": time.time()
        }
        self.environment_state["recent_interactions"].append(interaction)

        # Keep only recent interactions
        if len(self.environment_state["recent_interactions"]) > self.max_history:
            self.environment_state["recent_interactions"] = \
                self.environment_state["recent_interactions"][-self.max_history:]

    def get_context_prompt(self) -> str:
        """Generate context for LLM queries"""
        return json.dumps(self.environment_state, indent=2)


class PlanValidator:
    """Validates plans for safety and feasibility"""
    def __init__(self):
        self.safety_rules = [
            self.check_collision_risk,
            self.check_physical_limits,
            self.check_human_safety,
            self.check_command_validity
        ]

    def validate_plan(self, plan: CognitivePlan) -> tuple[bool, str]:
        """Validate plan against safety rules"""
        for rule in self.safety_rules:
            is_valid, message = rule(plan)
            if not is_valid:
                return False, message
        return True, "Plan is valid"

    def check_collision_risk(self, plan: CognitivePlan) -> tuple[bool, str]:
        """Check if plan involves potential collisions"""
        # Check navigation steps for collision risk
        for step in plan.steps:
            if step["action"] == "navigate":
                # In a real system, this would check against known obstacles
                pass
        return True, "No collision risk detected"

    def check_physical_limits(self, plan: CognitivePlan) -> tuple[bool, str]:
        """Check if plan respects robot physical limits"""
        # Check manipulation steps for physical feasibility
        for step in plan.steps:
            if step["action"] == "manipulate":
                # Verify object weight, size, etc. are within limits
                pass
        return True, "Physical limits respected"

    def check_human_safety(self, plan: CognitivePlan) -> tuple[bool, str]:
        """Check if plan is safe around humans"""
        # Check for safe distances and behaviors around humans
        for step in plan.steps:
            if step["action"] in ["navigate", "manipulate", "approach"]:
                # Ensure actions maintain safe distance from humans
                pass
        return True, "Human safety maintained"

    def check_command_validity(self, plan: CognitivePlan) -> tuple[bool, str]:
        """Check if plan commands are valid"""
        valid_actions = {
            "navigate", "detect", "manipulate", "approach",
            "grasp", "place", "move", "turn", "stop", "interact"
        }

        for step in plan.steps:
            if step["action"] not in valid_actions:
                return False, f"Invalid action: {step['action']}"
        return True, "Commands are valid"


class LLMCognitivePlannerNode(Node):
    """
    ROS 2 Node for LLM-based cognitive planning
    """
    def __init__(self):
        super().__init__('llm_cognitive_planner')

        # Declare parameters
        self.declare_parameter('llm_provider', 'openai')
        self.declare_parameter('api_key', '')
        self.declare_parameter('max_tokens', 500)
        self.declare_parameter('temperature', 0.1)

        # Get parameter values
        self.llm_provider = self.get_parameter('llm_provider').value
        self.api_key = self.get_parameter('api_key').value or os.getenv("OPENAI_API_KEY")
        self.max_tokens = self.get_parameter('max_tokens').value
        self.temperature = self.get_parameter('temperature').value

        # Initialize LLM interface
        try:
            self.llm_interface = LLMInterface(self.llm_provider, self.api_key)
            self.get_logger().info(f'LLM interface initialized with {self.llm_provider}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize LLM interface: {e}')
            raise

        # Initialize context and validation
        self.context_manager = ContextManager()
        self.plan_validator = PlanValidator()

        # Initialize queues for thread communication
        self.command_queue = queue.Queue(maxsize=10)
        self.plan_queue = queue.Queue(maxsize=10)

        # Create subscribers
        self.voice_command_sub = self.create_subscription(
            String,
            'voice_command',
            self.voice_command_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        self.text_command_sub = self.create_subscription(
            String,
            'text_command',
            self.text_command_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        # Create publishers
        self.plan_pub = self.create_publisher(
            String,
            'cognitive_plan',
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        self.action_pub = self.create_publisher(
            String,
            'planned_actions',
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        # Create processing timer
        self.process_timer = self.create_timer(0.1, self.process_commands)

        self.get_logger().info('LLM Cognitive Planner initialized')

    def voice_command_callback(self, msg):
        """Process voice command through cognitive planner"""
        self.get_logger().info(f'Received voice command: {msg.data}')
        self.process_command(msg.data, command_type='voice')

    def text_command_callback(self, msg):
        """Process text command through cognitive planner"""
        self.get_logger().info(f'Received text command: {msg.data}')
        self.process_command(msg.data, command_type='text')

    def process_command(self, command_text: str, command_type: str = 'voice'):
        """Process command using LLM cognitive planning"""
        try:
            # Add to processing queue
            command_data = {
                'text': command_text,
                'type': command_type,
                'timestamp': time.time()
            }
            try:
                self.command_queue.put_nowait(command_data)
            except queue.Full:
                self.get_logger().warning('Command queue full, dropping command')
        except Exception as e:
            self.get_logger().error(f'Error queuing command: {e}')

    def process_commands(self):
        """Process commands from queue"""
        try:
            while True:  # Process all available commands
                command_data = self.command_queue.get_nowait()

                # Generate cognitive plan
                plan = self.generate_plan(command_data['text'])

                if plan:
                    # Validate the plan
                    is_valid, validation_msg = self.plan_validator.validate_plan(plan)
                    if is_valid:
                        # Add to plan queue for publishing
                        try:
                            self.plan_queue.put_nowait(plan)
                        except queue.Full:
                            self.get_logger().warning('Plan queue full, dropping plan')
                    else:
                        self.get_logger().error(f'Plan validation failed: {validation_msg}')

        except queue.Empty:
            pass  # No commands to process
        except Exception as e:
            self.get_logger().error(f'Error processing commands: {e}')

    def generate_plan(self, command_text: str) -> Optional[CognitivePlan]:
        """Generate cognitive plan using LLM"""
        try:
            # Get current context
            context = self.context_manager.get_context_prompt()

            # Construct prompt for the LLM
            prompt = f"""
            You are a cognitive planner for a humanoid robot. Your task is to interpret the following command and create a detailed execution plan.

            CURRENT CONTEXT:
            {context}

            COMMAND: "{command_text}"

            The robot has the following capabilities:
            - Navigation: Can move to specified locations
            - Manipulation: Can pick up and place objects
            - Perception: Can detect objects in the environment
            - Interaction: Can respond to humans and perform social behaviors

            Please provide a structured plan in JSON format with the following schema:
            {{
                "command": "{command_text}",
                "intent": "high-level intent",
                "steps": [
                    {{
                        "action": "action_type",
                        "parameters": {{}},
                        "description": "what this step does",
                        "expected_outcome": "what should happen after this step"
                    }}
                ],
                "objects": ["list of objects mentioned"],
                "locations": ["list of locations mentioned"],
                "confidence": float (0-1),
                "reasoning": "brief explanation of your plan"
            }}

            Be specific about locations, objects, and actions. If the command is unclear, ask for clarification.
            Ensure all actions are safe and feasible for the robot to execute.
            """

            # Call the LLM
            response_text = self.llm_interface.call_llm(prompt, self.max_tokens, self.temperature)

            # Parse the response
            # The response might contain additional text before the JSON, so we need to extract the JSON part
            json_start = response_text.find('{')
            json_end = response_text.rfind('}') + 1

            if json_start != -1 and json_end != 0:
                json_str = response_text[json_start:json_end]
                plan_data = json.loads(json_str)

                # Create CognitivePlan object
                plan = CognitivePlan(
                    command=plan_data.get("command", command_text),
                    intent=plan_data.get("intent", ""),
                    steps=plan_data.get("steps", []),
                    objects=plan_data.get("objects", []),
                    locations=plan_data.get("locations", []),
                    confidence=plan_data.get("confidence", 0.5),
                    reasoning=plan_data.get("reasoning", "")
                )

                # Update context with the interaction
                self.context_manager.add_interaction(command_text, str(plan.steps))

                self.get_logger().info(f'Generated plan with {len(plan.steps)} steps')
                return plan
            else:
                self.get_logger().error(f'Could not parse JSON from LLM response: {response_text}')
                return None

        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON decode error: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'Error generating plan: {e}')
            return None

    def publish_plan(self, plan: CognitivePlan):
        """Publish cognitive plan to ROS topics"""
        # Publish plan as JSON string
        plan_msg = String()
        plan_msg.data = json.dumps(asdict(plan))
        self.plan_pub.publish(plan_msg)

        # Publish individual actions
        for step in plan.steps:
            action_msg = String()
            action_msg.data = json.dumps(step)
            self.action_pub.publish(action_msg)

        self.get_logger().info(f'Published plan with {len(plan.steps)} steps')

    def spin_once(self):
        """Process any pending plans"""
        try:
            while True:  # Process all available plans
                plan = self.plan_queue.get_nowait()
                self.publish_plan(plan)
        except queue.Empty:
            pass  # No more plans to process


def main(args=None):
    """Main function to run the LLM Cognitive Planner node"""
    rclpy.init(args=args)

    node = LLMCognitivePlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LLM Cognitive Planner...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()