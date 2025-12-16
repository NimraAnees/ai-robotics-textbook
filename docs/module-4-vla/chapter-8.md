---
sidebar_label: 'Chapter 8: LLM-Based Cognitive Planning to ROS Actions'
sidebar_position: 2
---

# Chapter 8: LLM-Based Cognitive Planning to ROS Actions

## Overview
This chapter covers the implementation of Large Language Model (LLM)-based cognitive planning systems that translate high-level natural language commands into executable ROS actions for humanoid robots. Students will learn to design cognitive architectures that enable robots to understand complex commands, plan multi-step tasks, and execute coordinated behaviors.

## Learning Objectives
After completing this chapter, students will be able to:
- Integrate LLMs with ROS 2 for cognitive planning
- Design cognitive architectures for task planning and execution
- Convert natural language commands to structured ROS action sequences
- Implement multi-step task planning and execution
- Handle task failures and plan adaptations
- Validate cognitive planning accuracy and safety

## 8.1 Introduction to LLM-Based Cognitive Planning

### Cognitive Planning in Robotics
Cognitive planning for robots involves:
- **Understanding**: Interpreting high-level commands in natural language
- **Reasoning**: Determining appropriate sequences of actions
- **Planning**: Creating detailed execution plans with temporal and spatial constraints
- **Execution**: Coordinating ROS nodes and services to execute the plan
- **Monitoring**: Tracking plan execution and adapting to changes

### Role of LLMs in Cognitive Planning
Large Language Models serve as:
- **Natural Language Interface**: Converting human commands to structured representations
- **World Model**: Maintaining knowledge about the environment and objects
- **Reasoning Engine**: Determining appropriate action sequences
- **Adaptation System**: Handling unexpected situations and plan modifications

### Architecture Overview
```
Natural Language Command
         ↓
    [LLM] - Cognitive Reasoning
         ↓
Structured Action Plan
         ↓
   [Planner] - Task Decomposition
         ↓
ROS Action Sequence
         ↓
   [Executor] - ROS Node Coordination
         ↓
Physical Robot Actions
```

## 8.2 LLM Integration with ROS 2

### Supported LLM Platforms
For robotics applications, several LLM platforms can be integrated:

1. **OpenAI GPT Models**: High accuracy, cloud-based, requires API access
2. **Anthropic Claude**: Strong reasoning capabilities, cloud-based
3. **Open-Source Models**: Mistral, Llama 2/3, locally deployable
4. **Specialized Robotics Models**: Models fine-tuned for robotics tasks

### Basic LLM Integration Example
```python
import rclpy
from rclpy.node import Node
import openai
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
import json
import time

class LLMCognitivePlanner(Node):
    def __init__(self):
        super().__init__('llm_cognitive_planner')

        # Initialize LLM client (using OpenAI as example)
        # In practice, you might use other LLM providers or local models
        self.llm_client = None  # Initialize based on your chosen provider

        # Subscribers for various inputs
        self.voice_command_sub = self.create_subscription(
            String,
            'voice_command',
            self.voice_command_callback,
            10
        )

        self.text_command_sub = self.create_subscription(
            String,
            'text_command',
            self.text_command_callback,
            10
        )

        # Publishers for planning outputs
        self.plan_pub = self.create_publisher(
            String,
            'cognitive_plan',
            10
        )

        self.action_pub = self.create_publisher(
            String,
            'planned_actions',
            10
        )

        # Service clients for robot capabilities
        self.nav_client = self.create_client(
            NavigateToPose,
            'navigate_to_pose'
        )

        self.manipulation_client = self.create_client(
            ManipulationCommand,
            'manipulation_command'
        )

        self.get_logger().info('LLM Cognitive Planner initialized')

    def voice_command_callback(self, msg):
        """Process voice command through LLM cognitive planner"""
        self.process_command(msg.data, command_type='voice')

    def text_command_callback(self, msg):
        """Process text command through LLM cognitive planner"""
        self.process_command(msg.data, command_type='text')

    def process_command(self, command_text, command_type='voice'):
        """Process command using LLM cognitive planning"""
        try:
            # Generate cognitive plan using LLM
            plan = self.generate_plan(command_text)

            if plan:
                # Publish the plan
                plan_msg = String()
                plan_msg.data = json.dumps(plan)
                self.plan_pub.publish(plan_msg)

                # Execute the plan
                self.execute_plan(plan)

        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')

    def generate_plan(self, command_text):
        """Generate cognitive plan using LLM"""
        # Construct prompt for the LLM
        prompt = f"""
        You are a cognitive planner for a humanoid robot. Your task is to interpret the following command and create a detailed execution plan.

        Command: "{command_text}"

        The robot has the following capabilities:
        - Navigation: Can move to specified locations
        - Manipulation: Can pick up and place objects
        - Perception: Can detect objects in the environment
        - Interaction: Can respond to humans and perform social behaviors

        Please provide a structured plan in JSON format with the following schema:
        {{
            "command": "original command",
            "intent": "high-level intent",
            "steps": [
                {{
                    "action": "action_type",
                    "parameters": {{}},
                    "description": "what this step does"
                }}
            ],
            "objects": ["list of objects mentioned"],
            "locations": ["list of locations mentioned"],
            "confidence": float (0-1)
        }}

        Be specific about locations, objects, and actions. If the command is unclear, ask for clarification.
        """

        try:
            # Call the LLM (this is a simplified example)
            # In practice, you'd use the appropriate API call for your LLM
            response = self.call_llm(prompt)
            plan = json.loads(response)
            return plan
        except Exception as e:
            self.get_logger().error(f'Error generating plan: {e}')
            return None

    def call_llm(self, prompt):
        """Call the LLM with the given prompt"""
        # This is a placeholder - implement based on your chosen LLM provider
        # For example, with OpenAI:
        # response = openai.ChatCompletion.create(
        #     model="gpt-3.5-turbo",
        #     messages=[{"role": "user", "content": prompt}],
        #     temperature=0.1
        # )
        # return response.choices[0].message.content

        # For now, return a mock response for demonstration
        return '''
        {
            "command": "Go to the kitchen and bring me a cup",
            "intent": "fetch object from specific location",
            "steps": [
                {
                    "action": "navigate",
                    "parameters": {"location": "kitchen"},
                    "description": "Navigate to the kitchen area"
                },
                {
                    "action": "detect",
                    "parameters": {"object": "cup"},
                    "description": "Look for a cup in the environment"
                },
                {
                    "action": "manipulate",
                    "parameters": {"action": "pick", "object": "cup"},
                    "description": "Pick up the detected cup"
                },
                {
                    "action": "navigate",
                    "parameters": {"location": "starting_position"},
                    "description": "Return to the starting position"
                },
                {
                    "action": "manipulate",
                    "parameters": {"action": "place", "object": "cup"},
                    "description": "Place the cup down"
                }
            ],
            "objects": ["cup"],
            "locations": ["kitchen", "starting_position"],
            "confidence": 0.9
        }
        '''
```

## 8.3 Cognitive Architecture Design

### Planning Hierarchy
Cognitive planning typically follows a hierarchical structure:

1. **Task Level**: High-level goals and objectives
2. **Action Level**: Sequences of actions to achieve goals
3. **Motion Level**: Specific robot motions and configurations
4. **Execution Level**: Low-level control commands

### Context Management
```python
class ContextManager:
    def __init__(self):
        self.current_task = None
        self.task_history = []
        self.environment_state = {}
        self.robot_state = {}
        self.user_preferences = {}

    def update_environment_state(self, sensor_data):
        """Update environment state based on sensor data"""
        # Process sensor data and update environment model
        pass

    def get_context_prompt(self):
        """Generate context for LLM queries"""
        context = {
            "environment": self.environment_state,
            "robot": self.robot_state,
            "task_history": self.task_history[-5:],  # Last 5 tasks
            "current_task": self.current_task
        }
        return json.dumps(context, indent=2)
```

### Plan Validation and Safety
```python
class PlanValidator:
    def __init__(self):
        self.safety_rules = [
            self.check_collision_risk,
            self.check_physical_limits,
            self.check_human_safety
        ]

    def validate_plan(self, plan):
        """Validate plan for safety and feasibility"""
        for rule in self.safety_rules:
            if not rule(plan):
                return False, f"Plan failed {rule.__name__} check"
        return True, "Plan is valid"

    def check_collision_risk(self, plan):
        """Check if plan involves potential collisions"""
        # Implement collision checking logic
        return True

    def check_physical_limits(self, plan):
        """Check if plan respects robot physical limits"""
        # Implement physical limit checking
        return True

    def check_human_safety(self, plan):
        """Check if plan is safe around humans"""
        # Implement human safety checking
        return True
```

## 8.4 Multi-Step Task Planning

### Sequential Task Execution
```python
class TaskExecutor:
    def __init__(self, node):
        self.node = node
        self.current_plan = None
        self.current_step = 0
        self.execution_status = "idle"

    def execute_plan(self, plan):
        """Execute a multi-step plan"""
        self.current_plan = plan
        self.current_step = 0
        self.execution_status = "executing"

        while self.current_step < len(plan['steps']) and self.execution_status == "executing":
            step = plan['steps'][self.current_step]

            success = self.execute_step(step)

            if success:
                self.current_step += 1
            else:
                self.handle_failure(step)
                break

        self.execution_status = "completed" if self.current_step >= len(plan['steps']) else "failed"

    def execute_step(self, step):
        """Execute a single step in the plan"""
        action_type = step['action']
        parameters = step['parameters']

        self.node.get_logger().info(f'Executing step: {step["description"]}')

        if action_type == 'navigate':
            return self.execute_navigation(parameters)
        elif action_type == 'detect':
            return self.execute_detection(parameters)
        elif action_type == 'manipulate':
            return self.execute_manipulation(parameters)
        elif action_type == 'interact':
            return self.execute_interaction(parameters)
        else:
            self.node.get_logger().error(f'Unknown action type: {action_type}')
            return False

    def execute_navigation(self, params):
        """Execute navigation step"""
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.get_pose_for_location(params['location'])

        # Send navigation goal
        future = self.node.nav_client.call_async(goal_msg)

        # Wait for result (with timeout)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)

        if future.result() is not None:
            return future.result().result.success
        else:
            return False

    def execute_manipulation(self, params):
        """Execute manipulation step"""
        # Create manipulation goal
        goal_msg = ManipulationCommand.Goal()
        goal_msg.action = params['action']
        if 'object' in params:
            goal_msg.object_name = params['object']

        # Send manipulation goal
        future = self.node.manipulation_client.call_async(goal_msg)

        # Wait for result (with timeout)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)

        if future.result() is not None:
            return future.result().result.success
        else:
            return False
```

## 8.5 LLM Prompt Engineering for Robotics

### Effective Prompting Strategies
```python
class PromptEngineer:
    def __init__(self):
        self.system_prompt = """
        You are a cognitive planning assistant for a humanoid robot. Your role is to:
        1. Interpret natural language commands from humans
        2. Create detailed, executable plans for the robot
        3. Consider safety, feasibility, and efficiency
        4. Ask for clarification when commands are ambiguous

        Always respond in valid JSON format with the specified schema.
        """

    def create_planning_prompt(self, command, context):
        """Create a prompt for cognitive planning"""
        return f"""
        {self.system_prompt}

        CONTEXT:
        {context}

        COMMAND: {command}

        Please provide a detailed plan in the following JSON format:
        {{
            "command": "{command}",
            "intent": "high-level intent",
            "steps": [
                {{
                    "action": "action_type",
                    "parameters": {{}},
                    "description": "what this step does",
                    "expected_outcome": "what should happen after this step"
                }}
            ],
            "objects": ["list", "of", "relevant", "objects"],
            "locations": ["list", "of", "relevant", "locations"],
            "confidence": 0.0-1.0,
            "reasoning": "brief explanation of your plan"
        }}

        Requirements:
        - Each step should be executable by the robot
        - Include error handling where appropriate
        - Consider the current state of the world
        - Prioritize safety in all actions
        """

    def create_clarification_prompt(self, ambiguous_command, context):
        """Create a prompt to ask for clarification"""
        return f"""
        {self.system_prompt}

        CONTEXT:
        {context}

        AMBIGUOUS COMMAND: {ambiguous_command}

        The command is ambiguous. Please ask specific questions to clarify:
        1. Which specific object is meant?
        2. Which specific location is meant?
        3. What exactly should the robot do?

        Respond with a list of specific questions to ask the user.
        """
```

## 8.6 Handling Uncertainty and Adaptation

### Plan Adaptation System
```python
class PlanAdaptationSystem:
    def __init__(self):
        self.adaptation_strategies = {
            'object_not_found': self.handle_object_not_found,
            'location_not_reachable': self.handle_location_not_reachable,
            'action_failed': self.handle_action_failed
        }

    def handle_failure(self, failed_step, error_type):
        """Handle plan execution failure"""
        if error_type in self.adaptation_strategies:
            return self.adaptation_strategies[error_type](failed_step)
        else:
            return self.generic_failure_handling(failed_step)

    def handle_object_not_found(self, step):
        """Handle case where expected object is not found"""
        # Ask LLM for alternative approaches
        prompt = f"""
        The robot was looking for {step['parameters'].get('object', 'an object')}
        but could not find it. What should the robot do next?

        Possible alternatives:
        1. Look in other locations
        2. Ask user for help
        3. Use a substitute object
        4. Abort the task

        Respond with a new plan step or a list of options.
        """

        # Call LLM for adaptation strategy
        # Return new plan or continue with adaptation
        pass

    def handle_location_not_reachable(self, step):
        """Handle case where navigation target is not reachable"""
        # Implement navigation adaptation
        pass

    def handle_action_failed(self, step):
        """Handle case where an action failed"""
        # Implement action-specific adaptation
        pass
```

## 8.7 Integration with ROS 2 Ecosystem

### Service Definitions
```python
# cognitive_planning_interfaces/srv/GeneratePlan.srv
string command
string context
---
string plan_json
float64 confidence
bool success
string error_message

# cognitive_planning_interfaces/srv/ExecutePlan.srv
string plan_json
---
bool success
string status
float64 execution_time
```

### Action Integration
```python
from rclpy.action import ActionClient
from cognitive_planning_interfaces.action import ExecuteCognitivePlan

class LLMPlannerWithActions(LLMCognitivePlanner):
    def __init__(self):
        super().__init__()

        # Create action client for plan execution
        self.plan_execution_client = ActionClient(
            self,
            ExecuteCognitivePlan,
            'execute_cognitive_plan'
        )

    def execute_plan_with_feedback(self, plan):
        """Execute plan with progress feedback"""
        goal_msg = ExecuteCognitivePlan.Goal()
        goal_msg.plan_json = json.dumps(plan)
        goal_msg.timeout = rclpy.Duration(seconds=300)  # 5 minute timeout

        # Wait for action server
        self.plan_execution_client.wait_for_server()

        # Send goal and get feedback
        future = self.plan_execution_client.send_goal_async(
            goal_msg,
            feedback_callback=self.plan_feedback_callback
        )

        future.add_done_callback(self.plan_complete_callback)

    def plan_feedback_callback(self, feedback_msg):
        """Handle plan execution feedback"""
        self.get_logger().info(f'Plan progress: {feedback_msg.feedback.progress}%')

    def plan_complete_callback(self, future):
        """Handle plan completion"""
        goal_handle = future.result()
        result = goal_handle.get_result_async()
        result.add_done_callback(self.plan_result_callback)

    def plan_result_callback(self, future):
        """Handle plan execution result"""
        result_msg = future.result().result
        self.get_logger().info(f'Plan execution result: {result_msg.success}')
```

## 8.8 Practical Exercise: Cognitive Planning System

### Exercise Objective
Create a complete cognitive planning system that takes natural language commands and executes them on a simulated humanoid robot.

### Steps
1. Set up LLM integration with ROS 2
2. Implement context management system
3. Create plan validation and safety checks
4. Develop multi-step task execution
5. Add failure handling and adaptation
6. Test with various natural language commands
7. Validate safety and correctness

### Expected Results
- Natural language command understanding (>80% accuracy)
- Safe and correct plan execution
- Proper failure handling and adaptation
- Integration with ROS 2 messaging and services

## Summary
LLM-based cognitive planning provides a powerful approach to bridge natural language commands with robotic action execution. By properly designing the cognitive architecture, implementing safety checks, and handling uncertainty, we can create intelligent humanoid robots capable of understanding and executing complex tasks through natural interaction.

## Key Terms
- **Cognitive Planning**: High-level planning using reasoning and knowledge
- **Large Language Model (LLM)**: AI model for natural language understanding
- **Task Decomposition**: Breaking complex tasks into simpler steps
- **Plan Validation**: Checking plans for safety and feasibility
- **Adaptation System**: Handling plan failures and unexpected situations

## References
- [OpenAI API Documentation](https://platform.openai.com/docs/)
- [Robotics and AI Integration](https://arxiv.org/abs/2303.04650)
- [Natural Language to Robot Action](https://arxiv.org/abs/2205.12258)
- [Cognitive Architectures for Robotics](https://arxiv.org/abs/2103.06759)