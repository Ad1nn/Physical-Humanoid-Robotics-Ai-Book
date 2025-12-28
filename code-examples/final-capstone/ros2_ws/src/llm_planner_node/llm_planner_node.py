import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from openai import OpenAI # Example for OpenAI API

class LlmPlannerNode(Node):
    """
    Placeholder ROS 2 node for LLM-based cognitive planning.
    Subscribes to high-level text commands, queries an LLM for a plan, and publishes the structured plan.
    """
    def __init__(self):
        super().__init__('llm_planner_node')
        self.declare_parameter('llm_model', 'gpt-4-turbo-preview')
        self.llm_model = self.get_parameter('llm_model').get_parameter_value().string_value
        
        self.client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY")) # Initialize OpenAI client

        self.command_subscription = self.create_subscription(
            String,
            'voice_commands_text', # Input from voice command node
            self.command_callback,
            10
        )
        self.plan_publisher = self.create_publisher(
            String, # Publish JSON string representation of the plan
            'robot_plan',
            10
        )
        self.get_logger().info(f"LLM Planner Node started with model: {self.llm_model}")

    def command_callback(self, msg: String):
        """
        Callback function when a human command is received.
        Queries the LLM and publishes the generated plan.
        """
        human_command = msg.data
        self.get_logger().info(f"Received human command: '{human_command}' for planning.")

        # In a full system, robot_state and available_actions would come from other ROS nodes
        robot_state = {
            "location": "living room",
            "battery": "90%",
            "objects_in_view": ["couch", "TV"]
        }
        available_actions = ["navigate_to(location)", "detect_object(object_name)", "grasp_object(object_id)", "report_status(message)"]

        plan = self._query_llm_for_plan(human_command, robot_state, available_actions)
        
        if "error" not in plan:
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_publisher.publish(plan_msg)
            self.get_logger().info("Published generated plan.")
        else:
            self.get_logger().error(f"Failed to generate plan: {plan['error']}")

    def _query_llm_for_plan(self, goal_description, robot_state, available_actions):
        """
        Internal function to query an LLM for a robot plan.
        """
        prompt = f"""
        You are a helpful robot task planner. Your goal is to break down high-level human commands into a sequence of specific robot actions.
        
        Current Robot State: {json.dumps(robot_state, indent=2)}
        Available Actions: {json.dumps(available_actions, indent=2)}
        
        Human Command: {goal_description}
        
        Please provide a plan as a JSON array of actions. Each action should have "name" and "parameters".
        
        Example:
        [
          {{"name": "navigate_to", "parameters": {{"location": "kitchen"}}}},
          {{"name": "detect_object", "parameters": {{"object_name": "red cup"}}}},
          {{"name": "grasp_object", "parameters": {{"object_id": "red_cup_instance_1"}}}}
        ]
        
        Provide only the JSON array.
        """

        try:
            response = self.client.chat.completions.create(
                model=self.llm_model,
                messages=[
                    {"role": "system", "content": "You are a robot task planner."},
                    {"role": "user", "content": prompt}
                ],
                response_format={"type": "json_object"}
            )
            plan_text = response.choices[0].message.content
            return json.loads(plan_text)
        except Exception as e:
            return {"error": str(e)}

def main(args=None):
    rclpy.init(args=args)
    try:
        node = LlmPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
