#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool

class SimpleFoodDeliveryRobot:
    def __init__(self):
        rospy.init_node('simple_food_delivery_robot', anonymous=True)
        
        # Define positions for home, kitchen, and customer tables
        self.home_position = (0.0, 0.0)
        self.kitchen_position = (-2.0, 1)
        self.customer_table_positions = {
            'table1': (2.0, 2.0),
            'table2': (5.0, 0.5),
            'table3': (4.0, 2.0)
        }
        
        # Action client to control robot movement
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server")

        # Initialize confirmation flags for each table (set to False initially)
        self.confirmations = {'table1': True, 'table2': True, 'table3': False}
        
        # Placeholder for cancel flag (external cancellation)
        self.cancel_task = False

    def move_to_goal(self, x, y, frame="map"):
        # Move the robot to the specified position
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0  # No rotation
        
        rospy.loginfo(f"Moving to position: ({x}, {y})")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        # Check if the robot successfully reached the goal
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Successfully reached the goal!")
        else:
            rospy.logwarn("Failed to reach the goal!")

    def handle_confirmation(self, table, timeout=10):
        # Subscribe to the table confirmation topic (e.g., /table1_confirmation)
        confirmation_topic = f"/{table}_confirmation"
        rospy.Subscriber(confirmation_topic, Bool, self.confirmation_callback, callback_args=table)
        
        # Wait for confirmation for the given table, with a timeout
        rospy.loginfo(f"Waiting for confirmation at {table}...")
        start_time = rospy.get_time()
        
        # Check if the confirmation flag becomes True within the timeout period
        while rospy.get_time() - start_time < timeout:
            if self.confirmations[table]:
                rospy.loginfo(f"Received confirmation at {table}")
                return True
            rospy.sleep(1)
        
        # If no confirmation received within the timeout period
        rospy.logwarn(f"Timeout waiting for confirmation at {table}")
        return False

    def confirmation_callback(self, msg, table):
        # Update the confirmation flag when confirmation is received
        if msg.data:
            self.confirmations[table] = True
            rospy.loginfo(f"Confirmation received for {table}")
        else:
            self.confirmations[table] = False

    def cancel_and_return(self):
        # If the task is canceled, return to kitchen and then home
        rospy.loginfo("Task canceled. Returning to kitchen and then home.")
        self.move_to_goal(self.kitchen_position[0], self.kitchen_position[1])
        self.move_to_goal(self.home_position[0], self.home_position[1])

    def process_order(self, tables):
        for table in tables:
            # Go to kitchen to collect food
            rospy.loginfo("Going to the kitchen to collect food...")
            self.move_to_goal(self.kitchen_position[0], self.kitchen_position[1])
            
            # Simulate receiving food from the kitchen
            rospy.loginfo("Received food from kitchen.")
            
            # Deliver food to the table
            rospy.loginfo(f"Going to {table}...")
            self.move_to_goal(self.customer_table_positions[table][0], self.customer_table_positions[table][1])

            # Wait for confirmation at the table
            if not self.handle_confirmation(table, timeout=5):
                rospy.logwarn(f"No one attended at {table}. Returning to kitchen...")
                self.move_to_goal(self.kitchen_position[0], self.kitchen_position[1])
                self.move_to_goal(self.home_position[0], self.home_position[1])
                return
            
        # After all deliveries, return to home position
        rospy.loginfo("All deliveries completed. Returning to home.")
        self.move_to_goal(self.home_position[0], self.home_position[1])

    def cancel_task_callback(self, msg):
        self.cancel_task = msg.data
        if self.cancel_task:
            rospy.loginfo("Task canceled by external signal.")
            self.cancel_and_return()

    def run(self):
        # Subscribe to the cancel task signal (external cancellation)
        rospy.Subscriber("/cancel_task", Bool, self.cancel_task_callback)
        
        # Example: Process orders for table1, table2, and table3
        tables_to_serve = ['table1', 'table2', 'table3']
        self.process_order(tables_to_serve)

if __name__ == "__main__":
    try:
        robot = SimpleFoodDeliveryRobot()
        robot.run()
    except rospy.ROSInterruptException:
        pass
