#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int64

class ResultNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('result_node')
        
        # Store user information
        self.username = None
        self.name = None
        self.age = None
        
        # Store score
        self.final_score = None
        
        # Subscribe to user information and game result
        self.user_sub = rospy.Subscriber('user_information', String, self.user_callback)
        self.result_sub = rospy.Subscriber('result_information', Int64, self.result_callback)
        
        rospy.loginfo("Result Node started")
    
    def user_callback(self, data):
        """Callback for receiving user information"""
        # Parse the comma-separated string
        try:
            self.name, self.username, age_str = data.data.split(',')
            self.age = int(age_str)
            rospy.loginfo(f"Received user information for: {self.username}")
        except Exception as e:
            rospy.logerr(f"Error processing user information: {e}")
    
    def result_callback(self, data):
        """Callback for receiving game result"""
        self.final_score = data.data
        rospy.loginfo(f"Received final score: {self.final_score}")
        
        # Display final results when both user info and score are available
        if self.username is not None and self.final_score is not None:
            self.display_results()
    
    def display_results(self):
        """Display the final results in a formatted way"""
        print("\n" + "="*50)
        print("GAME OVER - FINAL RESULTS")
        print("="*50)
        print(f"Player Name: {self.name}")
        print(f"Username: {self.username}")
        print(f"Age: {self.age}")
        print(f"Final Score: {self.final_score}")
        print("="*50)
        
        # Add some comments based on the score
        if self.final_score > 1000:
            print("Outstanding performance! You're a Brick Breaker master!")
        elif self.final_score > 500:
            print("Great job! You're getting really good at this!")
        elif self.final_score > 100:
            print("Good game! Keep practicing to improve your score!")
        else:
            print("Thanks for playing! Try again to beat your score!")
        print("="*50 + "\n")

    def run(self):
        """Main run loop"""
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ResultNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
