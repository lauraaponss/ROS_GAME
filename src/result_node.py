#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64
from RP_Pons_Laura_Cotobal_Claudia.msg import user_msg
from RP_Pons_Laura_Cotobal_Claudia.srv import GetUserScore

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
        self.user_sub = rospy.Subscriber('user_information', user_msg, self.user_callback)
        self.result_sub = rospy.Subscriber('result_information', Int64, self.result_callback)
        
        # Wait for the user_score service
        rospy.loginfo("Waiting for user_score service...")
        rospy.wait_for_service('user_score')
        self.get_score = rospy.ServiceProxy('user_score', GetUserScore)
        
        rospy.loginfo("Result Node started")
    
    def user_callback(self, data):
        """Callback for receiving user information"""
        try:
            self.name = data.name
            self.username = data.username
            self.age = data.age
            rospy.loginfo(f"Received user information for: {self.username}")
            
            # Try to get user's previous score
            try:
                response = self.get_score(self.username)
                if response.score > 0:
                    rospy.loginfo(f"Previous best score for {self.username}: {response.score}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                
        except Exception as e:
            rospy.logerr(f"Error processing user information: {e}")
    
    def result_callback(self, data):
        """Callback for receiving game result"""
        self.final_score = data.data
        rospy.loginfo(f"Received final score: {self.final_score}")
        
        if self.username is not None and self.final_score is not None:
            # Get score percentage through service
            try:
                response = self.get_score(self.username)
                score_percentage = (self.final_score / 1000.0) * 100  # Assuming max score is 1000
                rospy.loginfo(f"Score percentage for {self.username}: {score_percentage}%")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
            
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
        
        # Get previous score through service
        try:
            response = self.get_score(self.username)
            if response.score > 0 and response.score != self.final_score:
                print(f"Previous Best Score: {response.score}")
            score_percentage = (self.final_score / 1000.0) * 100
            print(f"Score Percentage: {score_percentage:.1f}%")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        
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