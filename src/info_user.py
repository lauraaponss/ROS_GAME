#!/usr/bin/env python3

import rospy
import os
import sys
from std_msgs.msg import String

# Add the msg directory to the Python path
msg_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'msg')
sys.path.append(msg_path)

class UserMsg:
    def __init__(self):
        self.name = ""
        self.username = ""
        self.age = 0

class InfoUserNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('info_user_node')
        
        # Create publisher for user information
        self.user_info_pub = rospy.Publisher('user_information', String, queue_size=10)
        
        # Log node startup
        rospy.loginfo("Info User Node started")
    
    def get_user_info(self):
        """Get user information from terminal input"""
        print("\n=== Welcome to Brick Breaker Deluxe ===")
        print("Please enter your information to start the game:\n")
        
        # Get name with validation
        while True:
            name = input("Enter your name: ").strip()
            if name and all(c.isalpha() or c.isspace() for c in name):
                break
            print("Please enter a valid name (only letters and spaces)")
        
        # Get username with validation
        while True:
            username = input("Enter your username: ").strip()
            if username and ' ' not in username:
                break
            print("Please enter a valid username (no spaces)")
        
        # Get age with validation
        while True:
            try:
                age = int(input("Enter your age: "))
                if 0 < age < 150:  # Basic age validation
                    break
                print("Please enter a valid age (1-150)")
            except ValueError:
                print("Please enter a number for age")
        
        return name, username, age
    
    def publish_user_info(self):
        """Get and publish user information"""
        try:
            # Get user information
            name, username, age = self.get_user_info()
            
            # Create message as a string for now
            msg = String()
            msg.data = f"{name},{username},{age}"  # Format: "name,username,age"
            
            # Publish the message
            self.user_info_pub.publish(msg)
            rospy.loginfo("User information published successfully")
            
            print("\nInformation sent! The game will start shortly...")
            
            # Keep node running
            rospy.spin()
            
        except KeyboardInterrupt:
            rospy.loginfo("User input interrupted")
        except Exception as e:
            rospy.logerr(f"Error in publishing user information: {e}")

if __name__ == '__main__':
    try:
        # Create and run the node
        info_user_node = InfoUserNode()
        info_user_node.publish_user_info()
    except rospy.ROSInterruptException:
        pass
