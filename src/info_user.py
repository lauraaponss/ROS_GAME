#!/usr/bin/env python3

import rospy
import os
import sys
from RP_Pons_Laura_Cotobal_Claudia.msg import user_msg  # Import custom message
import time

class InfoUserNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('info_user_node')
        
        # Create publisher for user information using custom message
        self.user_info_pub = rospy.Publisher('user_information', user_msg, queue_size=10)
        
        # Give time for other nodes to start
        time.sleep(2)
        
        # Log node startup
        rospy.loginfo("Info User Node started")
    
    def get_user_info(self):
        """Get user information from terminal input"""
        # Clear terminal
        os.system('clear')
        
        print("\n=== Welcome to Brick Breaker Deluxe ===")
        print("Please enter your information to start the game:\n")
        
        # Get name with validation
        while True:
            try:
                sys.stdout.write("Enter your name: ")
                sys.stdout.flush()
                name = sys.stdin.readline().strip()
                if name and all(c.isalpha() or c.isspace() for c in name):
                    break
                print("Please enter a valid name (only letters and spaces)")
            except:
                continue
        
        # Get username with validation
        while True:
            try:
                sys.stdout.write("Enter your username: ")
                sys.stdout.flush()
                username = sys.stdin.readline().strip()
                if username and ' ' not in username:
                    break
                print("Please enter a valid username (no spaces)")
            except:
                continue
        
        # Get age with validation
        while True:
            try:
                sys.stdout.write("Enter your age: ")
                sys.stdout.flush()
                age = int(sys.stdin.readline().strip())
                if 0 < age < 150:  # Basic age validation
                    break
                print("Please enter a valid age (1-150)")
            except ValueError:
                print("Please enter a number for age")
            except:
                continue
        
        return name, username, age
    
    def publish_user_info(self):
        """Get and publish user information"""
        try:
            # Get user information
            name, username, age = self.get_user_info()
            
            # Create custom message
            msg = user_msg()
            msg.name = name
            msg.username = username
            msg.age = age
            
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