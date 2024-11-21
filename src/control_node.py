#!/usr/bin/env python3

import rospy
import sys
import tty
import termios
import select
from std_msgs.msg import String

class ControlNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('control_node')
        
        # Create publisher for keyboard controls
        self.control_pub = rospy.Publisher('keyboard_control', String, queue_size=10)
        
        # Store the terminal settings to restore them later
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        
        rospy.loginfo("Control Node started")
        rospy.loginfo("Use keyboard to control:")
        rospy.loginfo("a or ← : Move Left")
        rospy.loginfo("d or → : Move Right")
        rospy.loginfo("spacebar : Launch Ball")
        rospy.loginfo("q : Quit")
    
    def get_key(self):
        """Get a single keypress from the terminal"""
        try:
            # Set the terminal to raw mode
            tty.setraw(self.fd)
            
            # Wait for input with timeout
            if select.select([sys.stdin], [], [], 0.1)[0] == [sys.stdin]:
                key = sys.stdin.read(1)
                # Handle arrow keys
                if key == '\x1b':
                    # Read the next two characters
                    sys.stdin.read(1)  # skip '['
                    arrow = sys.stdin.read(1)
                    return f'arrow_{arrow}'
                return key
            return None
            
        finally:
            # Restore terminal settings
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
    
    def run(self):
        """Main run loop"""
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                
                if key is None:
                    continue
                
                if key in ['a', 'arrow_D']:  # 'a' or left arrow
                    self.control_pub.publish("LEFT")
                    rospy.logdebug("Published: LEFT")
                
                elif key in ['d', 'arrow_C']:  # 'd' or right arrow
                    self.control_pub.publish("RIGHT")
                    rospy.logdebug("Published: RIGHT")
                
                elif key == ' ':  # spacebar
                    self.control_pub.publish("SPACE")
                    rospy.logdebug("Published: SPACE")
                
                elif key == 'q':  # quit
                    rospy.loginfo("Quitting control node...")
                    break
                
        except Exception as e:
            rospy.logerr(f"Error in control node: {e}")
        finally:
            # Restore terminal settings
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

if __name__ == '__main__':
    try:
        node = ControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Ensure terminal settings are restored
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, 
                         termios.tcgetattr(sys.stdin.fileno()))