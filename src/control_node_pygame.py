#!/usr/bin/env python3

import rospy
import pygame
from std_msgs.msg import String

class ControlNodePygame:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('control_node_pygame')
        
        # Create publisher for keyboard controls
        self.control_pub = rospy.Publisher('keyboard_control', String, queue_size=10)
        
        # Initialize Pygame for input handling
        pygame.init()
        # Create a small window for input capture
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Game Controls")
        
        # Initialize clock for controlling publish rate
        self.clock = pygame.time.Clock()
        
        rospy.loginfo("Control Node (Pygame) started - Use arrow keys to control")
        rospy.loginfo("Controls:")
        rospy.loginfo("← : Move Left")
        rospy.loginfo("→ : Move Right")
        rospy.loginfo("SPACE : Launch Ball")
        rospy.loginfo("Close window to exit")
    
    def handle_input(self):
        """Handle Pygame keyboard events and publish control messages"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            # Handle key press events
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.control_pub.publish("SPACE")
                    rospy.logdebug("Published: SPACE")
        
        # Get the current state of all keyboard buttons
        keys = pygame.key.get_pressed()
        
        # Handle held keys for continuous movement
        if keys[pygame.K_LEFT]:
            self.control_pub.publish("LEFT")
            rospy.logdebug("Published: LEFT")
        
        if keys[pygame.K_RIGHT]:
            self.control_pub.publish("RIGHT")
            rospy.logdebug("Published: RIGHT")
        
        return True
    
    def run(self):
        """Main run loop"""
        try:
            while not rospy.is_shutdown():
                # Handle input
                if not self.handle_input():
                    break
                
                # Update display
                self.screen.fill((0, 0, 0))  # Black background
                pygame.display.flip()
                
                # Control the loop rate
                self.clock.tick(60)  # 60 FPS
                
        except Exception as e:
            rospy.logerr(f"Error in control node: {e}")
        finally:
            pygame.quit()

if __name__ == '__main__':
    try:
        node = ControlNodePygame()
        node.run()
    except rospy.ROSInterruptException:
        pygame.quit()