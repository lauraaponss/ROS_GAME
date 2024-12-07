#!/usr/bin/env python3

import rospy
import pygame
import random
import math
from std_msgs.msg import Int64, String
from RP_Pons_Laura_Cotobal_Claudia.msg import user_msg
from RP_Pons_Laura_Cotobal_Claudia.srv import GetUserScore, GetUserScoreResponse, SetGameDifficulty, SetGameDifficultyResponse

class ReleasedObject:
    def __init__(self, x, y):
        self.rect = pygame.Rect(x, y, 30, 30)  # Size of the released object
        self.speed = 2  # Speed at which the object falls

    def move(self):
        self.rect.y += self.speed  # Move the object down

    def draw(self, screen):
        pygame.draw.rect(screen, (255, 0, 0), self.rect)  # Draw the object in red

class GameNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('game_node')
        self.paused = False
        
        # Initialize parameters
        self.user_name = rospy.get_param('~user_name', 'default_user')
        self.player_color = rospy.get_param('~change_player_color', 1)  # 1:Red, 2:Purple, 3:Blue
        self.screen_param = rospy.get_param('~screen_param', 'phase1')
        
        # ROS Publishers and Subscribers
        self.user_sub = rospy.Subscriber('user_information', user_msg, self.user_callback)
        self.keyboard_sub = rospy.Subscriber('keyboard_control', String, self.keyboard_callback)
        self.result_pub = rospy.Publisher('result_information', Int64, queue_size=10)
        
        # Initialize services
        self.score_service = rospy.Service('user_score', GetUserScore, self.handle_get_user_score)
        self.difficulty_service = rospy.Service('difficulty', SetGameDifficulty, self.handle_set_difficulty)
        
        # Dictionary to store user scores
        self.user_scores = {}
        
        # Game phases
        self.current_phase = "WELCOME"  # WELCOME, GAME, FINAL
        self.username = None
        
        # Initialize difficulty
        self.difficulty = 'medium'  # default difficulty
        
        # Initialize Pygame
        pygame.init()
        
        # Set up display
        self.width = 800
        self.height = 600
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Brick Breaker Deluxe")
        
        # Define colors
        self.BACKGROUND_COLOR = (0, 20, 40)  # Dark blue background
        self.PADDLE_COLOR = (200, 200, 200)  # Light gray paddle
        self.COMBO_COLOR = (0, 255, 0)       # Green for combos
        
        # Define brick types
        self.BRICK_TYPES = {
            'normal': {'color': (255, 0, 0), 'hits': 1},
            'tough': {'color': (0, 0, 255), 'hits': 2},
            'super': {'color': (0, 255, 0), 'hits': 3}
        }

        # Paddle properties
        self.INITIAL_PADDLE_WIDTH = 120
        self.MIN_PADDLE_WIDTH = 60
        self.PADDLE_SHRINK_RATE = 5
        self.PADDLE_SHRINK_HITS = 5
        
        # Initialize other variables
        self.released_objects = []
        self.best_score = 0
        self.hits_since_shrink = 0
        
        # Game components initialization
        self.initialize_game_components()
        
        rospy.loginfo("Game Node started - Waiting for player information")

    def handle_get_user_score(self, req):
        """Service handler for GetUserScore"""
        response = GetUserScoreResponse()
        if req.username in self.user_scores:
            response.score = self.user_scores[req.username]
        else:
            response.score = 0
        return response

    def handle_set_difficulty(self, req):
        """Service handler for SetGameDifficulty"""
        response = SetGameDifficultyResponse()
        
        if self.current_phase != "WELCOME":
            response.success = False
            return response
            
        difficulty = req.change_difficulty.lower()
        if difficulty in ['easy', 'medium', 'hard']:
            self.difficulty = difficulty
            # Adjust game parameters based on difficulty
            if difficulty == 'easy':
                self.PADDLE_SHRINK_RATE = 3
                self.ball_speed_x = 4
                self.ball_speed_y = -4
            elif difficulty == 'medium':
                self.PADDLE_SHRINK_RATE = 5
                self.ball_speed_x = 5
                self.ball_speed_y = -5
            else:  # hard
                self.PADDLE_SHRINK_RATE = 7
                self.ball_speed_x = 6
                self.ball_speed_y = -6
            
            response.success = True
        else:
            response.success = False
            
        return response
    def initialize_game_components(self):
            """Initialize all game variables"""
            # Paddle and ball properties
            self.paddle_width = self.INITIAL_PADDLE_WIDTH
            self.paddle_height = 20
            self.ball_radius = 10
            self.paddle_speed = 20
            
            # Create paddle and ball
            self.paddle = pygame.Rect(self.width // 2 - self.paddle_width // 2,
                                    self.height - 50,
                                    self.paddle_width,
                                    self.paddle_height)
            
            self.ball = pygame.Rect(self.paddle.x + self.paddle.width // 2 - self.ball_radius,
                                self.paddle.y - self.ball_radius * 2,
                                self.ball_radius * 2,
                                self.ball_radius * 2)
            
            # Set initial game speeds based on difficulty
            if self.difficulty == 'easy':
                self.ball_speed_x = 4
                self.ball_speed_y = -4
            elif self.difficulty == 'hard':
                self.ball_speed_x = 6
                self.ball_speed_y = -6
            else:  # medium
                self.ball_speed_x = 5
                self.ball_speed_y = -5
            
            # Game variables
            self.score = 0
            self.lives = 4
            self.combos = 3
            self.level = 1
            self.rebounds = 0
            self.ball_moving = False
            self.hits_since_shrink = 0
            
            # Clear released objects
            self.released_objects = []
            
            # Create bricks
            self.bricks = self.create_bricks(self.level)
            
            # Load fonts
            self.font_title = pygame.font.Font(None, 74)
            self.font_button = pygame.font.Font(None, 36)
            self.font_small = pygame.font.Font(None, 24)

    def create_bricks(self, level):
        """Create bricks for the current level"""
        bricks = []
        rows = min(level, 8)  # Cap at 8 rows
        brick_height = 20
        brick_width = self.width // 10  # 10 bricks per row
        
        for row in range(rows):
            for col in range(10):
                if level <= 3:
                    brick_type = random.choices(['normal', 'tough', 'super'], weights=[0.8, 0.15, 0.05])[0]
                elif level <= 6:
                    brick_type = random.choices(['normal', 'tough', 'super'], weights=[0.6, 0.3, 0.1])[0]
                else:
                    brick_type = random.choices(['normal', 'tough', 'super'], weights=[0.4, 0.4, 0.2])[0]
                
                brick_info = self.BRICK_TYPES[brick_type].copy()
                rect = pygame.Rect(col * brick_width, row * brick_height + 50, brick_width - 2, brick_height - 2)
                bricks.append({'rect': rect, 'hits': brick_info['hits'], 'color': brick_info['color']})
        
        return bricks

    def user_callback(self, data):
        """Callback for receiving user information"""
        if self.current_phase == "WELCOME":
            # Now data.name, data.username, and data.age are directly accessible
            rospy.loginfo(f"Welcome {data.name}! Starting game...")
            self.username = data.username
            rospy.set_param('~user_name', data.name)
            rospy.set_param('~screen_param', 'phase2')
            self.current_phase = "GAME"
            self.initialize_game_components()

    def restart_game(self):
        """Handle game restart"""
        if self.current_phase == "FINAL":
            self.current_phase = "GAME"
            self.initialize_game_components()
            self.paused = False
            rospy.set_param('~screen_param', 'phase2')  # Update screen parameter
        else:
            self.current_phase = "GAME"
            self.initialize_game_components()
            self.paused = False
        rospy.loginfo("Game restarted")
    
    def toggle_pause(self):
        """Toggle game pause state"""
        self.paused = not self.paused
        if self.paused:
            rospy.loginfo("Game paused")
        else:
            rospy.loginfo("Game resumed")
    
    def keyboard_callback(self, data):
        """Callback for receiving keyboard controls"""
        if data.data == "RESTART":
            if self.current_phase == "FINAL":
                self.current_phase = "GAME"
                self.initialize_game_components()
                self.paused = False
                rospy.set_param('~screen_param', 'phase2')
            return
        elif data.data == "PAUSE":
            self.toggle_pause()
            return
            
        if not self.paused and self.current_phase == "GAME":
            if data.data == "LEFT" and self.paddle.left > 0:
                self.paddle.x -= self.paddle_speed
            elif data.data == "RIGHT" and self.paddle.right < self.width:
                self.paddle.x += self.paddle_speed
            elif data.data == "SPACE":
                self.ball_moving = True

    def next_level(self):
        """Handle progression to next level"""
        self.level += 1
        self.bricks = self.create_bricks(self.level)
        self.ball.x = self.paddle.x + self.paddle.width // 2 - self.ball_radius
        self.ball.y = self.paddle.y - self.ball_radius * 2
        self.ball_moving = False
        self.ball_speed_x = 5 + self.level
        self.ball_speed_y = -(5 + self.level)
        self.paddle.width = self.INITIAL_PADDLE_WIDTH
        self.paddle.x = max(0, min(self.width - self.paddle.width, self.paddle.x))
        self.hits_since_shrink = 0
        rospy.loginfo(f"Level {self.level} started!")

    def update_game(self):
            """Update game state"""
            if self.paused:
                return
            if not self.ball_moving:
                self.ball.x = self.paddle.x + self.paddle.width // 2 - self.ball_radius
                self.ball.y = self.paddle.y - self.ball_radius * 2
                return

            # Ball movement
            self.ball.x += self.ball_speed_x
            self.ball.y += self.ball_speed_y

            # Wall collisions
            if self.ball.left <= 0 or self.ball.right >= self.width:
                self.ball_speed_x = -self.ball_speed_x
            if self.ball.top <= 0:
                self.ball_speed_y = -self.ball_speed_y

            # Ball fell below paddle
            if self.ball.bottom >= self.height:
                self.handle_ball_lost()

            # Update and check released objects
            for released_object in self.released_objects[:]:
                released_object.move()
                
                # Check for collision with paddle
                if released_object.rect.colliderect(self.paddle):
                    self.lives -= 1
                    self.combos -= 1
                    self.released_objects.remove(released_object)
                    if self.lives <= 0 or self.combos < 0:
                        self.end_game()
                        return

                # Remove if off screen
                if released_object.rect.top >= self.height:
                    self.released_objects.remove(released_object)

            # Paddle collision
            if self.ball.colliderect(self.paddle):
                self.handle_paddle_collision()

            # Brick collision
            self.handle_brick_collisions()

    def handle_ball_lost(self):
        """Handle when ball goes below paddle"""
        if self.ball_moving:
            if self.combos > 0:
                self.combos -= 1
            self.lives -= 1
        
        if self.lives <= 0:
            self.end_game()
        else:
            self.reset_ball()

    def handle_paddle_collision(self):
        """Handle collision with paddle"""
        relative_intersect_x = (self.paddle.x + self.paddle.width / 2) - self.ball.centerx
        normalized_intersect = relative_intersect_x / (self.paddle.width / 2)
        bounce_angle = normalized_intersect * (5 * math.pi / 12)
        
        speed = math.sqrt(self.ball_speed_x**2 + self.ball_speed_y**2)
        self.ball_speed_x = -speed * math.sin(bounce_angle)
        self.ball_speed_y = -speed * math.cos(bounce_angle)
        
        # Handle paddle shrinking
        self.hits_since_shrink += 1
        if self.hits_since_shrink >= self.PADDLE_SHRINK_HITS:
            self.paddle.width = max(self.MIN_PADDLE_WIDTH, self.paddle.width - self.PADDLE_SHRINK_RATE)
            self.paddle.x = max(0, min(self.width - self.paddle.width, self.paddle.x))
            self.hits_since_shrink = 0
        
        # Speed up ball after certain number of rebounds
        self.rebounds += 1
        if self.rebounds % 10 == 0:
            self.ball_speed_x *= 1.1
            self.ball_speed_y *= 1.1

    def handle_brick_collisions(self):
        """Handle collisions with bricks"""
        for brick in self.bricks[:]:
            if self.ball.colliderect(brick['rect']):
                brick['hits'] -= 1
                if brick['hits'] <= 0:
                    self.bricks.remove(brick)
                    self.score += 10 * self.level
                    
                    # Randomly release an object (30% chance)
                    if random.random() < 0.3:
                        self.released_objects.append(
                            ReleasedObject(
                                brick['rect'].x + brick['rect'].width // 2 - 15,
                                brick['rect'].y + brick['rect'].height
                            )
                        )
                else:
                    brick['color'] = tuple(max(0, c - 50) for c in brick['color'])
                
                self.ball_speed_y = -self.ball_speed_y
                
                # Level progression
                if len(self.bricks) == 0:
                    self.next_level()
                break
            
    def show_game_over(self):
        """Show game over screen and handle restart"""
        overlay = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 200))
        restart_button = pygame.Rect(self.width // 2 - 100, self.height // 2 + 100, 200, 40)
        
        # Draw game over screen
        self.screen.fill(self.BACKGROUND_COLOR)
        self.screen.blit(overlay, (0, 0))
        
        # Draw game over text
        game_over_text = self.font_title.render("Game Over", True, (255, 0, 0))
        score_text = self.font_button.render(f"Final Score: {self.score}", True, (255, 255, 255))
        best_score_text = self.font_button.render(f"Best Score: {self.best_score}", True, (255, 255, 255))
        restart_info_text = self.font_small.render("Press R or click button to restart", True, (255, 255, 255))

        # Adjusted positions
        self.screen.blit(game_over_text, (self.width // 2 - game_over_text.get_width() // 2, self.height // 2 - 150))
        self.screen.blit(score_text, (self.width // 2 - score_text.get_width() // 2, self.height // 2 - 80))
        self.screen.blit(best_score_text, (self.width // 2 - best_score_text.get_width() // 2, self.height // 2 - 40))
        self.screen.blit(restart_info_text, (self.width // 2 - restart_info_text.get_width() // 2, self.height // 2 + 60))

        # Draw restart button
        pygame.draw.rect(self.screen, (0, 255, 0), restart_button)
        text_restart = self.font_small.render("Restart Game", True, (0, 0, 0))
        self.screen.blit(text_restart, (restart_button.x + 50, restart_button.y + 10))

        pygame.display.flip()

        # Check events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.MOUSEBUTTONDOWN:
                if restart_button.collidepoint(event.pos):
                    return True
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    return True

        return False

    def draw_game(self):
        """Draw game state"""
        self.screen.fill(self.BACKGROUND_COLOR)
        # Get current ball color parameter value
        self.player_color = rospy.get_param('~change_player_color', 1)
        
        # Set ball color based on parameter
        if self.player_color == 1:
            ball_color = (255, 0, 0)  # Red
        elif self.player_color == 2:
            ball_color = (186, 85, 211)  # Purple
        else:
            ball_color = (0, 0, 255)  # Blue
        
        # Draw paddle and ball
        pygame.draw.rect(self.screen, self.PADDLE_COLOR, self.paddle)
        pygame.draw.ellipse(self.screen, ball_color, self.ball)
        
        # Draw bricks
        for brick in self.bricks:
            pygame.draw.rect(self.screen, brick['color'], brick['rect'])
        
        # Draw released objects
        for released_object in self.released_objects:
            released_object.draw(self.screen)
        
        # Draw HUD
        score_text = self.font_button.render(f"Score: {self.score}", True, (255, 255, 255))
        level_text = self.font_button.render(f"Level: {self.level}", True, (255, 255, 255))
        lives_text = self.font_button.render(f"Lives: {self.lives}", True, (255, 255, 255))
        difficulty_text = self.font_button.render(f"Difficulty: {self.difficulty}", True, (255, 255, 255))
        
        self.screen.blit(score_text, (10, 10))
        self.screen.blit(level_text, (self.width - 120, 10))
        self.screen.blit(lives_text, (10, 40))
        self.screen.blit(difficulty_text, (self.width - 200, 40))
        
        # Draw combos
        for i in range(self.combos):
            pygame.draw.circle(self.screen, self.COMBO_COLOR, 
                             (30 + i * 30, self.height - 20), 10)
        
        # Draw pause indicator if paused
        if self.paused:
            pause_text = self.font_title.render("PAUSED", True, (255, 255, 255))
            text_rect = pause_text.get_rect(center=(self.width/2, self.height/2))
            self.screen.blit(pause_text, text_rect)

        pygame.display.flip()

    def end_game(self):
        """Handle game over"""
        if self.score > self.best_score:
            self.best_score = self.score
        
        # Store the score for the user
        if self.username:
            self.user_scores[self.username] = self.score
        
        # Update the screen parameter
        rospy.set_param('~screen_param', 'phase3')
        
        self.result_pub.publish(self.score)
        rospy.loginfo(f"Game Over! Final score: {self.score}")
        
        self.current_phase = "FINAL"
        
        # No need to call show_game_over here, it will be handled in the main loop

    def reset_ball(self):
        """Reset ball position"""
        self.ball.x = self.paddle.x + self.paddle.width // 2 - self.ball_radius
        self.ball.y = self.paddle.y - self.ball_radius * 2
        self.ball_moving = False
        self.ball_speed_x = 5
        self.ball_speed_y = -5

    def run(self):
        """Main game loop"""
        rate = rospy.Rate(60)  # 60Hz for smooth gameplay
        
        while not rospy.is_shutdown():
            # Process pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
            
            if self.current_phase == "GAME":
                self.update_game()
                self.draw_game()
            elif self.current_phase == "FINAL":
                if self.show_game_over():
                    self.current_phase = "GAME"
                    self.initialize_game_components()
                    self.paused = False
                    rospy.set_param('~screen_param', 'phase2')
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = GameNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        pygame.quit()