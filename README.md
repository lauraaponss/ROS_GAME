# RP_Pons_Laura_Cotobal_Claudia
# Brick Breaker ROS Game

This is a ROS implementation of a Brick Breaker game using multiple nodes for different functionalities.

## Dependencies

- ROS (tested on ROS Noetic)
- Python 3
- Pygame (`pip install pygame`)

## Node Structure

The game consists of five nodes:

1. **info_user_node**: Collects player information
   - Publishes to: `/user_information` (String)
   - Collects: name, username, and age

2. **game_node**: Main game logic
   - Subscribes to: `/user_information` (String), `/keyboard_control` (String)
   - Publishes to: `/result_information` (Int64)
   - Handles: game mechanics, score tracking, and game phases

3. **control_node**: Terminal-based control
   - Publishes to: `/keyboard_control` (String)
   - Controls: Arrow keys or 'a'/'d' for movement, spacebar to launch

4. **control_node_pygame**: Pygame-based control (alternative)
   - Publishes to: `/keyboard_control` (String)
   - Controls: Arrow keys for movement, spacebar to launch

5. **result_node**: Displays final results
   - Subscribes to: `/user_information` (String), `/result_information` (Int64)
   - Displays: Player information and final score

## How to Run

1. First, start roscore:
```bash
roscore
```

2. Start the result node (Terminal 1):
```bash
rosrun RP_Pons_Laura_Cotobal_Claudia result_node.py
```

3. Start the info_user node (Terminal 2):
```bash
rosrun RP_Pons_Laura_Cotobal_Claudia info_user_node.py
```

4. Start either control node (Terminal 3):
```bash
# Terminal-based control:
rosrun RP_Pons_Laura_Cotobal_Claudia control_node.py
# OR Pygame-based control:
rosrun RP_Pons_Laura_Cotobal_Claudia control_node_pygame.py
```

5. Start the game node (Terminal 4):
```bash
rosrun RP_Pons_Laura_Cotobal_Claudia game_node.py
```

## Game Controls

- Left Arrow or 'a': Move paddle left
- Right Arrow or 'd': Move paddle right
- Spacebar: Launch ball
- 'q': Quit (in terminal control mode)

## Node Communication Flow

1. `info_user_node` collects player information and publishes it
2. `game_node` receives player information and starts the game
3. `control_node` or `control_node_pygame` sends movement commands
4. `game_node` processes controls and updates game state
5. When game ends, `game_node` publishes the final score
6. `result_node` combines player information and score to display results

## Game Phases

1. **Welcome Phase**: 
   - Collects player information
   - Initializes game components

2. **Game Phase**:
   - Controls active
   - Ball and paddle movement
   - Score tracking
   - Lives and combo system

3. **Final Phase**:
   - Game over
   - Score publication
   - Result display

## Troubleshooting

If you encounter any issues:

1. Make sure all dependencies are installed
2. Verify that roscore is running
3. Check that all nodes have execute permissions:
```bash
chmod +x src/*.py
```
4. Source your workspace:
```bash
source ~/catkin_ws/devel/setup.bash
```
