# RP_Pons_Laura_Cotobal_Claudia
# Brick Breaker ROS Game

This is a ROS implementation of a Brick Breaker game using multiple nodes for different functionalities.

## Dependencies

- ROS (tested on ROS Noetic)
- Python 3
- Pygame (`pip install pygame`)
- * xterm (`sudo apt-get install xterm`) - Required for launch file
  * 
## Node Structure
The game consists of five nodes:
1. **info_user_node**: Collects player information
   * Publishes to: `/user_information` (String)
   * Collects: name, username, and age

2. **game_node**: Main game logic
   * Subscribes to: `/user_information` (String), `/keyboard_control` (String)
   * Publishes to: `/result_information` (Int64)
   * Provides services:
     * `/difficulty`: Change game difficulty (easy/medium/hard)
     * `/user_score`: Get user's score percentage
   * Parameters:
     * `change_player_color`: Change ball color (1:Red, 2:Purple, 3:Blue)
     * `user_name`: Store user's name
     * `screen_param`: Show game phase

3. **control_node**: Terminal-based control
   * Publishes to: `/keyboard_control` (String)
   * Controls: Arrow keys or 'a'/'d' for movement, spacebar to launch

4. **control_node_pygame**: Pygame-based control (alternative)
   * Publishes to: `/keyboard_control` (String)
   * Controls: Arrow keys for movement, spacebar to launch

5. **result_node**: Displays final results
   * Subscribes to: `/user_information` (String), `/result_information` (Int64)
   * Uses service: `/user_score` to get score percentage
   * Displays: Player information and final score

## How to Run

### Using Launch File (Recommended)
Start all nodes with:
```bash
roslaunch RP_Pons_Laura_Cotobal_Claudia game.launch
```

### Manual start
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
rosrun RP_Pons_Laura_Cotobal_Claudia info_user.py
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

Game Features and Controls

Basic Controls
* Left Arrow or 'a': Move paddle left
* Right Arrow or 'd': Move paddle right
* Spacebar: Launch ball
* 'q': Quit (in terminal control mode)

Game Customization
* Difficulty Settings (Available only during welcome phase)
  * Easy: Slower ball speed and gentler paddle shrinking
  * Medium: Standard game settings
  * Hard: Faster ball speed and aggressive paddle shrinking
  * Command to change:
    ```
    rosservice call /difficulty "change_difficulty: 'easy'"    # or 'medium' or 'hard'
    ```

* Ball Color Customization (Available anytime during gameplay)
  * Red (Default): Parameter value 1
  * Purple: Parameter value 2
  * Blue: Parameter value 3
  * Command to change:
    ```
    rosparam set /game_node/change_player_color VALUE  # Replace VALUE with 1, 2, or 3
    ```

Node Communication Flow
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
