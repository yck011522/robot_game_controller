# **Game Mechanics**

## **1. Team Structure**

- The game is designed on a 6 vs 6 match basis
- The game board is mostly symmetrical for both teams.

### **6 Players per Team:**

- Each player controls one joint of the robotic arm (e.g., base, shoulder, elbow, wrist 1, wrist 2 and wrist 3).
- The players in a team must coordinate their actions for the arm to perform some tasks within a time duration.

### **Team variation:**

- If fewer than 6 players, one player can control multiple joints.
- Control assignment is flexible and can be pre-set or dynamically reassigned.

## **2. Players Actions**

### **Movement**

- Rotate a dial-like controller on the desk to control the robot.
- Control the robot joints to move the scoop (which is placed at the end of the arms)

### **Interaction**

- Scoop balls from a shared pool using the scoop.
- Coordinate movement between and angles for efficient scooping.

### **Limitations & Constraints**

- Arm movement is collaborative-based.
- Misalignment can cause balls to spill or miss the bucket.
- There are soft boundaries set by the program

## **3. Setup**

### **Controllers**

Each player holds a haptic feedback dial-like controller. The absolute position of the dial can be accurately detected by the controller as an input to control a robot joint, the dial can track multiple turns. A gear ratio can be applied between the dial and the controlled joint, allowing the user to produce very fine control to the robot joints.

Because the user can rotate the dial manually very fast, while the robot can only accelerate or decelerate at a limited rate, the robots will accelerate towards the position that the user is targeting and is limited to a certain maximum speed (certain degrees per second). Therefore, the current position of the robot joint will lag behind the dial. This can be felt by the user from the haptic feedback as the dial will try to pull back towards the current position of the robot joint. As the robot joint catches up with the indicated dial position, the haptic feedback pulling force will reduce to zero.

### **Multi-function Display Panel**

A small size (10 to 14 inch) high-resolution (e.g. half HD) display will be mounted next to each of the 12 dials.

During the idle (waiting for player) stage, they display a message that invites the player to start the game by moving the controller.

During teaching stage, they show animation and interactive display with the dial to explain how the controller is used.

During game play, they show the real-time position of the dial and the current position of the robot joint similar to a gauge. Two digital needles indicate the two positions over a fixed marking, showing the full range of robotic motion (e.g. +360 to -360 for some joints). They also show the two extremes of the range in red shades, indicating their limits. If a physical collision is possible within the range, the red shades will dynamically update to indicate avoidance zone. Other telemetry data may also be displayed on the display but will serve little help to the user, they are more for interesting visuals and curiosity.

After game play during scoring, the display shows the score and a highscore leaderboard.

### **Robotic Arm and End Effector**

Each playing team controls one UR-12e robotic arm equipped with a stainless steel circular scoop mounted on the end-effector flange. The scoop is passive but contains LED lights at the bottom of the scoop, which can be used to indicate Game Start.

A pre-determined joint configuration will be used when the game starts. During gameplay, the robotic arm is controlled by the six dial controllers controlled by the players. Collision avoidance algorithms are used to avoid the robotic arm from colliding with itself or its environment.

After the game is finished, the robots will be reset to the starting position via a collision-free motion planned in real-time.

### **Ball Types**

Many circular plastic balls are filled into a pool surrounding the robotic arm. At certain orientations, the robotic arm can move the scoop close to the edges or the bottom of the pool. These fixed edges can be used to help with the scooping motion. There is no collision avoidance implemented between the arm, body, or the scoop with these balls as we will assume that the balls will move away by themselves, or are soft enough to be squished without damaging the hardware or the balls.

There will be one to two types of balls in the pool. Each type has a different weight and is signified by their color. Heavier balls will result in higher scores, but there will be only a few of them randomly distributed in the pool. Players must decide whether time spent hunting for these higher score balls is worth the effort.

### **Scoring Buckets**

Each team has two to three scoring buckets above the pool. Some buckets are positioned at easier-to-reach locations, while some of them are further away. Players must transport balls from the pool to their bucket using the robotic arm within a certain time duration (e.g. 2 to 3 mins).

Each bucket is relatively shallow and cannot hold many balls, so when the easy buckets are full, the players must aim for the more difficult buckets.

Scoring is performed by measuring the weight of balls in each bucket, scores from all buckets are combined.

After the game ends. The team with the higher score wins. The scoring buckets will be tilted to redistribute the balls back to the shared pool.

### **Score / Time Board**

During game play, a real-time scoreboard will present the real-time score of both teams to encourage competition and the remaining game time. Outside of gameplay, the scoreboard will show the highest score that was historically achieved.

## **4. Game Match Details**

### **Stage 1: Idle / Wait for player**

**Purpose:** Keep the system doing something visually interesting even when no one is around. Invites players to play, demonstrating to the audience that the robotic arm movement, the display guage and the dials are synchronised.
**Actions:**

- Monitor the dials for user input.

**Exit:** At least one (of the twelve) dial is moved beyond a certain threshold distance (e.g. 180 degrees) . Changes to stage 2.

------

### **Stage 2: Tutorial**

**Purpose:** Show users some instructions on how to play (control the robot). It may involve asking the user to perform a task on rotating the dial, and then they would also observe the position gauge moving. We also show the effects of the kicking feeling on the dial representing a collision zone or a no-go zone. Explain game time and game goal (move balls).
**Duration:** ~30 second
**Messages:**

- **How to control joint with controller**
  - Direction of joint controller relation to robot joint direction.
  - Meaning of the two sides of the position gauge.
    - Left marker relates to current robot position.
    - Right marker relates to dial position.
  - The robot position will slowly catch up with the dial position.
  - Collision avoidance warning Out-of-bound kick haptic feel
- **Goal**
  - Move balls into three buckets to score, time is 2 or 3 minutes.
- **Important safety info** (physical interaction rules, emergency stop).
  - Don’t go over safety barrier

**Basic game mechanics explained on the multi-function screen.**

**Exit:** A timer should appear and start counting down, maybe for 45 seconds or 30 seconds, and Flair is expected to go through the tutorial phase within this time. A player Is considered ready when he / she completes all the mini-tasks. After the countdown, if at least one player completes the tutorial, then we move out of this phase and the other players are assumed to be ready as well. If no player completes the tutorial, then we move back to the previous stage, assuming that visitors have left the game.



------



### **Stage 3: Game On**

**Actions:**

- Start **round timer** (2 to 3 minutes).
- Players control robotic arm joints collaboratively.
- Real-time jog motion planner running with collision avoidance algorithm
-
- Scoring based on **total weight of balls in bucket** at time expiry.

**Exit:** A timer should appear on the multipurpose screen and count down.

------



### **Stage 4: Game Conclusion**

**Actions:**

- When timer ends:
  - Animation to add up the weight of balls in all three buckets. Announce **winner** based on scoring rules.
  - Lighting effects and robot animation to congratulate the winning team.
  - Show all-time high score on the multi-purpose screen.
- Transition to **Stage 5**



------



### **Stage 5: Reset**

**Actions:**

- Reset the entire game setup to **default state**.
- Reset **robotic arm position** to starting position with a collision free motion
- Reset **all balls** back into the shared pool.
- Clear buckets and score counters.

**Exit:** After a set amount of time. (e.g. 30 secs)