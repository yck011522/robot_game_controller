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

During the daydreaming (attract) stage, they show ambient, varied visuals while the robot plays an animation. This is the "screensaver" of the installation and runs when nobody has touched a controller for a while.

During the idle (ready / waiting for player) stage, they display a message that invites the player to start the game by moving the controller upwards. There is no countdown in this stage.

During teaching stage, they show animation and interactive display with the dial to explain how the controller is used.

During game play, they show the real-time position of the dial and the current position of the robot joint similar to a gauge. Two digital needles indicate the two positions over a fixed marking, showing the full range of robotic motion (e.g. +360 to -360 for some joints). They also show the two extremes of the range in red shades, indicating their limits. If a physical collision is possible within the range, the red shades will dynamically update to indicate avoidance zone. Other telemetry data may also be displayed on the display but will serve little help to the user, they are more for interesting visuals and curiosity.

After game play during scoring, the display shows the score and a highscore leaderboard.

### **Robotic Arm and End Effector**

Each playing team controls one UR-12e robotic arm equipped with a stainless steel circular scoop mounted on the end-effector flange. The scoop is passive but contains LED lights at the bottom of the scoop, which can be used to indicate Game Start.

A pre-determined joint configuration will be used when the game starts. During gameplay, the robotic arm is controlled by the six dial controllers controlled by the players. Collision avoidance algorithms are used to avoid the robotic arm from colliding with itself or its environment.

After the game is finished, each robot retraces the collision-certified joint
targets recorded during gameplay in reverse. The rewind is retimed from path
geometry and configured robot velocity limits; it does not reuse gameplay
timing or perform collision checks during reset.

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

The match runs as a state machine. Two **independent** state variables describe the system at all times:

1. **Game stage** — where we are in the lifecycle (Daydreaming, Idle, Tutorial, Game On, Reset, Conclusion).
2. **Pause / E-stop state** — an overlay that can be active in *any* stage. It is separate from the game stage and never replaces it.

### **Stage flow**

```
Daydreaming  ⇄  Idle  →  Tutorial  →  Game On  →  Reset  →  Conclusion  →  (back to Idle)
     ▲___________│
   (idle timeout / movement)
```

- `Daydreaming ⇄ Idle` is a two-way edge: Idle drops to Daydreaming on an inactivity timeout, and any significant controller movement brings it back to Idle.
- Every other edge is one-way and ends by returning to Idle after Conclusion.

### **Pause / E-stop overlay (applies to every stage)**

The game can be paused at any time by the E-stop (software E-stop, the hardwired physical E-stop, a safety-barrier break, or a robot protective stop). The pause state is tracked by its **own variable**, independent of the game stage:

- If the current stage has a **countdown timer**, the timer **stops counting** while paused and resumes from where it left off.
- If the current stage has **no timer**, the input dials (haptic devices) simply keep **tracking their current position unchanged**, and the robot **stops moving**.
- Resuming is never automatic: once the blocking condition is clear, an operator must acknowledge (resume button / UI command) before motion continues.
- The game stage itself is preserved across a pause; only the overlay changes.

------

### **Stage 0: Daydreaming (attract mode)**

**Purpose:** Keep the installation visually alive when nobody is around. The robots play back an animation and the multi-function displays cycle through varied ambient visuals to draw an audience.
**Actions:**

- Robots run an idle animation track.
- Multi-function displays show ambient / attract visuals.
- Monitor the dials for any significant movement.

**Exit:** If any controller is moved significantly, the robots return to the ready position and we enter **Idle**.

------

### **Stage 1: Idle / Ready (wait for player)**

**Purpose:** The robots sit at a known ready position and the displays invite a player to start. This is the "ready to play" resting state.
**Actions:**

- Robots held at the ready position.
- Displays show a message prompting the player to move their controller **upwards** to begin. (Message UI not yet implemented.)
- There is **no countdown** and no on-screen counter in this stage.
- The dials keep tracking but rest near the zero position.

**Exit:**

- **To Tutorial:** if a player moves any one dial upwards by a set amount (e.g. ~360° on the controller), on either team.
- **To Daydreaming:** if Idle persists for too long without significant controller movement (e.g. ~1 minute), drop back to Daydreaming.

------

### **Stage 2: Tutorial**

**Purpose:** Show users how to play (how to control the robot). Explain the position gauge, the catch-up behaviour, the collision / out-of-bound kick feel, the game time, and the goal of moving balls.
**Duration:** ~30 second countdown.
**Actions:**

- A countdown timer (e.g. 30 s) is shown and runs down.
- The robot does **not** move during the tutorial.
- The haptic dial becomes a **scroll control** for the tutorial pages: it scrolls from a zero position up to a maximum (e.g. ~10 turns) with a dynamically changing **detent** feel, snapping to a few detents that correspond to tutorial pages.
- Multi-function displays show the how-to-play pages that the player scrolls through.

**Messages:**

- **How to control joint with controller**
  - Direction of joint controller relation to robot joint direction.
  - Meaning of the two sides of the position gauge.
    - Left marker relates to current robot position.
    - Right marker relates to dial position.
  - The robot position will slowly catch up with the dial position.
  - Collision avoidance warning / out-of-bound kick haptic feel.
- **Goal**
  - Move balls into the buckets to score; time is 2 to 4 minutes.
- **Important safety info** (physical interaction rules, emergency stop).
  - Don't go over the safety barrier.

**Exit:**

- When the countdown reaches zero, advance to **Game On**.
- (Intended, not yet implemented) The countdown can be cut short if **all** dials have been scrolled to the end (bottom/top) of the scrollable tutorial.

------

### **Stage 3: Game On**

**Actions:**

- Start **round timer** (≈3 to 4 minutes).
- Players control robotic arm joints collaboratively.
- Real-time jog motion planner running with the collision avoidance algorithm.
- Scoring based on **total weight of balls in the buckets**.

**Exit:** A countdown timer is shown on the multi-purpose screen. Advance to **Reset** when:

- the round timer reaches zero, **or**
- the **end-game button** is pressed (currently a GUI button; a physical button is planned) to cut the game short.

------

### **Stage 4: Reset (return to known position)**

**Purpose:** Bring the robots from wherever they ended up back to a **known starting position** *before* scoring is counted. Scoring (Conclusion) relies on the robot starting from this known pose.
**Actions:**

- During Game On, the measured entry pose and every certified robot joint
  target are stored in an in-memory COMPAS FAB `JointTrajectory` together with
  gameplay-relative timestamps.
- Reset creates a second reversed `JointTrajectory`. Its timing is generated
  only from joint-space geometry and a configured fraction of each robot's
  maximum joint velocity (30% in the initial hardware test profile).
- The rewind controller supplies robot targets to the game controller without
  running collision or proximity checks. Existing joint limits, pause/fault
  handling, and any configured safety interlocks remain in the command path.
- Haptic input does not control the robot during rewind. The haptic controllers
  continue tracking measured robot joint positions so players can feel and see
  the robot returning.
- Acceleration/deceleration retiming is intentionally deferred until the first
  end-to-end hardware workflow is validated.

**Exit:** When **all active robots** are measured within the configured joint
tolerance of their recorded Game On entry poses (0.5 degrees in the initial
test profile), advance to **Conclusion**. Profiles with rewind disabled retain
the legacy reset timer.

------

### **Stage 5: Conclusion (score counting)**

**Purpose:** Count and announce the score with the robot performing a scoring animation.
**Actions:**

- The robot performs a (largely pre-recorded) animation track, moving to **look at each scoring bucket in turn**.
- Count down / sum the score from each bucket and combine into a team total.
- Announce the **winner** (the team with the higher score), with lighting effects and a congratulatory robot animation.
- Show the all-time high score on the multi-purpose screen.

**Exit:** When the scoring animation completes, automatically return to **Idle**.

> Note: physical cleanup of the play field (tilting buckets to return balls to the
> shared pool, taring the load cells) happens as part of the Reset/Conclusion
> cleanup; the exact placement of the bucket-empty step is being finalised.
