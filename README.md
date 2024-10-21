# **Robot Line Following with Q-learning**
This project uses Q-learning to teach a robot to follow a line. The robot employs infrared sensors to detect the line and adjusts its movements based on feedback to stay on course. The simulation is carried out using **Webots**, a robotics simulation platform.

## **How It Works:**
The robot uses a combination of state detection and reinforcement learning to improve its behavior over time. It starts with random actions, explores the environment, and gradually optimizes its policy to follow the line more effectively.

## **Key Steps:**
1. **State Detection**: The robot detects its current state based on sensor values, determining whether it's on the line or off-track.
2. **Action Selection**: The robot chooses actions (turn left, turn right, or go straight) using Q-learning, balancing exploration and exploitation.
3. **Reinforcement Learning**: The robot receives feedback (reward or punishment) based on sensor readings, which helps adjust the Q-values.
4. **Obstacle Avoidance**: The robot can detect and avoid obstacles using its infrared sensors.

## **Watch the process:**
Below is a video showing how the robot moves after it has learned to follow the line effectively.

[Demo of the robot](Demo.mp4)

Please note that the development of this project was limited to the scope of an academic exercise. As such, the project has not undergone further refinement or optimization beyond this point.
