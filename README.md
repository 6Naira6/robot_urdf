# **ROS 2 Gazebo and Move Robot Package**

## **Overview**
This package integrates a simulated robot in Gazebo with a Python node to control its motion. The package:
1. Spawns the robot in a Gazebo environment.
2. Publishes the robot’s state using the Robot State Publisher.
3. Allows users to interactively control the robot's movement by setting linear and angular velocities via the `move_robot.py` node.
4. Provides position feedback from the `/odom` topic to track the robot’s location in the simulation.

---

## **Installation**

### **Step 1: Clone the Repository**
Clone the repository into your ROS 2 workspace:
```bash
cd ~/workspace/src
git clone https://github.com/6Naira6/robot_urdf.git
```

### **Step 2: Build the Workspace**
Build the ROS 2 workspace:
```bash
cd ~/workspace
colcon build
source install/setup.bash
```

---

## **How to Launch**

### **Launch the Simulation and Control Node**
Run the provided launch file to start Gazebo and RViz:
```bash
ros2 launch robot_urdf gazebo.launch.py
```
Then run the `move_robot.py` script:
```bash
ros2 run robot_urdf move_robot.py
```

### **Control the Robot**
Interact with the `move_robot.py` script by entering linear and angular velocities when prompted in the terminal. For example:
```plaintext
Enter linear velocity (m/s): 0.5
Enter angular velocity (rad/s): 0.2
```

The robot will move based on the specified velocities.

### **Monitor Robot Position**
The script logs the robot's position in the terminal by subscribing to the `/odom` topic:
```plaintext
Current Position -> x: 1.5, y: 0.8
```

---

## **Proof of Work**
![Demo GIF](work.gif)
