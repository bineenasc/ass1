# Assignment 1 – Reactive Robot World (Gazebo Sim)

This repository contains a Gazebo Sim environment developed for **Assignment 1** of a robotics course.

The goal of this environment is to simulate a structure shaped like the number **5**, where a robot can navigate using **reactive behaviors** such as wall-following.

The world is designed so that a robot can:

1. Start outside the structure
2. Follow the outer wall
3. Enter the circular section
4. Reach the center of the circle

---

# Repository Structure

# Assignment 1 – Reactive Robot World (Gazebo Sim)

This repository contains a Gazebo Sim environment developed for **Assignment 1** of a robotics course.

The goal of this environment is to simulate a structure shaped like the number **5**, where a robot can navigate using **reactive behaviors** such as wall-following.

The world is designed so that a robot can:

1. Start outside the structure
2. Follow the outer wall
3. Enter the circular section
4. Reach the center of the circle

---

# Repository Structure
ass1
├── worlds
│ └── building_robot.sdf
├── models
│ └── cinco.stl
└── README.md


### worlds/
Contains the Gazebo world file.

building_robot.sdf


This file defines:

- the simulation world
- the ground plane
- the lighting
- the structure shaped like the number **5**

### models/
Contains 3D mesh models used in the world.

cinco.stl


This mesh represents the **number 5 structure** used in the environment.

---

# Requirements

To run this simulation you need:

- **Ubuntu / WSL**
- **Gazebo Sim (gz-sim)**

Recommended setup:

Ubuntu 22.04 + Gazebo Harmonic

---

# Installing Gazebo Sim

If Gazebo Sim is not installed, run:

```bash
sudo apt update
sudo apt install gz-sim
```

You can verify installation with:
```bash
gz sim --version
```

# Clone the Repository

Clone the repository from GitHub:
```bash
git clone https://github.com/bineenasc/ass1.git
```
Enter the project folder:
```bash
cd ass1
```
# Running the Simulation

To start the Gazebo simulation run:

```bash
gz colcon build --symlink-install
gz source install/setup.bash
gz ros2 launch comand simulation.launch.py
```

Gazebo will open the world containing the number 5 environment.

# Simulation Description

The environment consists of:

1 - A flat ground plane
2 - A structure shaped like the number 5
3 - A circular section with an entrance

The purpose of this environment is to test reactive robot navigation algorithms, such as:

- Wall following
- Obstacle avoidance
- Reactive navigation strategies

Author

Pabline (bineenasc)

GitHub:
https://github.com/bineenasc

License
This project is provided for educational purposes.
