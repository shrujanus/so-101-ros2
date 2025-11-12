# 🤖 SO-101 ROS2 — Open-Source General Purpose Manipulator  
### Natural Language → VLM/VLA → MoveIt2 → Robot Action

---

## 🚀 Overview

**SO-101 ROS2** is an open-source project to build a general-purpose robotic manipulator capable of performing **pick and place tasks through natural language commands**.

Example:

> "Pick up the screwdriver and place it in the blue bin."

This repo contains:

| Component | Description |
|----------|-------------|
| `ros2_ws/` | ROS2 workspace with robot description, MoveIt2 configuration, controllers and hardware drivers |
| `action_bridge/` | Translates VLM/VLA output (action tokens) into MoveIt2 ROS commands |
| `training/` | Vision-Language(-Action) training pipeline using PyTorch + LeRobot |

The goal is to bridge **language → perception → motion planning → execution**.

---

## 🧠 Architecture

