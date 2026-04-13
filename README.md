# Drone Autonomy System

A real-time autonomous tracking system built on a Crazyflie drone with AI-Deck, using ROS 2 for perception and control.

## Overview

The drone autonomously detects and tracks targets using its onboard camera. It streams video over WiFi to a vision node that runs inference, then a controller node translates detections into flight commands.

## Detection Strategy

The system is being upgraded from face detection to **multi-class object detection** using YOLOv8:

- **People (full body)** — pedestrian detection is a core problem in every autonomous vehicle and drone stack. Full body detection works at longer range and lower resolution than face detection, making it far more practical for real-world drone use.
- **Vehicles** — ground vehicle detection for situational awareness
- **Other drones** — airspace awareness and deconfliction

This turns the system from a single-target face tracker into an **autonomous situational awareness platform** — the kind of perception pipeline used in production autonomy stacks at companies like Waymo, Shield AI, and Skydio.

Current state uses Haar cascade face detection as a placeholder while the YOLOv8 inference node is built out (see ROADMAP.txt).

## Architecture

```
[AI-Deck Camera] --> [vision.py] --> /face_position (Point)
                                            |
                                    [controller.py]
                                            |
                                     /cmd_vel (Twist)
                                            |
                                     [Crazyflie]
```

## Hardware

- Crazyflie 2.1
- AI-Deck (OV2640 camera, 324x244)
- Host machine connected to drone WiFi (192.168.4.1)

## Running

```bash
docker compose up --build
```

Requires the drone to be powered on and your machine connected to the AI-Deck WiFi network.

## Roadmap

See `ROADMAP.txt` for the full upgrade path toward a production-grade autonomy stack.
