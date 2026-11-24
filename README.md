# Vision-Based Mobile Ball-and-Plate System

Vision-based control of a mobile ball-and-plate system using a cascade architecture:
**LQR for the ball** (outer loop, vision) + **PID for the wheels/motors** (inner loop, actuation).

The camera detects the ball, the LQR controller calculates the optimal plate tilt
to bring the ball to the center, and the PID controller drives the wheels/motors
to achieve that tilt without overshoot or oscillation.

![Prototype](figuras/Fig05_real_prototype_photos.png)

## Table of Contents
- [System Overview](#system-overview)
- [Control Architecture](#control-architecture)
- [Vision Pipeline](#vision-pipeline)
- [Repository Structure](#repository-structure)
- [How to Run](#how-to-run)
- [Experimental Results](#experimental-results)
- [Hardware](#hardware)
- [Technologies](#technologies)

## System Overview

This project is a mobile ball-and-plate system developed for EC2 - HardwareX v6.

The goal is to keep a green ball balanced at the center of a moving plate, using
only vision as the main feedback sensor for the ball position.

Main components:
- Mobile platform with tilting plate
- DC motors / servomotors for tilt actuation
- Camera for ball detection
- myRIO for real-time control and communication
- Python + OpenCV for vision and control

![CAD Prototype](figuras/Fig01_prototype_CAD_annotated.png)
![Mechanism](figuras/Fig02_mechanism_CAD_annotated.png)

## Control Architecture

This system does NOT use a single controller. It uses **two controllers in cascade**:


### 1. Outer Loop: LQR for the Ball (Vision)

The LQR controller is the brain of the system.

- Input: Ball position and velocity estimated from the camera
- State vector: `[x, y, vx, vy, theta_x, theta_y]`
- Output: Desired plate inclination `theta_x_ref`, `theta_y_ref`

LQR is used here because the ball-on-plate is a multivariable system
(2 axes coupled). LQR finds the optimal tilt that minimizes both
position error and control effort, which gives smoother and more
stable behavior than a simple PID for the ball.

![Control Architecture](figuras/Fig06_control_architecture.png)

### 2. Inner Loop: PID for Wheels / Motors

The PID controller is the muscle of the system.

- Input: Desired tilt from LQR
- Output: Motor / servo commands via UART to myRIO
- Goal: Reach the desired tilt fast, without overshoot or steady-state error

PID is used here because it is fast, robust, and ideal for direct
motor control where we need quick response and disturbance rejection.

Tested variants for the inner loop:
- PD
- PD aggressive (high gains)
- PD2
- PID with smooth integral (to remove steady-state error)

> **Summary:** LQR decides **WHAT** tilt is needed. PID decides **HOW** to move the motors to achieve it.

## Vision Pipeline

1. Camera capture
2. RGB -> HSV conversion
3. Green color segmentation by HSV thresholding
4. Morphological cleaning (opening / closing)
5. Centroid detection with `moments` / `connectedComponents`
6. Position + velocity estimation for LQR

![Vision Pipeline](figuras/Fig07_vision_pipeline.png)
![Color Detection](figuras/Fig08_color_detection.png)
![Interface](figuras/Fig09_interface.png)

## Repository Structure
