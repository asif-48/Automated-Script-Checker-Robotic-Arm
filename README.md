# Automated Script Checker Using Robotic Arm

A computer vision based automated script checking system that detects incorrect symbols on answer sheets and uses a robotic arm to mark or correct errors.

<img width="783" height="588" alt="image" src="https://github.com/user-attachments/assets/8223792f-0c0d-48c6-8294-fb5f2eb4d3ad" />


## Overview

This project combines computer vision and robotics to automate evaluation tasks.

A webcam captures an image of the answer sheet. OpenCV processes the image to detect geometric symbols such as squares, circles, and triangles. The detected positions are converted from image coordinates into real-world coordinates, which are then used to control a robotic arm for marking operations.

## Features

- Shape detection using OpenCV
- Automated error identification
- Pixel-to-coordinate transformation
- Robotic arm control using Arduino
- Inverse kinematics based movement
- Servo motor control
- Automated marking/drawing operation

## System Workflow

```
Camera Capture
      |
      ↓
Image Processing (OpenCV)
      |
      ↓
Shape Detection
      |
      ↓
Coordinate Transformation
      |
      ↓
Inverse Kinematics
      |
      ↓
Robotic Arm Movement
      |
      ↓
Marking/Correction
```

## Technologies Used

- Python
- OpenCV
- NumPy
- Arduino
- Servo Motors
- Serial Communication
- Robotics Kinematics

## Hardware

- Arduino Nano
- 3-DOF Robotic Arm
- Servo Motors
- Webcam
- Power Supply

## Software

### Computer Vision

The vision system performs:

- Image capture
- Filtering
- Thresholding
- Contour detection
- Shape classification
- Coordinate extraction

### Robotic Arm Control

The Arduino system performs:

- Servo control
- Inverse kinematics calculation
- Pen movement
- Shape drawing

## Running the Project

Install dependencies:

```bash
pip install -r requirements.txt
```

Run:

```bash
python Python/draw_circle_wrong.py
```

Connect Arduino through the configured serial port.



## Documentation

Complete project report is available in:

```
Documentation/
```

## Author

Mohammad Asiful Islam
