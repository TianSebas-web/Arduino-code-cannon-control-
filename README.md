This project implements an automatic cannon for firing darts at balloons using computer vision and servo control on an Arduino. The system detects balloons using a YOLO model, calculates the target's position in real time, and adjusts the cannon's servos for precise aiming and firing.

Main components:
Arduino Nano controlling:
- 360° PAN servo
- 180° TILT servo
- Modified 360° trigger servo
- Laser servo (PAN)
- Laser ON/OFF module

Python + OpenCV + YOLOv8
- Balloon detection
- Pan/tilt angle calculation
- Proportional control (KP) for smoothing movement
- Sending commands to Arduino via serial

Required Libraries
In Arduino (IDE)
- Servo.h
- Arduino.h

In Python
- opencv-python
- ultralytics (YOLO)
- pyserial
- numpy

What does the system do?
- Detects balloons in real-time video.
- Calculates the target's angular deviation.
- Automatically moves the servos to aim.
- Activates the laser when it detects a potential target.
Automatically fires when:
- The balloon is large enough (close enough)
- The target is centered in the reticle

Project files:
- turret_servo_controller.ino: Arduino firmware, interprets P/T/F/L commands.
- balloon_tracker_yolo.py: Vision, tracking, aiming, and firing system.
