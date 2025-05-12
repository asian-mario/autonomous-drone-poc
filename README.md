# autonomous-drone-poc
This is a repository for the final code involved in my autonomous drone security system project for my EPQ. As of current, the drone will be able to do the following: hover, self-balancing, fly, detect and recognise faces.

### The following has been integrated into the drone:
- [x] 6-Axis PID System based off of an MPU6050 (potentially upgrading to a 9-axis IMU itf.) 
- [x] Full voltage regulation and power handling for a quadcopter-based motor setup
- [x] Command-over-WiFi system using a self-hosted ESP32 server
- [x] CNN-based face recognition (from a external server which receives FFMPEG data from a ESP32 video stream)
- [x] On-the-fly PID adjustment (CoW)
- [x] On-the-fly Trim adjustment (CoW)
- [x] On-the-fly Drone Control (CoW)
- [x] Streamed serial output from main control board to CoW Server

### The following will are either being planned or actively developed:
- [ ] Object avoidance system using a fusion of HC-SR04 sensors
- [ ] GPS Navigation using NEO-6M Module
- [ ] Sound Output
- [ ] Dedicated 'Modes'

My full report is listed [here](./report/DroneRep.pdf) and a guide will be made soon.

Components Required:
- Arduino Mega 2560 x1 (should be swapped out for a more modern board)
- ESP32
- ESP32-CAM
- MPU6050
- XT60 Male/Female Connector
- Converted-to-XT60 11.1V 3C 2600mAh Battery
- Matek 12V PDB
- A2212 1000KV DC Brushless Motors x4 (ideally swapped to a 1200KV)
- 30A ESC x4
- QAV250 Frame
- External Serial Monitor (To test IMU Data)
- Time and Patience

Instructions and prerequisities to run SFR can be found [here.](https://github.com/asian-mario/SFR-ESP32S)