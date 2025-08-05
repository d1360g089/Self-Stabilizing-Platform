# Self-Stabilizing Platform 
  - Control System Project 

This project is a dual-axis self-stabilizing platform built using an IMU sensor (BNO055), sensor fusion via complementary and quaternion filters, and PD control. The system models real-time 3D orientation using VPython and explores trade-offs in control design from bang-bang to PID.


Dual-Axis Self-Stabilizing Platform with Sensor Fusion and PD Control
This project is a dual-axis self-stabilizing platform built using an IMU sensor (BNO055), sensor fusion via complementary and quaternion filters, and PD control. The system models real-time 3D orientation using VPython and explores trade-offs in control design from bang-bang to PID.

However, the use of Euler angles introduced gimbal lock and instability due to reliance on trigonometric functions such as arctangent. To address this, I transitioned to a quaternion-based orientation representation, which improved accuracy, eliminated gimbal lock, and enabled smoother rotational tracking.

Real-time orientation of the platform was modeled using VPython, allowing for interactive 3D visualization based on sensor data. On the control side, I began with a simple constant correction (bang-bang) controller, then progressed to a proportional (P) controller. While the P controller improved performance, it introduced jitter and oscillations at larger angle deviations. To counteract this, I implemented a full PID controller. After testing, the integral component was found to be ineffective for this specific application, so the system was finalized as a PD (Proportional-Derivative) controller. The derivative term successfully dampened oscillations and improved responsiveness during rapid movements.

Through this project, I gained hands-on experience with sensor fusion algorithms, quaternion mathematics, control theory, and real-time system modeling. This project has laid a strong foundation for future work in robotics and embedded control systems, and will be followed by a self-balancing robot to further explore control dynamics in unstable systems.

## Demo Videos
link to short videos on youtube and tiktok

## 🧠 How It Works
- **Sensor:** BNO055 (gyroscope + accelerometer + magnetometer)
- **Actuators:** 2x servo motors to adjust pitch and roll
- **Filtering:** Low-pass + complementary filter for accurate orientation
- **Accuracy:** Used Quaternions for smoother motion and more accurate data
- **Control:** Proportional-Derivative Control System to keep platform level

#Files
## 🛠️ Components Used
- Arduino Uno
- BNO055 IMU
- 2x Servo motors (MG996R) 
- Breadboard, jumper wires
- Servo Brackets
  
