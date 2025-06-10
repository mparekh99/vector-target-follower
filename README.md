# Vector Target Follower

Enable your Anki Vector robot to autonomously follow another Vector in its field of view using real-time object detection and PID control. This project integrates YOLO-based vision, camera geometry, and control systems to create smooth, reactive tracking behavior.

---
![Following - Made with Clipchamp](https://github.com/user-attachments/assets/c50e69d4-0c0a-4ea4-8682-29ea61af18b4)
![Follow - Made with Clipchamp](https://github.com/user-attachments/assets/e95f828d-936f-47dc-9d96-41288d748870)
![Untitled video - Made with Clipchamp](https://github.com/user-attachments/assets/141f2aa0-8b17-4557-8b4e-d803dbcacd7f)
![Stationary](https://github.com/user-attachments/assets/8039db18-4671-4311-8824-1ea446194c7e)




## Project Output

The follower Vector:
- Detects the target Vector in its camera feed using YOLO
- Estimates the **distance** and **angle** to the target in real-time
- Uses **PID control** to smoothly adjust its movement and orientation
- Follows the target robot while maintaining a configurable following distance

---

## Target Distance Estimation

### 1. Focal Length Calculation

I calculate the focal length using the vertical field of view (FOV) and image height (in pixels), not width. This is intentional:

- The Anki Vector robot is longer than it is wide.
- Bounding box height changes more consistently with distance than width does — especially when the target rotates.
- Therefore, **height** provides a more stable feature for estimating distance.

**Why focal length matters:**  
It’s the critical parameter for estimating real-world distances from image measurements using the pinhole camera model. It represents the distance from the lens to the image plane.

### 2. Distance via Pinhole Camera Model

The formula used:

```
distance_mm = (focal_length_px * real_object_height_mm) / bounding_box_height_px
```
This is derived from the **pinhole camera model**, which describes how a 3D point `(X, Y, Z)` projects onto a 2D image plane.

### Why This Model?

- Mathematically simple
- Requires low computational overhead
- Delivers reasonably accurate results in real-time (±2 cm error)
- Avoids the latency of heavier ML-based methods like regression models

---

## Target Angle Calculation

To estimate the **angle** between the camera's center line and the detected target:

1. Compute the center x-coordinate of the bounding box.
2. Calculate `delta_x` — the pixel offset from the image center.
3. Multiply `delta_x` by degrees per pixel:

```
deg_per_pixel = FOV_HORIZONTAL / CAMERA_WIDTH
angle_to_target = delta_x * deg_per_pixel
```

## Why This Angle Estimation Works So Well

- A direct, linear mapping from pixels to angle  
- Each pixel represents a fixed angular step  
- Fast and robust — especially for fixed-lens cameras like Vector’s  

This makes it an ideal solution for real-time tracking where performance and simplicity matter.

---

## PID Controllers

Two separate **PID controllers** manage the robot’s tracking behavior:

- **Distance PID**: Controls forward/backward motion to maintain a ~90 mm distance from the target  
- **Angular PID**: Controls rotation to keep the robot oriented toward the target

### What I Learned About PID Control

Even simple-looking control systems require proper tuning:

- `P` (Proportional): Responds to current error  
- `I` (Integral): Accumulates past error  
- `D` (Derivative): Predicts future error based on rate of change  

Tuning these parameters helped reduce oscillations and ensured smoother, more stable movement toward the target.

---

## Output Scaling & Clamping

### Challenge

The two PID controllers operate on different units:

- **Distance PID** → millimeters/second  
- **Angular PID** → degrees or radians/second  

This caused an **imbalance**, where distance control dominated the movement and angular corrections were underpowered, making the robot slow to turn toward the target.

### My Solution

- Manually scaled the angular PID output based on runtime behavior  
- Added clamping to prevent excessive motor commands or jitter  
- The system is still being fine-tuned, but the robot now moves and turns smoothly and consistently

---

## Summary

This project deepened my understanding of:

- Camera geometry and projection  
- The pinhole camera model  
- PID control theory and tuning  
- Real-time performance optimization  
- Differential drive kinematics  

While not perfect, the Vector follower reliably tracks and follows a target robot with smooth, real-time behavior. Ongoing refinements will improve responsiveness and stability even further.

---

## Technologies Used

- YOLO object detection (custom-trained)
- Python & OpenCV
- Real-time camera stream processing
- PID control systems



