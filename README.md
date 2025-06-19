# Vector Target Follower

Enable your Anki Vector robot to autonomously follow another Vector in its field of view using real-time object detection and control systems. This project integrates YOLO-based vision, camera geometry, and both PID and feedback state control to create smooth, reactive tracking behavior.

---

[![Watch the video](https://img.youtube.com/vi/VU0lt7MGkOM/hqdefault.jpg)](https://www.youtube.com/watch?v=VU0lt7MGkOM)




## Project Output

The follower Vector:
- Detects the target Vector in its camera feed using a custom-trained YOLO model
- Estimates the **distance** and **angle** to the target in real-time
- Uses PID and kinematic control to adjust its movement and orientation
- Follows the target while maintaining a configurable distance

---

## Target Distance Estimation

### 1. Focal Length Calculation

Focal length is computed using the vertical field of view (FOV) and image height (in pixels) — intentionally using height over width due to the shape of the Vector and stability under rotation.

**Why focal length matters:**  
It’s the critical parameter for estimating real-world distances from image measurements using the pinhole camera model. It represents the distance from the lens to the image plane.

### 2. Distance via Pinhole Camera Model

The formula used:

```
distance_mm = (focal_length_px * real_object_height_mm) / bounding_box_height_px
```
This formula, based on the pinhole camera model, is computationally efficient and accurate within ±2 cm in real-time performance.

---

## Target Angle Calculation

To estimate the angle between the target and camera’s center:
```
deg_per_pixel = FOV_HORIZONTAL / CAMERA_WIDTH
angle_to_target = delta_x * deg_per_pixel
```

This method offers a fast, robust mapping from image space to angular displacement, ideal for real-time control.

---

## Control Systems


### Initial Approach: PID Controllers
Two PID controllers were used to manage tracking:

- **Distance PID**: Controls forward/backward motion  
- **Angular PID**: Controls rotation to face the target

### Lessons Learned
- Proportional, Integral, and Derivative tuning is essential
- Output scaling and clamping were added to smooth responses and reduce command imbalance — however, this introduced unnecessary complexity and sometimes interfered with the full output range of the PID controller 
  
## Redesigned Controller: Kinematic Feedback State Control

To improve trajectory tracking and smoothness, I redesigned the system using **differential drive kinematics** and **kinematic position control** based on a simplified **feedback control law**. Allowing me to have wheel velocities already scaled. This approach allowed for **directly scaled wheel velocities**, eliminating the need for additional PID output scaling and reducing system complexity.



### System Overview 
Using differential Drive Overview:
The robot’s motion is modeled as a unicycle, and wheel velocities are calculated using:

```
v_r = w × (R + L / 2)
v_l = w × (R - L / 2)
```
Where 
- v_r, v_l are the right and left wheel velocities
- w is the angular velocity
- R is the instantaneous turning radius
- L is the track width, computed as:

```
L = (tread_to_tread - tread_width) × 10  # in mm
```
This formulation maps angular velocity and turning radius into individual wheel speeds using robot geometry.

## Linear Speed and Radius Relationship

The relationship between **linear velocity (v)** and **angular velocity (w)** is given by:
```
v = w × R
```
Solving for the radius:
```
R = v / w
```
Where:
- v is the robot’s linear velocity
- w is the angular velocity
- R is the distance from the Instantaneous Center of Curvature (ICC) to the midpoint between the wheels (i.e., turn radius)

## Control Law 
```
v = k_p × ρ
w = k_α x α + k_b x b
```
Where:
- ρ is the distance to the target
- α is the angle to the target
- k_p is the linear gain
- k_a is the angular gain

Dropped b and k_b because will handle orientation later. 

## Final Wheel Velocity Computation
Once v and w are computed using the control law, they are plugged back into the differential drive equations:
```
v_r = v + (w × L / 2)
v_l = v - (w × L / 2)
```
This produces real-time wheel velocity commands that enable smooth, curved tracking behavior toward the moving target.

