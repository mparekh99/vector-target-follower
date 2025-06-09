# Vector Target Follower 
Enable your Anki Vector to autonomously follow another Vector robot detected in its field of view using real-time object detection and PID control. This project combines YOLO-based detection, camera geometry, and motion control to achieve smooth tracking behavior.

## Target Distance 
Using the Pinhole Camera Model:

$$
\frac{h}{f} = \frac{H}{Z} \quad \Rightarrow \quad Z = \frac{f \cdot H}{h}
$$

 
Z: Distance to the object (mm)

H: Real-world height of Vector (66.6 mm)

h: Height of bounding box (pixels)

f: Focal length (pixels), calculated from known camera FOV

## Target Angle Calculation

$$
\text{angle\_to\_target} = (c_x - \text{CAMERA\_CENTER\_X}) \cdot \left( \frac{\text{FOV\_HORIZONTAL}}{\text{CAMERA\_WIDTH}} \right)
$$


cx: Bounding box center x-coordinate

FOV_HORIZONTAL: Horizontal field of view of Vector’s camera

Converts pixel offset into angular error in degrees

## PID 
Two separate PID controllers are used:

### 1. Distance PID
Controls forward/backward motion

Units: mm → motor speed

### 2. Angle PID
Controls left/right turning

Units: degrees → motor speed

PID Formula (per controller):
$$
\text{Output} = K_P \cdot e(t) + K_I \cdot \int e(t) \, dt + K_D \cdot \frac{de(t)}{dt}
$$
**Where:**

- \( e \) is the current error  
- \( \int e \, dt \) is the integral of the error  
- \( \frac{de}{dt} \) is the derivative of the error


## Output Scalling & Clamping 
One of the main challenges was balancing output scales of the two controllers:

Distance output was in millimeters per second

Angle output was in degrees or radians per second
