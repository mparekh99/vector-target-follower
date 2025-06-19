import anki_vector 
import cv2 
import numpy as np
from PIL import Image 
from ultralytics import YOLO
import time
from anki_vector.util import degrees
import matplotlib.pyplot as plt
import math
import csv



# ### CONSTANTS

CAMERA_WIDTH = 640 # pixels
CAMERA_HEIGHT = 384  # pixels
CAMERA_CENTER_X = CAMERA_WIDTH //  2
CAMERA_CENTER_Y = CAMERA_HEIGHT // 2
FOV_HORIZONTAL = 90
FOV_VERTICAL = 50
NUM_STEPS = 5000
REAL_VECTOR_HIEGHT = 66.6  # MM
focal_length_px = (CAMERA_HEIGHT / 2) / math.tan(math.radians(FOV_VERTICAL / 2))


### Gains

k_p = 0.5 # linear gain
k_alpha = 2.0 # angular gain

 # Distance between wheels in mm
L = (5.87375 - 1.27) * 10 

## Want to be 90 mm away from Detected Robot.

TARGET_DIST = 90  ##mm

#### MODEL

model = YOLO("current.pt")



def estimate_distance(box_height_px):
    if box_height_px <= 0:
        return float('inf')

    distance_mm = (focal_length_px * REAL_VECTOR_HIEGHT) / box_height_px
    return distance_mm




def main():


    with anki_vector.Robot("00806b78") as robot:

        robot.behavior.set_head_angle(degrees(7.0))
        robot.behavior.set_lift_height(0.0)

        # Initialize Camera Feed 
        robot.camera.init_camera_feed()

        while True:

            frame_pil = robot.camera.latest_image.raw_image
            frame_np = np.array(frame_pil)
            frame = cv2.cvtColor(frame_np, cv2.COLOR_RGB2BGR)

            results = model(frame, conf=0.5)
            boxes = results[0].boxes.xywh
            classes = results[0].boxes.cls

            if boxes.shape[0] > 0:
                # Calculate areas
                areas = [w * h for (cx, cy, w, h) in boxes]
                max_idx = np.argmax(areas)

                # Get the largest box and its class
                cx, cy, w, h = boxes[max_idx]
                cls = int(classes[max_idx])
                label = model.names[cls]

                if label == 'vector':
                    delta_x = cx - CAMERA_CENTER_X

                    deg_per_pixel_h = FOV_HORIZONTAL / CAMERA_WIDTH
                    angle_to_target = delta_x * deg_per_pixel_h

                    ### DISTANCE ESTIMATION:
                    distance_mm = estimate_distance(h) - 25.4              


                    print(f'REAL ANGLE -> {angle_to_target}')
                    print(f'MOVE DIST -> {distance_mm}')


                    p = distance_mm
                    alpha = math.radians(angle_to_target)

                    if abs(alpha) > math.pi / 2:
                        # Go Backwards 
                        v = -k_p * p 
                    else:
                        v = k_p * p  # forward velocity (mm/s)

                    omega = k_alpha * alpha
                    
                    if abs(omega) < 1e-6:
                        R = 1e6  # effectively straight line
                    else:
                        R = v / omega

                    # Compute individual wheel velocities (mm/s)
                    v_r = omega * (R - (L / 2))
                    v_l = omega * (R + (L / 2))

                    robot.motors.set_wheel_motors(int(v_l), int(v_r))

                    print(f"x={cx:.1f}, y={cy:.1f}, theta={angle_to_target:.1f}°")
                    print(f"p={p:.1f}, alpha={math.degrees(alpha):.1f}°,")
                    print(f"v_l={v_l:.1f}, v_r={v_r:.1f}")
                    print("---")

                    ## PLOTTING 

                    x1 = int(cx - w / 2)
                    y1 = int(cy - h / 2)
                    x2 = int(cx + w / 2)
                    y2 = int(cy + h / 2)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                    cv2.putText(frame, f"{int(distance_mm)} mm", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)


                                        

            cv2.imshow("Vector FOV", frame)


            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.1) # ~ 10 Hz loop rate
            
        robot.motors.set_wheel_motors(0, 0) 
        cv2.destroyAllWindows()



if __name__ == '__main__':
    main()
