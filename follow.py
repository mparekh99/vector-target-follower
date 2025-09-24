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
import sys



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


### Tuned Through Trial & Error 

# Angular PID Values 

K_P = 0.10  # or 0.09

K_I = 0.002
K_D = 0.035  # or 0.04
BIAS = 0.0

# Distance PID Values

K_P_dist = 0.4
K_I_dist = 0.0
K_D_dist = 0.1
BIAS_dist = 0.0


## Want to be 90 mm away from Detected Robot.

TARGET_DIST = 90  ##mm

#### MODEL

model = YOLO("current.pt")


def to_float(x):
    # Convert tensor or other types to float safely
    return x.item() if hasattr(x, 'item') else float(x)

def move_deg_to_speed(move_deg, scale=7):
    # Scale move_deg (degrees) to wheel speed [-100, 100]
    return max(min(move_deg * scale, 100), -100)

def estimate_distance(box_height_px):
    if box_height_px <= 0:
        return float('inf')

    distance_mm = (focal_length_px * REAL_VECTOR_HIEGHT) / box_height_px
    return distance_mm

def save_logs_to_csv(filename, time_log, distance_log, angle_log, left_speed_log, right_speed_log):
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Time (s)', 'Distance (mm)', 'Angle (deg)', 'Left Speed (mm/s)', 'Right Speed (mm/s)'])
        for i in range(len(time_log)):
            writer.writerow([
                time_log[i],
                distance_log[i],
                angle_log[i],
                left_speed_log[i],
                right_speed_log[i]
            ])


def main():
    error_prev = 0
    error_dist_prev = 0
    integral = 0
    integral_dist = 0

    start_time = time.time()    # <--- Add this here
    last_time = start_time

    time_log = []
    distance_log = []
    angle_log = []
    left_speed_log = []
    right_speed_log = []



    with anki_vector.Robot("00603f86") as robot:

        robot.behavior.set_head_angle(degrees(7.0))
        robot.behavior.set_lift_height(0.0)

        # Initialize Camera Feed 
        robot.camera.init_camera_feed()

        while True:

            now = time.time()
            dt = (now - last_time)
            last_time = now


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

                    if distance_mm < 100:
                        save_logs_to_csv(
                            'vector_pid_log.csv',
                            time_log,
                            distance_log,
                            angle_log,
                            left_speed_log,
                            right_speed_log
                        )
                        print("Logs saved to vector_pid_log.csv")
                        sys.exit()  
                    

                    ## PID CONTROLLER DISTANCE MATH
                    error_dist = distance_mm - TARGET_DIST ## Target distance I need to be
                    integral_dist = integral_dist + (error_dist * dt)
                    derivative_dist = (error_dist - error_dist_prev) / dt
                    move_dist = (K_P_dist * error_dist) + (K_I_dist * integral_dist) + (K_D_dist * derivative_dist) + BIAS_dist

                    print(f'REAL DIST -> {distance_mm}')
                    print(f'PID MOVE DIST -> {move_dist}')


                    

                    # # move_dist = max(min(move_dist, 120), -120)
                    move_dist = max(min(move_dist, 150), -150)

                    # print(f'MOVE_DIST VALUE: {move_dist}')

                    error_dist_prev = error_dist


                    ## PID CONTROLLER ANGLE MATH 

                    error = angle_to_target

                    integral = integral + (error * dt)
                    derivative = (error - error_prev) / dt
                    move_deg = (K_P * error) + (K_I * integral) + (K_D * derivative) + BIAS
                    error_prev = error

                    print(f'REAL ANGLE -> {angle_to_target}')
                    print(f'PID MOVE DEGREE -> {move_deg}')

                    move_deg = max(min(move_deg, 5), -5)

                    

                    turn_speed = max(min(move_deg * 10, 100), -100)

                    

                    
                    left_speed = move_dist + turn_speed
                    right_speed = move_dist - turn_speed

                    # # Clamp speeds to motor limits
                    left_speed = max(min(left_speed, 200), -200)
                    right_speed = max(min(right_speed, 200), -200)

                    robot.motors.set_wheel_motors(int(left_speed), int(right_speed))

                    elapsed_time = now - start_time  # start_time you'll need to define before the loop

                    time_log.append(elapsed_time)
                    distance_log.append(to_float(distance_mm))
                    angle_log.append(to_float(angle_to_target))
                    left_speed_log.append(to_float(left_speed))
                    right_speed_log.append(to_float(right_speed))



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

        # save_logs_to_csv(
        #     'vector_pid_log.csv',
        #     time_log,
        #     angle_error_log,
        #     move_deg_log,
        #     dist_error_log,
        #     move_dist_log,
        #     left_speed_log,
        #     right_speed_log
        # )
        # print("Logs saved to vector_pid_log.csv")


if __name__ == '__main__':
    main()
