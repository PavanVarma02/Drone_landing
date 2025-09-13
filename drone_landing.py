# from coppeliasim_zmqremoteapi_client import RemoteAPIClient
# import time
# import math
# import cv2
# import numpy as np
# import threading

# # Connect to CoppeliaSim
# print("Connecting to CoppeliaSim...")
# client = RemoteAPIClient()
# sim = client.getObject('sim')
# print("Connected.")

# # Start simulation
# sim.startSimulation()
# time.sleep(0.5)

# # Get handles
# robot = sim.getObjectHandle('Pioneer_p3dx')
# motorLeft = sim.getObjectHandle('Pioneer_p3dx_leftMotor')
# motorRight = sim.getObjectHandle('Pioneer_p3dx_rightMotor')
# camera = sim.getObjectHandle('visionSensor')

# # Waypoint handles
# waypoint_names = ['Waypoint1', 'Waypoint2', 'Waypoint3', 'Waypoint4', 'Waypoint5', 'Waypoint6']
# waypoints = [sim.getObjectHandle(name) for name in waypoint_names]

# # Control parameters
# kp = 1.0
# maxSpeed = 2.0
# proximityThreshold = 0.3

# # Thread function for vision streaming
# def stream_vision_sensor():
#     try:
#         while True:
#             img_dict = sim.getVisionSensorImage(camera)
#             img_data = img_dict['image']
#             resolution = img_dict['resolution']

#             if img_data:
#                 img_array = np.array(img_data, dtype=np.float32)
#                 img_array = ((img_array + 1) / 2.0 * 255).astype(np.uint8)
#                 img_array = img_array.reshape(resolution[1], resolution[0], 3)
#                 img_array = cv2.flip(img_array, 0)
#                 img_array = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)

#                 cv2.imshow('Vision Sensor Stream', img_array)
#                 if cv2.waitKey(1) & 0xFF == ord('q'):
#                     break

#             time.sleep(0.03)
#     except Exception as e:
#         print(f"[Vision Thread Error] {e}")
#     finally:
#         cv2.destroyAllWindows()

# # Start vision sensor stream in a separate thread
# vision_thread = threading.Thread(target=stream_vision_sensor)
# vision_thread.start()

# # Path following loop
# for wp in waypoints:
#     while True:
#         robotPos = sim.getObjectPosition(robot, sim.handle_world)
#         robotOri = sim.getObjectOrientation(robot, sim.handle_world)
#         targetPos = sim.getObjectPosition(wp, sim.handle_world)

#         dx = targetPos[0] - robotPos[0]
#         dy = targetPos[1] - robotPos[1]
#         dist = math.sqrt(dx**2 + dy**2)

#         angleToTarget = math.atan2(dy, dx)
#         angleDiff = angleToTarget - robotOri[2]

#         # Normalize angle
#         while angleDiff > math.pi:
#             angleDiff -= 2 * math.pi
#         while angleDiff < -math.pi:
#             angleDiff += 2 * math.pi

#         speed = maxSpeed * min(dist / 1.0, 1.0)
#         vLeft = speed - kp * angleDiff
#         vRight = speed + kp * angleDiff
#         vLeft = max(min(vLeft, maxSpeed), -maxSpeed)
#         vRight = max(min(vRight, maxSpeed), -maxSpeed)

#         sim.setJointTargetVelocity(motorLeft, vLeft)
#         sim.setJointTargetVelocity(motorRight, vRight)

#         if dist < proximityThreshold:
#             break
#         time.sleep(0.05)

# # Stop robot
# sim.setJointTargetVelocity(motorLeft, 0)
# sim.setJointTargetVelocity(motorRight, 0)
# time.sleep(1)

# # Stop simulation
# sim.stopSimulation()
# print("Path following complete.")


from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math
import cv2
import numpy as np
import matplotlib.pyplot as plt

# from PnP_using_polygon import centers_2D

# Connect to CoppeliaSim
print("Connecting to CoppeliaSim...")
# client = RemoteAPIClient()
client = RemoteAPIClient(port=23000)
sim = client.getObject('sim')
print("Connected.")

measurements = []
predictions = []
predicted = []
vx_p_history = []
time_history = []
start_time = time.time()
# Start simulation
sim.startSimulation()
time.sleep(0.5)

try:
    class KalmanFilter_Ver1(object):
        def __init__(self, x, y):
            self.kf = cv2.KalmanFilter(4, 2)
            self.kf.statePre = np.array([[x], [y], [0], [0]], dtype=np.float32)
            self.kf.statePost = np.array([[x], [y], [0], [0]], dtype=np.float32)
            dt = 0.1  # time step

            self.kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                                  [0, 1, 0, 0]], np.float32)

            self.kf.transitionMatrix = np.array([[1, 0, dt, 0],
                                                 [0, 1, 0, dt],
                                                 [0, 0, 1, 0],
                                                 [0, 0, 0, 1]], np.float32)

            self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-4
            self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1

        def update(self, coordX, coordY):
            """Update step with measured coordinates"""
            measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
            self.kf.correct(measured)

        def predict(self, coordX, coordY):
            """Predict next state"""
            predicted = self.kf.predict()
            x, y = int(predicted[0].item()), int(predicted[1].item())
            return x, y


    # Get handles
    robot = sim.getObjectHandle('/PioneerP3DX')
    motorLeft = sim.getObjectHandle('/leftMotor')
    motorRight = sim.getObjectHandle('/rightMotor')
    camera = sim.getObjectHandle('/visionSensor')
    propeller = sim.getObjectHandle('/propeller')
    drone = sim.getObjectHandle('/Quadcopter')

    cx = 320
    cy = 320

    # Waypoint handles
    waypoint_names = ['Waypoint1', 'Waypoint2', 'Waypoint3', 'Waypoint4', 'Waypoint5', 'Waypoint6','Waypoint7','Waypoint8','Waypoint9' ]
    waypoints = [sim.getObjectHandle(name) for name in waypoint_names]
    # joint_names = ['propeller[0]', 'propeller[1]', 'propeller[2]', 'propeller[3]']
    #
    # # Get joint handles
    # joint_handles = [sim.getObject(name) for name in joint_names]
    #
    #
    # # Function to convert RPM to rad/s
    # def rpm_to_rad_per_sec(rpm):
    #     return (rpm * 2 * math.pi) / 60
    #
    #         # Example RPM values (you can change this dynamically)
    # rpm_values = [4000, 4000, 4000, 4000]  # Modify as needed
    #
    # for i, handle in enumerate(joint_handles):
    #     target_velocity = rpm_to_rad_per_sec(rpm_values[i])
    #     sim.setJointTargetVelocity(handle, target_velocity)
    #
    #     time.sleep(0.05)
    # # Control parameters
    kpr = 1.0
    maxSpeed = 1
    proximityThreshold = 0.2
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    aruco_params = cv2.aruco.DetectorParameters()
    # def aruco_marker_prediction(markerCorner):
    #     corners_r = markerCorner.reshape((4, 2))
    #     (topLeft, topRight, bottomRight, bottomLeft) = corners_r
    #     p1_integrated = int(topLeft[0] + (topRight[0] - topLeft[0]) / 2 + 0.5)
    #     p2_integrated = int(topLeft[1] + (bottomLeft[1] - topLeft[1]) / 2 + 0.5)
    #     print(p1_integrated, p2_integrated)
    # Path following loop
    dt = 0.2
    vx0i = 0
    vy0i = 0
    vx0d = 0
    vy0d = 0
    vx_d_filtered_prev = 0
    vy_d_filtered_prev = 0
    start_time_for_move= 0
    for wp in waypoints:
        while True:
            current_time = time.time() - start_time

            robotPos = sim.getObjectPosition(robot, sim.handle_world)
            robotOri = sim.getObjectOrientation(robot, sim.handle_world)
            targetPos = sim.getObjectPosition(wp, sim.handle_world)

            dx = targetPos[0] - robotPos[0]
            dy = targetPos[1] - robotPos[1]
            dist = math.sqrt(dx**2 + dy**2)

            angleToTarget = math.atan2(dy, dx)
            angleDiff = angleToTarget - robotOri[2]

            # Normalize angle
            while angleDiff > math.pi:
                angleDiff -= 2 * math.pi
            while angleDiff < -math.pi:
                angleDiff += 2 * math.pi

            speed = maxSpeed * min(dist / 1.0, 1.0)
            vLeft = speed - kpr * angleDiff
            vRight = speed + kpr * angleDiff
            vLeft = max(min(vLeft, maxSpeed), -maxSpeed)
            vRight = max(min(vRight, maxSpeed), -maxSpeed)

            sim.setJointTargetVelocity(motorLeft, vLeft)
            sim.setJointTargetVelocity(motorRight, vRight)

            # --- Vision Sensor Stream Live ---
            # img_data = sim.getVisionSensorImage(camera)
            # res = sim.getVisionSensorResolution(camera)
            # Get image and resolution using the new method
            img, resolution = sim.getVisionSensorImg(camera)
            resX, resY = resolution

            if img:
                img_array = np.frombuffer(img, dtype=np.uint8)
                img_array = img_array.reshape((resY, resX, 3))
                img_array = cv2.flip(img_array, 0)
                img_array = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)

                # cv2.imshow('Vision Sensor Stream', img_array)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    raise KeyboardInterrupt
                centers_2D={}
                # Detect ArUco markers
                corners, ids, _ = cv2.aruco.detectMarkers(img_array, aruco_dict, parameters=aruco_params)

                if ids is not None:
                    print("Detected IDs:", ids.flatten())  # Debug print

                    for i, marker_id in enumerate(ids.flatten()):
                        if marker_id == 2:
                            c = corners[i][0]
                            cxm = int(np.mean(c[:, 0]))
                            cym = int(np.mean(c[:, 1]))
                            centers_2D[marker_id] = (cxm, cym)
                            # print(cxm, cym)
                            cv2.circle(img_array, (cxm, cym), 5, (0, 0, 255), -1)
                            cv2.putText(img_array, f"ID:{marker_id}", (cxm + 10, cym - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                            if marker_id == 2:
                                kf = KalmanFilter_Ver1(cxm, cym)
                                # kf.update(cxm, cym)
                                predicted = kf.predict(cxm, cym)


                                # print(predicted)

                            dx = predicted[0] - cx
                            dy = predicted[1] - cy
                            drone_pos = sim.getObjectPosition(drone, sim.handle_world)
                            landing_rate_0=0.12
                            # if 0<=dx<=cx/8 and 1.5<drone_pos[2]<2.5:
                            #     new_z = drone_pos[2] - landing_rate
                            #     k_px = 0.0015
                            #     k_py = 0.0015
                            #
                            # elif cx/8<dx<=cx/4 and 1< drone_pos[2]<1.5:
                            #     new_z = drone_pos[2] - landing_rate
                            #     k_px = 0.0015
                            #     k_py = 0.0015
                            # elif cx/4<dx<=cx/2:
                            #     k_px = 0.001
                            #     k_py = 0.001
                            # elif dx>cx/2:
                            #     k_px = 0.0008
                            #     k_py = 0.0008
                            #
                            # if 0<=dy<=cx/8:
                            #     k_px = 0.0015
                            #     k_py = 0.0015
                            #
                            # elif cx/8<dy<=cx/4:
                            #     k_px = 0.0015
                            #     k_py = 0.0015
                            # elif cx/4<dy<=cx/2:
                            #     k_px = 0.001
                            #     k_py = 0.001
                            # elif dy>cx/2:
                            #     k_px = 0.0008
                            #     k_py = 0.0008
                            err_xy = math.sqrt(dx ** 2 + dy ** 2)
                            hover_done = False
                            if drone_pos[2] > 3:
                                # if start_time_for_move is None:
                                #     start_time_for_move = current_time
                                # t = current_time - start_time_for_move
                                #
                                # td = 3.0  # total duration
                                # x0 = drone_pos[0]  # initial drone x position
                                # xd = cxm  # desired x (marker position)
                                # yd = cym
                                # tau = t / td  # normalized time (0 to 1)
                                #
                                # new_x = x0 + 3 * (xd - x0) * (tau ** 2) - 2 * (xd - x0) * (tau ** 3)
                                # new_y = y0 + 3 * (yd - x0) * (tau ** 2) - 2 * (yd - x0) * (tau ** 3)


                                # High phase descent
                                new_z = drone_pos[2] - landing_rate_0

                                if dx > 160:
                                    k_px = 0.0009
                                elif 80< dx < 160:
                                    k_px = 0.0007
                                elif 20 < dx < 80:
                                    k_px= 0.0005
                                else:
                                    k_px= 0.0008

                                if dy> 160:
                                    k_py = 0.0009
                                elif 80< dy < 160:
                                     k_py = 0.0007
                                elif 20 <  dy < 80:
                                     k_py = 0.0005
                                else:
                                    k_py = 0.0003

                                k_dx = k_dy = 0.00018
                                k_ix = k_iy = 0.00018



                                # k_px = 0.00001
                                # k_py = 0.00001
                                # k_dx = 0.0001
                                # k_dy = 0.0001
                                # k_ix = 0.0001
                                # k_iy = 0.0001

                                # k_px = 0.00001
                                # k_py = 0.00001
                                # k_dx = 0.0001
                                # k_dy = 0.0001
                                # k_ix = 0.0001
                                # k_iy = 0.0001
                                print('height', drone_pos[2])
                                print( dx,dy)
                                print('total_error', err_xy)

                            elif 1.5 < drone_pos[2] <= 3:
                                # if not hover_done and abs(drone_pos[2] - 3.5) < 0.05:
                                #     print("Hovering at 3.5 m...")
                                #     time.sleep(1)  # Hover for 1 second
                                #     hover_done = True

                                # High phase descent
                                # landing_rate_1 = 0.12
                                if abs(dx) < 30 or abs(dy) < 30 :
                                    landing_rate_1 = 0.1
                                    print('landing_rate_1', landing_rate_1)
                                else:
                                    landing_rate_1 = 0.08

                                new_z = drone_pos[2] - landing_rate_1

                                if dx > 160:
                                    k_px = 0.0008
                                elif 80 < dx < 160:
                                    k_px = 0.0004
                                elif 20 < dx < 80:
                                    k_px = 0.0002
                                else:
                                    k_px = 0.0001

                                if dy > 160:
                                    k_py = 0.0009
                                elif 80 < dy < 160:
                                    k_py = 0.0004
                                elif 20 < dy < 80:
                                    k_py = 0.0002
                                else:
                                    k_py = 0.0001

                                k_dx = k_dy = 0.0003
                                k_ix = k_iy = 0.0003

                                print('height',drone_pos[2])
                                print(dx, dy)
                                print('total_error', err_xy)

                            # elif 1.5 <drone_pos[2]<2:
                            #     if abs(dx) < 30 or abs(dy) < 30 :
                            #         landing_rate_1 = 0.1
                            #         print('landing_rate_1', landing_rate_1)
                            #     else:
                            #         landing_rate_1 = 0.08
                            #     new_z = drone_pos[2] - landing_rate_1
                            #     # k_px = 0.000007
                            #     # k_py = 0.000007
                            #     # k_dx = 0.00001
                            #     # k_dy = 0.00001
                            #     # k_ix = 0.00001
                            #     # k_iy = 0.00001
                            #
                            #     if err_xy > 240:
                            #
                            #         k_px, k_py = 0.0009, 0.0009
                            #
                            #     elif 160 < err_xy < 240:
                            #
                            #         k_px, k_py = 0.0009, 0.0009
                            #     elif 80 < err_xy < 160:
                            #
                            #         k_px, k_py = 0.0004, 0.0004
                            #
                            #     elif 20 < err_xy < 80:
                            #
                            #         k_px, k_py = 0.0002, 0.0002
                            #
                            #     else:
                            #
                            #         k_px, k_py = 0.0001, 0.00001
                            #
                            #     k_dx = k_dy = 0.0002
                            #
                            #     k_ix = k_iy = 0.0001
                            #
                            #     print('height', drone_pos[2])
                            #     print(dx, dy)
                            #     print('total_error', err_xy)

                            elif 0.8 <drone_pos[2]<1.5:
                                if abs(dx) < 15 or abs(dy) < 15 :
                                    landing_rate_1 = 0.08
                                    print('landing_rate_1', landing_rate_1)
                                else:
                                    landing_rate_1 = 0.08
                                new_z = drone_pos[2] - landing_rate_1
                                # k_px = 0.000007
                                # k_py = 0.000007
                                # k_dx = 0.00001
                                # k_dy = 0.00001
                                # k_ix = 0.00001
                                # k_iy = 0.00001

                                if err_xy > 240:

                                    k_px, k_py = 0.0004, 0.0004

                                elif 160 < err_xy < 240:

                                    k_px, k_py = 0.0004, 0.0004
                                elif 80 < err_xy < 160:

                                    k_px, k_py = 0.008, 0.0008

                                elif 20 < err_xy < 80:

                                    k_px, k_py = 0.0006, 0.0006

                                else:

                                    k_px, k_py = 0.0004, 0.0004

                                k_dx = k_dy = 0.0003

                                k_ix = k_iy = 0.0003

                                print('height', drone_pos[2])
                                print(dx, dy)

                            elif 0.5 < drone_pos[2] < 0.8:
                                landing_rate = 0.05
                                new_z = drone_pos[2] - landing_rate

                            # Check if we have reached ~0.3 m height
                                if abs(drone_pos[2]) == 0.3:  # ±2 cm tolerance
                                    print("Landing...")
                                    new_z = 0.0  # Touchdown

                                k_px = 0.0004
                                k_py = 0.0004
                                k_dx = 0.0001
                                k_dy = 0.0001
                                k_ix = 0.0001
                                k_iy = 0.0001

                            vx_p = k_px * dx  # forward/backward control
                            vy_p = k_py * dy  # left/right control

                            vx0i = vx0i + dx * dt
                            vx_i = vx0i * k_ix
                            vy0i = vy0i + dy * dt
                            vy_i = vy0i * k_iy
                            i_max= 50
                            vx0i = max(min(vx0i, i_max), -i_max)
                            vy0i = max(min(vy0i, i_max), -i_max)

                            vx_d = k_dx * (vx0d- dx) / dt
                            vx0d = dx
                            vy_d = k_dy * (vy0d -dy) / dt
                            vy0d = dy


                            alpha = 0.3  # Filter coefficient
                            vx_d_filtered = alpha * vx_d_filtered_prev + (1 - alpha) * vx_d
                            vy_d_filtered = alpha * vy_d_filtered_prev + (1 - alpha) * vy_d

                            vx = (k_px * dx + vx_i + vx_d)
                            vy = -(k_py * dy + vy_i + vy_d)
                            print(vx,  vy )
                            vx_p_history.append(vx_p)
                            time_history.append(current_time)
                            # Uncomment the line below to use the animation version instead:
                            # ani = animate_version()
                            new_x = drone_pos[0] + vx
                            new_y = drone_pos[1] + vy
                            # new_z =  vz
                            landing_threshold_xy = 20  # Pixel threshold to consider aligned for final landing
                            # landing_rate = 0.02  # Descent rate per frame
                            min_z = 0.1  # Hover height before final landing
                            landing_z = 0.1  # Final landing height
                            align_trigger_threshold = 40  # Begin descent until dx < this

                            #
                            # if abs(dx)<= landing_threshold_xy:
                            #
                            #     sim.setObjectPosition(drone, sim.handle_world, [new_x, new_y, new_z])
                            # Update drone position
                            sim.setObjectPosition(drone, sim.handle_world, [new_x, new_y, new_z])
                            # ---------- LANDING CONTROL ----------
                            # If close to marker (dx, dy small), allow descent

                            cv2.imshow('Vision Sensor Stream', img_array)
                else:
                    print("No markers detected.")

            if dist < proximityThreshold:
                break

            time.sleep(0.05)

        #
        # def plot_vx_p_results():
        #     plt.figure(figsize=(12, 8))
        #
        #     # Main plot
        #     plt.subplot(2, 1, 1)
        #     plt.plot(time_history, vx_p_history, 'b-', linewidth=2, label='vx_p')
        #     plt.xlabel('Time (seconds)')
        #     plt.ylabel('vx_p (velocity)')
        #     plt.title('vx_p vs Time')
        #     plt.grid(True, alpha=0.3)
        #     plt.legend()
        #
        #     # Statistical subplot
        #     plt.subplot(2, 1, 2)
        #     plt.hist(vx_p_history, bins=50, alpha=0.7, color='skyblue', edgecolor='black')
        #     plt.xlabel('vx_p values')
        #     plt.ylabel('Frequency')
        #     plt.title('Distribution of vx_p values')
        #     plt.grid(True, alpha=0.3)
        #
        #     plt.tight_layout()
        #     plt.savefig('vx_p_analysis.png', dpi=300, bbox_inches='tight')
        #     plt.show()
        #
        # plot_vx_p_results()

    # Stop the robot
    sim.setJointTargetVelocity(motorLeft, 0)
    sim.setJointTargetVelocity(motorRight, 0)
    time.sleep(1)

except KeyboardInterrupt:
    print("Stopped by user.")

except Exception as e:
    print(f"❌ Error occurred: {e}")

finally:
    cv2.destroyAllWindows()
    sim.stopSimulation()
    print("Simulation stopped.")
    # if len(vx_p_history) > 0:
    #     plot_vx_p_results()