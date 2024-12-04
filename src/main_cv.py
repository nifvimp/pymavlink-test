import math
import cv2
import socket
import struct
import time

from threading import Thread

import numpy as np

from dronekit import connect, VehicleMode, LocationGlobalRelative
from typing import Tuple

MAP_ORIGIN = -35.3632621, 149.1652374
EARTH_RADIUS = 6371000.0
ARDUPILOT_PORT = 14550
CAMERA_PORT = 5599


def camera_stuff():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("127.0.0.1", 5599))
    global circle_found
    circle_found = False

    header_size = struct.calcsize("=HH")
    while True:
        # receive header
        header = s.recv(header_size)
        if len(header) != header_size:
            print("Header size mismatch")
            break

        # parse header
        width, height = struct.unpack("=HH", header)

        # receive image
        bytes_to_read = width * height
        img = bytes()
        while len(img) < bytes_to_read:
            img += s.recv(min(bytes_to_read - len(img), 4096))

        # Do cool stuff with the image here
        # convert incoming bytes to a numpy array (a grayscale image)
        img = np.frombuffer(img, np.uint8).reshape((height, width))
        img = np.array(img)

        # Apply Gaussian Blur to reduce noise and improve circle detection
        blurred_img = cv2.GaussianBlur(img, (9, 9), 2)

        # Use Hough Circle Transform to detect circles
        circles = cv2.HoughCircles(blurred_img,
                                   cv2.HOUGH_GRADIENT,
                                   dp=1,
                                   minDist=20,
                                   param1=50,
                                   param2=30,
                                   minRadius=1,
                                   maxRadius=100)

        # If some circles are detected, draw them on the image
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                # Draw the outer circle
                cv2.circle(img, (i[0], i[1]), i[2], (255, 0, 0), 2)  # Blue circle
                # Draw the center of the circle
                cv2.circle(img, (i[0], i[1]), 2, (0, 255, 0), 3)  # Green center

            if not circle_found:
                print("Circle Found!")
                circle_found = True

        cv2.imshow("image", img)
        if cv2.waitKey(1) == ord("q"):
            break


def convert_to_map_coordinates(global_relative_frame: LocationGlobalRelative, gps_origin: Tuple[float, float]) -> Tuple[float, float, float]:
    lat_delta = haversine_distance(gps_origin[0], gps_origin[1], global_relative_frame.lat, gps_origin[1])
    lon_delta = haversine_distance(gps_origin[0], gps_origin[1], gps_origin[0], global_relative_frame.lon)

    x_sign = 1 if global_relative_frame.lat >= gps_origin[0] else -1
    y_sign = -1 if global_relative_frame.lon > gps_origin[1] else 1

    return x_sign * lat_delta, y_sign * lon_delta, global_relative_frame.alt


def convert_to_geo_coordinates(map_position: Tuple[float, float, float], gps_origin: Tuple[float, float]) -> LocationGlobalRelative:
    lat = gps_origin[0] + math.degrees(map_position[0] / EARTH_RADIUS)
    lon = gps_origin[1] + math.degrees(map_position[1] * (-1) / (EARTH_RADIUS * math.cos(math.radians(lat))))

    return LocationGlobalRelative(lat, lon, map_position[2])


def haversine_distance(lat1, lon1, lat2, lon2):
    # Convert latitude and longitude from degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Calculate the distance
    distance = EARTH_RADIUS * c

    return distance


if __name__ == '__main__':
    connection_string = f'127.0.0.1:{ARDUPILOT_PORT}'
    drone = connect(connection_string, wait_ready=True)

    camera_thread = Thread(target=camera_stuff, daemon=True)
    camera_thread.start()

    sys_status = drone._master.recv_match(type='SYS_STATUS', blocking=True)
    while 0 != sys_status.onboard_control_sensors_enabled & ~sys_status.onboard_control_sensors_health:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)
        sys_status = drone._master.recv_match(type='SYS_STATUS', blocking=True)

    target_alt = 5
    target_x = 100
    speed = 1.25

    drone.mode = VehicleMode("GUIDED")
    drone.armed = True
    drone.simple_takeoff(target_alt)
    while not drone.location.global_relative_frame.alt >= target_alt * 0.95:
        time.sleep(1)

    drone.simple_goto(convert_to_geo_coordinates((target_x, 0, target_alt), MAP_ORIGIN), groundspeed=speed)
    while not circle_found:
        time.sleep(1)

    drone.mode = VehicleMode("LAND")
    while not drone.location.global_relative_frame.alt <= .5:
        time.sleep(1)

    drone.armed = False
    drone.mode = VehicleMode("STABILIZE")
