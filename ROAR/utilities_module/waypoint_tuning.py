# Imports
import os
import numpy as np
import cv2
import csv
import typing


### CHECKPOINTS ##
def prep_map(map_path, checkpoints_path):
    """Prepares a map to be used for other backend functions"""
    # Loading in the data
    data = np.load(map_path)
    data = data.astype(np.uint8)

    return data

def prep_checkpoints(checkpoints_path):
    checkpoints_file = open(checkpoints_path, newline="")
    checkpoints_reader = csv.reader(checkpoints_file)

    checkpoints = []
    for checkpoint in checkpoints_reader:
        y_coord = round(float(checkpoint[0]))
        x_coord = round(float(checkpoint[2]))
        checkpoints.append([x_coord, y_coord])

    return checkpoints 

def checkpoint(checkpoints, car_coords):
    """Checks if the car has reached a checkpoint"""
    car_coords = (int(car_coords[1]), int(car_coords[0]))
    for i, checkpoint in enumerate(checkpoints):
        i += 1
        if abs(car_coords[0] - checkpoint[0]) < 7 and abs(car_coords[1] - checkpoint[1]) < 7:
            print(f"CHECKPOINT {i} TOUCHED")

### MAPPING VISUALIZATION ###
def prep_map_visualization(map_path, checkpoints_path):
    """Prepares a map to be visualized"""
    # Loading in the data
    data = np.load(map_path)

    # Modifying the image to include checkpoints
    checkpoints_file = open(checkpoints_path, newline="")
    checkpoints_reader = csv.reader(checkpoints_file)
    data = data.astype(np.uint8)
    data = cv2.cvtColor(data, cv2.COLOR_GRAY2BGR)

    # Drawing the checkpoints
    for i, checkpoint in enumerate(checkpoints_reader):
        y_coord = round(float(checkpoint[0]))
        x_coord = round(float(checkpoint[2]))
        data[x_coord-30:x_coord+30, y_coord-30:y_coord+30] = (255, 0, 0)
        data = cv2.putText(data, str(i + 1), (max(0, (y_coord - 150)), (max(0, (x_coord - 150)))), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 255, 0), 10, cv2.LINE_AA)
    return data

def show_map(data, car_coords, speed = None, throttle = None):
    """
    Shows a birds eye view of the full map, with the car's location
    data : Numpy Array of map
    car_coords (OPTIONAL) : 2-item tuple (x,y) of car coordinates
    speed (OPTIONAL) : float of speed (normalized velocity)
    throttle (OPTIONAL) : float between 0-1, from vehicle.control.throttle
    """

    # # Normalize acceleration
    # x, y, z = [i[1] for i in acceleration]
    # acceleration = (x**2 + y**2 + z**2) ** 0.5
    # print(acceleration)

    # drawing car coords
    if car_coords != None:
        car_coords = (int(car_coords[1]), int(car_coords[0]))
        speed_width = 40
        throttle_width = 20
        if speed != None:
            data[car_coords[0]-speed_width:car_coords[0]+speed_width, car_coords[1]-speed_width:car_coords[1]+speed_width] = (0, speed, 255 - speed)
        if throttle != None:
            data[car_coords[0]-throttle_width:car_coords[0]+throttle_width, car_coords[1]-throttle_width:car_coords[1]+throttle_width] = (0, throttle * 255, 255 - throttle * 255)
        else:
            data[car_coords[0]-speed_width:car_coords[0]+speed_width, car_coords[1]-speed_width:car_coords[1]+speed_width] = (0, 200, 200)

    dim = (int(data.shape[0] * 0.10), int(data.shape[1] * 0.10))
    cv2.imshow("map",  cv2.resize(data, dim, interpolation = cv2.INTER_AREA))

    return data

def show_lane(data, car_coords, speed = None, throttle = None):
    """
    Shows a birds eye view of the map, at the specific location the car is at
    data : Numpy Array of map
    car_coords : 2-item tuple (x,y) of car coordinates
    speed (OPTIONAL) : float of speed (normalized velocity)
    throttle (OPTIONAL) : float between 0-1, from vehicle.control.throttle
    """
    # Changing type of car_coords
    if car_coords != None:
        car_coords = (int(car_coords[1]), int(car_coords[0]))
        speed_width = 10
        throttle_width = 5
        if speed != None:
            data[car_coords[0]-speed_width:car_coords[0]+speed_width, car_coords[1]-speed_width:car_coords[1]+speed_width] = (0, speed, 255 - speed)
        if throttle != None:
            data[car_coords[0]-throttle_width:car_coords[0]+throttle_width, car_coords[1]-throttle_width:car_coords[1]+throttle_width] = (0, throttle * 255, 255 - throttle * 255)
        else:
            data[car_coords[0]-speed_width:car_coords[0]+speed_width, car_coords[1]-speed_width:car_coords[1]+speed_width] = (0, 200, 200)

    # Cropping data
    width = 200
    height = 200
    data = data[car_coords[0]-height:car_coords[0]+height, car_coords[1]-width:car_coords[1]+width]

    # drawing car coords
    # if car_coords != None:
    #     car_coords = (int(car_coords[0]), int(car_coords[1]))
    #     data[car_coords[0]-40:car_coords[0]+40, car_coords[1]-40:car_coords[1]+40] = (0, 255, 0)

    if data.size != 0:
        cv2.imshow("map",  data)

    return data