from ROAR.utilities_module.waypoint_tuning import *
import cv2

crash_logs_file = open("crash_logs_21.txt", "r")

map = prep_map_visualization(os.path.join("ROAR", "datasets", "birds_eye_map.npy"), os.path.join("ROAR", "datasets", "checkpoints.csv"))

for i in crash_logs_file.readlines():
    coords = i[1:-2].split(",")
    coords = [float(coord) for coord in coords]
    map = show_map(map, coords)

cv2.waitKey(0)