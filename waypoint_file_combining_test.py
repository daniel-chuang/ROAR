from ROAR.utilities_module.waypoint_tuning import combine_waypoints
import os

front = os.path.join("ROAR", "datasets")
total = combine_waypoints(os.path.join(front, "redmellon_waypoints.txt"), os.path.join(front, "waypoints.txt"))

new_file_path = os.path.join(front, "test.txt")
new_file = open(new_file_path, "w")
new_file.write(total)
