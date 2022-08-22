from ROAR.agent_module.agent import Agent
from pathlib import Path
from ROAR.control_module.pid_fast_controller import PIDFastController
from ROAR.planning_module.local_planner.simple_waypoint_following_local_planner_fast import \
    SimpleWaypointFollowingLocalPlanner
from ROAR.planning_module.behavior_planner.behavior_planner import BehaviorPlanner
from ROAR.planning_module.mission_planner.waypoint_following_mission_planner import WaypointFollowingMissionPlanner
from ROAR.utilities_module.data_structures_models import SensorsData
from ROAR.utilities_module.vehicle_models import VehicleControl, Vehicle
import logging

# showing map
from ROAR.utilities_module.waypoint_tuning import *
import os

# SWITCH OFF FOR SUBMISSIONS
competitive_mode = True

class PIDFastAgent(Agent):
    def __init__(self, target_speed=40, **kwargs):
        super().__init__(**kwargs)
        self.target_speed = target_speed
        self.logger = logging.getLogger("PID Agent")

        # Combining all waypoint files into one
        self.waypoints_folder = os.path.join(os.path.normpath(self.agent_settings.waypoint_file_path + os.sep + os.pardir))
        self.all_waypoints_file = os.path.join(self.waypoints_folder, "all_waypoints.txt")
        self.waypoints_file_paths_list = [os.path.join(self.waypoints_folder, file) for file in os.listdir(self.waypoints_folder) if "all_waypoints.txt" not in file]
        combine_waypoints(self.all_waypoints_file, *self.waypoints_file_paths_list)

        self.route_file_path = Path(self.all_waypoints_file)
        self.pid_controller = PIDFastController(agent=self, steering_boundary=(-1, 1), throttle_boundary=(0, 1))
        self.mission_planner = WaypointFollowingMissionPlanner(agent=self)
        # initiated right after mission plan
        self.behavior_planner = BehaviorPlanner(agent=self)
        self.local_planner = SimpleWaypointFollowingLocalPlanner(
            agent=self,
            spawn_point_id=self.agent_settings.spawn_point_id,
            controller=self.pid_controller,
            mission_planner=self.mission_planner,
            behavior_planner=self.behavior_planner,
            closeness_threshold=1,) # original 1
        self.logger.debug(
            f"Waypoint Following Agent Initiated. Reading f"
            f"rom {self.route_file_path.as_posix()}")

        # For waypoint tuning
        self.checkpoints = prep_checkpoints(os.path.join("ROAR", "datasets", "checkpoints.csv"))
        self.most_recent_checkpoint = -1

        if not competitive_mode:

            # For map visualizations
            self.map = prep_map_visualization(os.path.join("ROAR", "datasets", "birds_eye_map.npy"), os.path.join("ROAR", "datasets", "checkpoints.csv"))
            self.lane_map = prep_map_visualization(os.path.join("ROAR", "datasets", "birds_eye_map.npy"), os.path.join("ROAR", "datasets", "checkpoints.csv"))

            # For returning current time
            self.return_time = False
            self.times_list = []
            self.time_var = -5
            self.start_time = 0

    def call_in_carla_runner_to_return_time(self):
        if not competitive_mode:
            return self.return_time

    def get_diff_in_times(self):
        if not competitive_mode:
            times_list_diffs = []
            for i in range(len(self.times_list) - 1):
                times_list_diffs.append(self.times_list[i + 1] - self.times_list[i])
            return(times_list_diffs)

    def run_step(self, vehicle: Vehicle,
                 sensors_data: SensorsData) -> VehicleControl:
        super(PIDFastAgent, self).run_step(vehicle=vehicle,
                                       sensors_data=sensors_data)
        

        self.car_coords = [float(i) for i in self.vehicle.transform.record().split(",")][0:3:2]

        if not competitive_mode:
            # Showing live minimap
            self.speed = self.vehicle.get_speed(self.vehicle)
            self.throttle = self.vehicle.control.throttle
            self.map = show_map(self.map, self.car_coords, self.speed, self.throttle)
            #show_lane(self.lane_map, self.car_coords, self.speed, self.throttle)

        # Checking for checkpoint
        # Maining time list
        cur_checkpoint = checkpoint(self.checkpoints, self.car_coords)
        if not competitive_mode:
            self.return_time = False

            if len(self.times_list) == 0.0:
                self.start_time = self.time_var

            if self.time_var - self.start_time not in self.times_list and cur_checkpoint == self.most_recent_checkpoint:
                self.times_list.append(self.time_var - self.start_time)
                self.times_list_diffs = self.get_diff_in_times()

                print("Times", self.times_list)
                print("Time Diffs", self.times_list_diffs)
                print("Total Time", sum(self.times_list_diffs))
                print("\n")

        if cur_checkpoint is not None and cur_checkpoint != self.most_recent_checkpoint:
            if not competitive_mode:
                print(f"AT CHECKPOINT {cur_checkpoint}")
            self.return_time = True
            self.most_recent_checkpoint = cur_checkpoint

        # checking if first turn passed
        if self.most_recent_checkpoint in [12, 12.25]:
            # during turn
            if at_point((1332.74462890625, 4245.38818359375), car_coords=self.car_coords) or at_point((1329.954345703125, 4236.50537109375), car_coords=self.car_coords):
                self.most_recent_checkpoint = 12.5
                if not competitive_mode:
                    print("REACHED")
            # pre turn
            if at_point((1687.881103515625, 4203.75439453125), car_coords=self.car_coords):
                self.most_recent_checkpoint = 12.25
                if not competitive_mode:
                    print("REACHED")
        
        # checking for six ramp
        if self.most_recent_checkpoint == 6:
            if at_point((3679.475341796875, 2432.698486328125), car_coords=self.car_coords, margin = 3):
                self.most_recent_checkpoint = 6.5
                if not competitive_mode:
                    print("REACHED BEGINNING")

        # checking for six ramp end
        if self.most_recent_checkpoint == 6.5:
            if at_point((3688.43994140625, 2545.862060546875), car_coords=self.car_coords, margin = 3):
                self.most_recent_checkpoint = 6
                if not competitive_mode:
                    print("REACHED END")

        # Other
        self.transform_history.append(self.vehicle.transform)        
        # print(self.vehicle.transform, self.vehicle.velocity)
        if self.is_done:
            control = VehicleControl()
            self.logger.debug("Path Following Agent is Done. Idling.")
        else:
            control = self.local_planner.run_in_series(self.most_recent_checkpoint)
        return control
