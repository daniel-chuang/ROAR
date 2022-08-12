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

class PIDFastAgent(Agent):
    def __init__(self, target_speed=40, **kwargs):
        super().__init__(**kwargs)
        self.target_speed = target_speed
        self.logger = logging.getLogger("PID Agent")
        self.route_file_path = Path(self.agent_settings.waypoint_file_path)
        self.pid_controller = PIDFastController(agent=self, steering_boundary=(-1, 1), throttle_boundary=(0, 1))
        self.mission_planner = WaypointFollowingMissionPlanner(agent=self)
        # initiated right after mission plan

        self.behavior_planner = BehaviorPlanner(agent=self)
        self.local_planner = SimpleWaypointFollowingLocalPlanner(
            agent=self,
            controller=self.pid_controller,
            mission_planner=self.mission_planner,
            behavior_planner=self.behavior_planner,
            closeness_threshold=1) # original 1
        self.logger.debug(
            f"Waypoint Following Agent Initiated. Reading f"
            f"rom {self.route_file_path.as_posix()}")
        
        # For waypoint tuning
        self.checkpoints = prep_checkpoints(os.path.join("data", "checkpoints.csv"))

        # For map visualizations
        self.map = prep_map_visualization(os.path.join("data", "birds_eye_map.npy"), os.path.join("data", "checkpoints.csv"))
        self.lane_map = prep_map_visualization(os.path.join("data", "birds_eye_map.npy"), os.path.join("data", "checkpoints.csv"))

    def run_step(self, vehicle: Vehicle,
                 sensors_data: SensorsData) -> VehicleControl:
        super(PIDFastAgent, self).run_step(vehicle=vehicle,
                                       sensors_data=sensors_data)
        
        # Showing live minimap
        self.speed = self.vehicle.get_speed(self.vehicle)
        self.throttle = self.vehicle.control.throttle
        self.car_coords = [float(i) for i in self.vehicle.transform.record().split(",")][0:3:2]
        #self.map = show_map(self.map, self.car_coords, self.speed, self.throttle)
        show_lane(self.lane_map, self.car_coords, self.speed, self.throttle)
        self.transform_history.append(self.vehicle.transform)
        
        # print(self.vehicle.transform, self.vehicle.velocity)
        
        if self.is_done:
            control = VehicleControl()
            self.logger.debug("Path Following Agent is Done. Idling.")
        else:
            control = self.local_planner.run_in_series()
        return control
