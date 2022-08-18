from ROAR.agent_module.agent import Agent
from ROAR.utilities_module.data_structures_models import SensorsData
from ROAR.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR.configurations.configuration import Configuration as AgentConfig
from pathlib import Path

# showing map
from ROAR.utilities_module.waypoint_tuning import *
import os

class WaypointGeneratingAgent(Agent):
    def __init__(self, vehicle: Vehicle, agent_settings: AgentConfig, **kwargs):
        super().__init__(vehicle=vehicle, agent_settings=agent_settings, **kwargs)
        # User input for start and end
        self.start_checkpoint = agent_settings.spawn_point_id
        self.end_checkpoint = int(input("What checkpoint are you ending at? "))

        # Output path
        self.output_file_path: Path = self.output_folder_path / (str(self.end_checkpoint - 1) + ".txt")
        if self.output_folder_path.exists() is False:
            self.output_folder_path.mkdir(exist_ok=True, parents=True)
        self.output_file = self.output_file_path.open('w')

        # For waypoint tuning
        self.checkpoints = prep_checkpoints(os.path.join("ROAR", "datasets", "checkpoints.csv"))

        # For map visualizations
        self.map = prep_map_visualization(os.path.join("ROAR", "datasets", "birds_eye_map.npy"), os.path.join("ROAR", "datasets", "checkpoints.csv"))
        self.lane_map = prep_map_visualization(os.path.join("ROAR", "datasets", "birds_eye_map.npy"), os.path.join("ROAR", "datasets", "checkpoints.csv"))
        
        # For returning current time
        self.return_time = False
        self.times_list = []
        self.time_var = -5
        self.most_recent_checkpoint = -1
        self.start_time = 0
        self.start_recording = False

    def call_in_carla_runner_to_return_time(self):
        return self.return_time

    def get_diff_in_times(self):
        times_list_diffs = []
        for i in range(len(self.times_list) - 1):
            times_list_diffs.append(self.times_list[i + 1] - self.times_list[i])
        return(times_list_diffs)


    def run_step(self, sensors_data: SensorsData,
                 vehicle: Vehicle) -> VehicleControl:
        super(WaypointGeneratingAgent, self).run_step(sensors_data=sensors_data,
                                                     vehicle=vehicle)
        
        # Showing minimap
        # Showing live minimap
        car_coords = [float(i) for i in self.vehicle.transform.record().split(",")]
        self.speed = self.vehicle.get_speed(self.vehicle)
        self.throttle = self.vehicle.control.throttle
        self.car_coords = [float(i) for i in self.vehicle.transform.record().split(",")][0:3:2]
        # self.map = show_map(self.map, self.car_coords, self.speed, self.throttle)
        show_lane(self.lane_map, self.car_coords, self.speed, self.throttle)
        self.transform_history.append(self.vehicle.transform)

        # Checking for checkpoint
        # Maining time list
        self.return_time = False
        cur_checkpoint = checkpoint(self.checkpoints, self.car_coords)

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
            print(f"AT CHECKPOINT {cur_checkpoint}")
            self.return_time = True
            self.most_recent_checkpoint = cur_checkpoint
        
        # Quits the script after the final checkpoint is LEFT
        if cur_checkpoint is None and self.most_recent_checkpoint == self.end_checkpoint:
            print("FINAL CHECKPOINT REACHED, TERMINATING SCRIPT")
            exit()

        # Check to start recording
        if cur_checkpoint is None and self.most_recent_checkpoint == self.start_checkpoint:
            self.start_recording = True

        # Logging
        if self.time_counter > 1 and self.start_recording == True:
            #print(f"Writing to [{self.output_file_path}]: {self.vehicle.transform}")
            self.output_file.write(self.vehicle.transform.record() + "\n")
        return VehicleControl()
