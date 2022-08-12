from ROAR.agent_module.agent import Agent
from ROAR.utilities_module.data_structures_models import SensorsData
from ROAR.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR.configurations.configuration import Configuration as AgentConfig
from pathlib import Path

# showing map
from ROAR.utilities_module.waypoint_tuning import show_map, prep_map_visualization
import os

class WaypointGeneratingAgent(Agent):
    def __init__(self, vehicle: Vehicle, agent_settings: AgentConfig, **kwargs):
        super().__init__(vehicle=vehicle, agent_settings=agent_settings, **kwargs)
        self.output_file_path: Path = self.output_folder_path / "waypoints.txt"
        if self.output_folder_path.exists() is False:
            self.output_folder_path.mkdir(exist_ok=True, parents=True)
        self.output_file = self.output_file_path.open('w')
        self.map = prep_map_visualization(os.path.join("data", "birds_eye_map.npy"), os.path.join("data", "checkpoints.csv"))


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
        self.map = show_map(self.map, self.car_coords, self.speed, self.throttle)
        #show_lane(self.lane_map, self.car_coords, self.speed, self.throttle)
        self.transform_history.append(self.vehicle.transform)

        if self.time_counter > 1:
            print(f"Writing to [{self.output_file_path}]: {self.vehicle.transform}")
            self.output_file.write(self.vehicle.transform.record() + "\n")
        return VehicleControl()
