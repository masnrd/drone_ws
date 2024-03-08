from datetime import datetime
from typing import Any
from math import isnan
from pathlib import Path
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.exceptions import ParameterUninitializedException
from px4_msgs.msg import VehicleGlobalPosition
from mc_interface_msgs.msg import Detected
from .maplib import LatLon
from .hackrf_sensor import SimulatedSensorModule
from .detection_utils import DetectedEntity

SENSOR_MAPFILENAME = "25-20240308-180819.simmap"
SENSOR_MAPFILE = Path(get_package_share_directory("sensor_node")).joinpath("simmaps").joinpath(SENSOR_MAPFILENAME)

class SensorNode(Node):
    def __init__(self):
        super().__init__("sensor_node")

        # Obtain drone ID
        ## For ROS2 Iron
        try:
            self.declare_parameter("droneId", rclpy.Parameter.Type.INTEGER)
            drone_id = self.get_parameter("droneId").get_parameter_value().integer_value
        except ParameterUninitializedException:
            drone_id = None

        # ## For ROS2 Foxy
        # self.declare_parameter("droneId", -1)
        # drone_id = self.get_parameter("droneId").get_parameter_value().integer_value
        # if drone_id == -1:
        #     drone_id = None
        
        self.drone_id = drone_id
        self._drone_ns = f"/px4_{drone_id}" if drone_id is not None else ""
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub_vehglobpos = self.create_subscription(
            VehicleGlobalPosition,
            f"{self._drone_ns}/fmu/out/vehicle_global_position",
            self.fc_recv_vehglobpos,
            self.qos_profile,
        )
        self.pub_mc_detected = self.create_publisher(
            Detected,
            f"/mc_{drone_id}/mc/out/detected",
            self.qos_profile
        )
        
        self.sensor_mod = SimulatedSensorModule(SENSOR_MAPFILE)  #TODO: Change for real one
            
    def start_scan(self):
        while True:
            detected = self.sensor_mod.scan()
            if detected:
                entity = DetectedEntity(self.drone_id, self.cur_latlon, datetime.now())
                self.mc_publish_detected(entity)

    def fc_recv_vehglobpos(self, msg: VehicleGlobalPosition):
        """ Handle an incoming GlobalPosition message from FC. """
        if isnan(msg.lat) or isnan(msg.lon):
            return
        self.cur_latlon = LatLon(msg.lat, msg.lon)
        self.sensor_mod._update_position(self.cur_latlon)

    def mc_publish_detected(self, entity: DetectedEntity):
        msg = entity.to_message()
        self.pub_mc_detected.publish(msg)
        print(f"Drone {self.drone_id}: Reporting found entity at {entity.coords}")

def main(args=None):
    rclpy.init(args=args)
    sensor_node = SensorNode()

    rclpy.spin(sensor_node)

    sensor_node.destroy_node()

if __name__ == '__main__':
    main()