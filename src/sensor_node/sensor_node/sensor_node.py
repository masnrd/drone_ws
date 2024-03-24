from pathlib import Path
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterUninitializedException
from sensor_interface_msgs.srv import ScanRequest
from .maplib import LatLon
from .hackrf_sensor import SimulatedSensorModule

SENSOR_MAPFILENAME = "empty_map.simmap"
SENSOR_MAPFILE = Path(get_package_share_directory("sensor_node")).joinpath("simmaps").joinpath(SENSOR_MAPFILENAME)
SENSOR_SCAN_TIMEFRAME = 2  # How long to scan for

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
        self.srv_sensor_scan = self.create_service(
            ScanRequest,
            f"/sensor_{drone_id}/srv/scan",
            self.sensor_handle_scan_req,
        )
        
        self.sensor_mod = SimulatedSensorModule(SENSOR_MAPFILE)  #TODO: Change for real one

    def sensor_handle_scan_req(self, request: ScanRequest.Request, response: ScanRequest.Response):
        latlon = LatLon(request.lat, request.lon)
        self.sensor_mod._update_position(latlon)
        detected = self.sensor_mod.scan(channel=8, time_frame=SENSOR_SCAN_TIMEFRAME)
        response.timestamp = self.clock_microseconds()
        response.lat = request.lat
        response.lon = request.lon
        response.detected = detected
        return response

    # Helper Methods
    def clock_microseconds(self) -> int:
        """ Returns the current clock in microseconds. """
        return self.get_clock().now().nanoseconds // 1000
    
def main(args=None):
    rclpy.init(args=args)
    sensor_node = SensorNode()

    rclpy.spin(sensor_node)

    sensor_node.destroy_node()

if __name__ == '__main__':
    main()