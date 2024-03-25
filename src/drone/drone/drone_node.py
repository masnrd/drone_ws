from typing import Dict, Any, Union, List, Deque
from enum import IntEnum
from collections import deque
import struct
from math import isnan
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.task import Future
from .maplib import LatLon
from .pathfinder import PathfinderState
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleGlobalPosition, VehicleLocalPosition, VehicleCommandAck, TrajectorySetpoint, VehicleControlMode, VehicleStatus, BatteryStatus
from mc_interface_msgs.msg import Ready, Detected
from mc_interface_msgs.srv import Command, Status
from sensor_interface_msgs.srv import ScanRequest

""" CONSTANTS """
# Intervals
CYCLE_INTERVAL = 0.1
HB_FC_INTERVAL = 0.1
HB_MC_INTERVAL = 0.1
SENSOR_CHECK_INTERVAL = 0.5 # How often to check the sensor's status

## If the drone remains in IDLE_AIR mode for this number of cycles (i.e. connected, in the air), it lands.
MAX_IDLE_CYCLES_IN_AIR = 5 * 60 * 10 # 5 minutes

## If the drone remains in CONN_AIR mode for this number of cycles (i.e. cannot connect to MC and is in the air), it gives up and lands.
MAX_CONN_MC_IN_AIR = 15 * 10  # 15 seconds

# These control how many failed heartbeats is necessary to determine a loss of connection
FC_CONNECTED_TIMEOUT = 1
MC_CONNECTED_TIMEOUT = 3
CONN_FC_TIMEOUT = 60  # After this timeout, drone has failed to connect to flight controller

# Default constants for the drone
DEFAULT_DRONE_ID = 69
DEFAULT_DIST_FROM_GROUND = 7.0
DEFAULT_RCH_THRESH = 1  # Distance in metres from a point to consider it 'reached'.
DEFAULT_LANDED_THRESH = 0.5  # Distance in metres for dfg, for drone to be "landed".


class DroneState(IntEnum):
    """ Current State of the drone """
    INIT     = 0
    CONN_GRD = 1   # Drone is connecting to FC/MC, and is on the ground
    IDLE_GRD = 2   # Drone is connected, and on the ground.
    TAKEOFF  = 3   # Drone is taking off
    LANDING  = 4
    CONN_AIR = 5   # Drone is connecting to MC, and in the air. (This is so the drone doesn't just try to land upon disconnects)
    IDLE_AIR = 6   # Drone is connected, and in the air.
    TRAVEL   = 7   # Drone is moving to a position
    SEARCH   = 8   # Drone is searching a sector
    EXIT     = 9    # Drone is landing and ending its state.

"""
INSTRUCTION
An instruction is a low-level instruction for the drone.
"""

class InstructionId(IntEnum):
    DISCONNECT = 0
    LAND       = 1
    MOVETO     = 2
    SEARCH     = 3
    STOP       = 4

class Instruction:
    def __init__(self, id: InstructionId, dest_latlon: LatLon = None):
        self.instr_id = id
        self.dest_latlon = dest_latlon
        if dest_latlon is None:
            if id in [InstructionId.MOVETO, InstructionId.SEARCH]:
                raise ValueError("Expected a latlon for MOVETO or LAND.")

"""
COMMAND
A command is a high-level command coming from Mission Control, and will be parsed into lower-level Instructions
for the drone.
"""

class CommandId(IntEnum):
    RTB           = 0  # RTB(): Force RTB. No further commands will be accepted.
    SEARCH_SECTOR = 1  # SEARCH_SECTOR(start, prob_map): Searches a specific sector
    MOVE_TO       = 2  # MOVE_TO(pos): Go to a specific lat/lon.
    LAND          = 3  # LAND(): Land the drone
    DISCONNECT    = 4  # DISCONNECT(): Disconnect the drone from flight controller and exit.

def serialise_path(pos_ls: List[LatLon]) -> bytes:
    """ Serialises a list of LatLon points into a bytearray. """
    ret = b""
    for pos in pos_ls:
        if pos is not None:
            ret += struct.pack("!ff", pos.lat, pos.lon)
    return ret


"""
Drone ROS2 Node
"""

class DroneNode(Node):
    def __init__(self):
        super().__init__("drone")

        # Obtain drone ID
        self.declare_parameter("droneId", -1)
        drone_id = self.get_parameter("droneId").get_parameter_value().integer_value
        if drone_id == -1:
            drone_id = None

        # Initialise drone node
        self.drone_state = DroneState.INIT
        self.drone_id = drone_id if drone_id is not None else DEFAULT_DRONE_ID
        self.pathfinder = None
        self.cur_latlon: Union[LatLon, None] = None
        self.tgt_latlon: Union[LatLon, None] = None
        self.ref_latlon: Union[LatLon, None] = None
        self.home_latlon: Union[LatLon, None] = None
        self.cur_alt, self.tgt_alt = 0.0, 0.0
        self.tgt_dfg = DEFAULT_DIST_FROM_GROUND  # Target distance from ground
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Initialise publishers, subscribers for this drone
        self._drone_ns = f"/px4_{drone_id}" if drone_id is not None else ""
        ## Flight Controller
        self.fc_sys_id = None
        self.fc_com_id = None
        self.sub_vehstatus= self.create_subscription(
            VehicleStatus,
            f"{self._drone_ns}/fmu/out/vehicle_status",
            self.fc_recv_vehstatus,
            self.qos_profile,
        )
        self.sub_vehglobpos = self.create_subscription(
            VehicleGlobalPosition,
            f"{self._drone_ns}/fmu/out/vehicle_global_position",
            self.fc_recv_vehglobpos,
            self.qos_profile,
        )
        self.sub_vehlocpos = self.create_subscription(
            VehicleLocalPosition,
            f"{self._drone_ns}/fmu/out/vehicle_local_position",
            self.fc_recv_vehlocpos,
            self.qos_profile,
        )
        self.sub_batstatus = self.create_subscription(
            BatteryStatus,
            f"{self._drone_ns}/fmu/out/battery_status",
            self.fc_recv_batstatus,
            self.qos_profile,
        )
        self.pub_vehcom = self.create_publisher(
            VehicleCommand,
            f"{self._drone_ns}/fmu/in/vehicle_command",
            self.qos_profile,
        )
        self.pub_ocm = self.create_publisher(
            OffboardControlMode,
            f"{self._drone_ns}/fmu/in/offboard_control_mode",
            self.qos_profile,
        )
        self.pub_trajsp = self.create_publisher(
            TrajectorySetpoint,
            f"{self._drone_ns}/fmu/in/trajectory_setpoint",
            self.qos_profile,
        )
        ## Mission Control
        self.pub_mc_ready = self.create_publisher(
            Ready,
            f"/mc_{drone_id}/mc/out/ready",
            self.qos_profile,
        )
        self.pub_mc_detected = self.create_publisher(
            Detected,
            f"/mc_{drone_id}/mc/out/detected",
            self.qos_profile,
        )
        self.cli_mc_status = self.create_client(
            Status,
            f"/mc_{drone_id}/mc/srv/status"
        )
        self.srv_mc_cmd = self.create_service(
            Command,
            f"/mc_{drone_id}/mc/srv/cmd",
            self.mc_handle_command,
        )
        ## Sensor
        self.cli_sensor_scan = self.create_client(
            ScanRequest,
            f"/sensor_{drone_id}/srv/scan"
        )

        # Cycle Information
        self.cycles = 0
        self.cur_instr: Union[Instruction, None] = None
        self.instr_queue: Deque[Instruction] = deque()
        self.cycle_timer = self.create_timer(CYCLE_INTERVAL, self.drone_run)
        self.cycle_error_str = ""
        self.idle_cycles = 0
        self.idle_ping_count = 0
        self._conn_air_cycles = 0

        ## FC
        self.fc_cycles = 0
        self.fc_hb_timer = self.create_timer(HB_FC_INTERVAL, self.fc_run)
        self.battery_percentage = -1.0
        self.battery_secondsleft = 0.0
        self.fc_armed = False
        self.fc_connected = False
        self.fc_nav_mode = -1  # 14 if offboard
        self.fc_connect_attempts = 0
        self._fc_start_arming = False

        ## MC
        self.mc_connected = False
        self.mc_cycles = 0
        self.mc_hb_timer = self.create_timer(HB_MC_INTERVAL, self.mc_run)
        self.mc_dropped_count = 0
        self.mc_last_rtt = 0.0
        self.mc_last_cmd_id = -1
        self._mc_prev_fut: Union[Future, None] = None

        ## Sensor
        self.sensor_scan_timer = self.create_timer(SENSOR_CHECK_INTERVAL, self.drone_request_scan)
        self._sensor_prev_fut: Union[Future, None] = None

    # State Management
    def change_state(self, new_state: DroneState, msg: str = ""):
        if self.drone_state == DroneState.EXIT:
            self.log(f"Ignore state change to {DroneState(new_state).name}, drone is in EXIT state.")
            return
        
        if len(msg) == 0:
            self.log(f"State change from {self.drone_state.name} to {new_state.name}.")
        else:
            self.log(f"State change from {self.drone_state.name} to {new_state.name}: {msg}")
        
        # Initialise values for certain states
        if new_state == DroneState.CONN_GRD:
            self.fc_connect_attempts = 0
        elif new_state == DroneState.IDLE_AIR:
            self.idle_cycles = 0
        elif new_state == DroneState.CONN_AIR:
            self._conn_air_cycles = 0

        self.drone_state = new_state

    def drone_run(self):
        """ Main loop for the Drone """
        # Maintain variables
        if self.ref_latlon is None or self.cur_latlon is None or self.fc_sys_id is None or self.fc_com_id is None:
            self.fc_connected = False
        else:
            self.fc_connected = True

        # Run state functions
        if self.drone_state == DroneState.INIT:
            self.drone_run_init()
        elif self.drone_state == DroneState.CONN_GRD:
            self.drone_run_conn_grd()
        elif self.drone_state == DroneState.IDLE_GRD:
            self.drone_run_idle_grd()
        elif self.drone_state == DroneState.TAKEOFF:
            self.drone_run_takeoff()
        elif self.drone_state == DroneState.LANDING:
            self.drone_run_landing()
        elif self.drone_state == DroneState.CONN_AIR:
            self.drone_run_conn_air()
        elif self.drone_state == DroneState.IDLE_AIR:
            self.drone_run_idle_air()
        elif self.drone_state == DroneState.TRAVEL:
            self.drone_run_travel()
        elif self.drone_state == DroneState.SEARCH:
            self.drone_run_search()
        elif self.drone_state == DroneState.EXIT:
            self.drone_run_exit()
        else:
            self.error(f"Invalid drone state {self.drone_state}")
        
        self.cycles += 1

    def fc_run(self):
        """ FC Heartbeat """
        if self.drone_state in [DroneState.TAKEOFF, DroneState.IDLE_AIR, DroneState.CONN_AIR, DroneState.TRAVEL, DroneState.SEARCH, DroneState.LANDING]:
            # Drone should be FLYING.
            self.fc_cycles += 1
            self.fc_publish_heartbeat()
            if self.drone_state != DroneState.LANDING:
                self.fc_publish_trajectorysetpoint()
        
        if self._fc_start_arming:
            self.fc_publish_vehiclecommand(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.fc_publish_vehiclecommand(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def mc_run(self):
        """ MC Heartbeat """

        # Determine if the previous request was dropped
        if self._mc_prev_fut is None or not self._mc_prev_fut.done():
            # Either we haven't sent a request, or we're still waiting (i.e. assume MC dropped message)
            if self._mc_prev_fut is not None:
                self._mc_prev_fut.cancel()

            self.mc_dropped_count += 1
        else:
            # Previous request was received, not dropped
            self.mc_dropped_count = 0
            self.mc_connected = True
        
        if self.mc_dropped_count >= MC_CONNECTED_TIMEOUT: # LOST connection to MC
            self.mc_connected = False
            self.mc_dropped_count = 0
            return
        
        self._mc_prev_fut = self.mc_request_status()
        self._mc_prev_fut.add_done_callback(self.mc_response_status)

    # States
    def drone_run_init(self):
        """ (Re-)Initialise drone """
        self.cur_latlon, self.ref_latlon, self.tgt_latlon = None, None, None
        self.cur_alt, self.tgt_alt = 0, 0
        self.fc_sys_id, self.fc_com_id = None, None
        self.change_state(DroneState.CONN_GRD)
    
    def drone_run_conn_grd(self):
        """ Only progress once the drone is connected to both MC and FC. """
        if self.fc_connected and self.mc_connected:
            self.home_latlon = self.cur_latlon
            self.change_state(DroneState.IDLE_GRD, f"Ref LatLon/Cur LatLon:{self.ref_latlon}/{self.cur_latlon}. System ID/Component Id: {self.fc_sys_id}/{self.fc_com_id}")
            return
        
        if not self.fc_connected:
            self.fc_connect_attempts += 1
            if self.fc_connect_attempts >= (CONN_FC_TIMEOUT // CYCLE_INTERVAL):
                self.error(f"Could not obtain reference point or system ID after {CONN_FC_TIMEOUT} seconds.")

    def drone_run_idle_grd(self):
        if not (self.fc_connected and self.mc_connected):
            self.change_state(DroneState.CONN_GRD, "Lost connection to MC or FC.")
            return
        
        if len(self.instr_queue) == 0:
            return
        
        self.tgt_latlon = self.cur_latlon
        
        # Peek at instr queue
        instr = self.instr_queue[0]
        if instr.instr_id == InstructionId.DISCONNECT:
            self.change_state(DroneState.EXIT, "Instructed to disconnect.")
        elif instr.instr_id == InstructionId.LAND or instr.instr_id == InstructionId.STOP:
            # We're already landed and idle.
            self.instr_queue.popleft()
        elif instr.instr_id == InstructionId.MOVETO or instr.instr_id == InstructionId.SEARCH:
            # Take off, but don't pop the queue to keep the instruction there for IDLE_AIR.
            self.change_state(DroneState.TAKEOFF, "Received instruction to move, taking off.")

    def drone_run_takeoff(self):
        if not self.fc_armed:
            self._fc_start_arming = True
            return
        self._fc_start_arming = False
        self.tgt_latlon = self.cur_latlon
        self.tgt_alt = DEFAULT_DIST_FROM_GROUND
        
        if self.cur_alt > self.tgt_alt - 1:
            self.change_state(DroneState.IDLE_AIR, "Drone is armed and in the air.")
    
    def drone_run_landing(self):
        self.fc_publish_land()
        if (self.cur_alt <= DEFAULT_LANDED_THRESH) or (not self.fc_armed) or (not self.fc_connected):
            self.change_state(DroneState.IDLE_GRD, "Drone has landed.")

    def drone_run_conn_air(self):
        if not (self.fc_connected and self.fc_armed):
            self.change_state(DroneState.CONN_GRD, "Lost connection to flight controller, assume FC is landing automatically.")
            return
        if self.mc_connected:
            self.change_state(DroneState.IDLE_AIR, "Reconnected to MC.")
            return
        
        self._conn_air_cycles += 1
        if self._conn_air_cycles >= MAX_CONN_MC_IN_AIR:
            self.change_state(DroneState.CONN_GRD, f"Landing, after failure to connect to MC after {self._conn_air_cycles * CYCLE_INTERVAL}")

    def drone_run_idle_air(self):
        if not (self.fc_connected and self.fc_armed):
            self.change_state(DroneState.CONN_GRD, "Lost connection to flight controller, assume FC is landing automatically.")
            return
        if not self.mc_connected:
            self.change_state(DroneState.CONN_AIR, "Lost connection to mission control.")
            return
        
        if len(self.instr_queue) == 0:
            self.idle_cycles += 1
            if self.idle_cycles >= MAX_IDLE_CYCLES_IN_AIR:
                self.change_state(DroneState.LANDING, f"Landing due to inactivity at {self.cur_latlon}")
            return
        
        # Pop from instr queue
        instr = self.instr_queue.popleft()
        if instr.instr_id == InstructionId.MOVETO:
            self.tgt_latlon = instr.dest_latlon
            self.change_state(DroneState.TRAVEL, f"Moving to {instr.dest_latlon}.")
        elif instr.instr_id == InstructionId.SEARCH:
            self.tgt_latlon = instr.dest_latlon
            self.change_state(DroneState.SEARCH, f"Searching {instr.dest_latlon}")
        elif instr.instr_id == InstructionId.LAND:
            self.change_state(DroneState.LANDING, f"Landing at {self.cur_latlon}.")
        elif instr.instr_id == InstructionId.DISCONNECT:
            self.instr_queue.appendleft(instr)
            self.change_state(DroneState.LANDING, f"Disconnecting. Landing first at {self.cur_latlon}.")
        elif instr.instr_id == InstructionId.STOP:
            return
    
    def drone_run_travel(self):
        if not (self.mc_connected and self.fc_connected and self.fc_armed):
            self.instr_queue.appendleft(Instruction(InstructionId.MOVETO, self.tgt_latlon))
            self.change_state(DroneState.CONN_AIR, "Lost connection to mission control.")
            return
        
        if len(self.instr_queue) > 0:
            # Peek at instruction queue
            instr = self.instr_queue[0]
            if instr.instr_id == InstructionId.STOP:
                self.instr_queue.popleft()
                self.tgt_latlon = self.cur_latlon
                self.change_state(DroneState.IDLE_AIR)
            elif instr.instr_id == InstructionId.DISCONNECT:
                self.tgt_latlon = self.cur_latlon
                self.change_state(DroneState.IDLE_AIR)

        # Check if we've reached
        reached = (self.tgt_latlon is None) or (self.tgt_latlon.distFromPoint(self.cur_latlon) <= DEFAULT_RCH_THRESH)
        if reached:
            # Change state to the next state. Any necessary arguments should have been processed when the command was first received.
            self.change_state(DroneState.IDLE_AIR, f"MOVETO {self.tgt_latlon} done.")
            self.tgt_latlon = self.cur_latlon

    def drone_run_search(self):
        if not (self.mc_connected and self.fc_connected and self.fc_armed):
            self.instr_queue.appendleft(Instruction(InstructionId.MOVETO, self.tgt_latlon))
            self.change_state(DroneState.CONN_AIR, "Lost connection to mission control.")
            return
        
        if len(self.instr_queue) > 0:
            # Peek at instruction queue
            instr = self.instr_queue[0]
            if instr.instr_id == InstructionId.STOP:
                self.instr_queue.popleft()
                self.tgt_latlon = self.cur_latlon
                self.change_state(DroneState.IDLE_AIR)
            elif instr.instr_id == InstructionId.DISCONNECT:
                self.tgt_latlon = self.cur_latlon
                self.change_state(DroneState.IDLE_AIR)
        
        # Check if we've reached
        reached = (self.tgt_latlon is None) or (self.tgt_latlon.distFromPoint(self.cur_latlon) <= DEFAULT_RCH_THRESH)
        if reached:
            # Get next waypoint
            if self.pathfinder is None:
                self.error("In search mode, but pathfinder is not defined.")
                return
            next_latlon = self.pathfinder.get_next_waypoint(self.tgt_latlon)
            if next_latlon is None:
                self.idle_cycles = 0
                self.change_state(DroneState.IDLE_AIR, f"Completed search.")
                self.tgt_latlon = self.cur_latlon
            else:
                self.tgt_latlon = next_latlon

    def drone_run_exit(self):
        self.log("Exiting offboard mode.")
        exit(0)

    def drone_request_scan(self):
        # Get result of previous scan
        scan_done = False

        if self._sensor_prev_fut is None:
            scan_done = True
        else:
            if self._sensor_prev_fut.done():
                # Parse the result
                detection_msg: ScanRequest.Response = self._sensor_prev_fut.result()
                if detection_msg.detected:
                    det_latlon = LatLon(detection_msg.lat, detection_msg.lon)
                    det_ts = detection_msg.timestamp
                    self.mc_publish_detected(det_latlon, det_ts)
                scan_done = True
            else:
                scan_done = False

        if scan_done:
            self._sensor_prev_fut = self.sensor_request_scan()

    # ROS2 functions
    ## Subscriptions
    def fc_recv_vehstatus(self, msg: VehicleStatus):
        self.fc_sys_id = msg.system_id
        self.fc_com_id = msg.component_id
        self.fc_armed = True if msg.arming_state == 2 else False
        self.fc_nav_mode = msg.nav_state

    def fc_recv_vehglobpos(self, msg: VehicleGlobalPosition):
        """ Handle an incoming GlobalPosition message from FC. """
        if isnan(msg.lat) or isnan(msg.lon):
            return
        self.cur_latlon = LatLon(msg.lat, msg.lon)
                
    def fc_recv_batstatus(self, msg: BatteryStatus):
        self.battery_percentage = msg.remaining
        self.battery_secondsleft = msg.time_remaining_s

    def fc_recv_vehlocpos(self, msg: VehicleLocalPosition):
        """
        Handle an incoming LocalPosition message.
        """
        if isnan(msg.ref_lat) or isnan(msg.ref_lon):
            return
        self.ref_latlon = LatLon(msg.ref_lat, msg.ref_lon)
        
        # Set tgt_alt, based on distance from ground
        self.cur_alt = -msg.z
        self.tgt_alt = self.cur_alt - msg.dist_bottom + self.tgt_dfg

    # --- PUBLISHERS ---
    def fc_publish_heartbeat(self):
        msg = OffboardControlMode()
        msg.position, msg.velocity, msg.acceleration, msg.body_rate = True, False, False, False
        msg.timestamp = self.clock_microseconds()
        self.pub_ocm.publish(msg)

    def fc_publish_vehiclecommand(self, cmd: int, p1: float = 0.0, p2: float = 0.0, p3: float = 0.0, p4: float = 0.0, p5: float = 0.0, p6: float = 0.0, p7: float = 0.0):
        msg = VehicleCommand()
        msg.command = cmd
        msg.target_system, msg.target_component = self.fc_sys_id, self.fc_com_id
        msg.source_system, msg.source_component = 0, 0
        msg.from_external = True
        msg.param1, msg.param2, msg.param3, msg.param4, msg.param5, msg.param6, msg.param7 = p1, p2, p3, p4, p5, p6, p7
        msg.timestamp = self.clock_microseconds()
        self.pub_vehcom.publish(msg)

    def fc_publish_land(self):
        self.fc_publish_vehiclecommand(VehicleCommand.VEHICLE_CMD_NAV_LAND, p4=0.0, p5=self.cur_latlon.lat, p6=self.cur_latlon.lon, p7=0.0)

    def fc_publish_trajectorysetpoint(self):
        if self.tgt_latlon is None:
            self.log("WARNING: tgt_latlon is None")
            return
        tgt_xy = self.tgt_latlon.toXY(self.ref_latlon)  # TrajSP takes only XY relative to the FC's reference point.
        msg = TrajectorySetpoint()
        msg.position = [tgt_xy.x, tgt_xy.y, -self.tgt_alt]
        msg.timestamp = self.clock_microseconds()
        self.pub_trajsp.publish(msg)

    def mc_publish_ready(self):
        # DEPRECATED
        msg = Ready()
        msg.drone_id = self.drone_id
        self.pub_mc_ready.publish(msg)

    def mc_publish_detected(self, coords: LatLon, timestamp: int):
        msg = Detected()
        msg.drone_id = self.drone_id
        msg.lat = coords.lat
        msg.lon = coords.lon
        msg.timestamp = timestamp
        self.pub_mc_detected.publish(msg)
        print(f"Drone {self.drone_id}: Reporting found entity at {coords} at {timestamp}")

    # --- CLIENTS ---
    def mc_request_status(self) -> Future:
        # Note: This is sending the drone's current status as a request, not sending a status request
        msg = Status.Request()
        msg.drone_id, msg.drone_mode = self.drone_id, self.drone_state
        msg.timestamp = self.clock_microseconds()
        msg.last_rtt = float(self.mc_last_rtt)
        msg.battery_percentage = float(self.battery_percentage)
        msg.battery_secondsleft = float(self.battery_secondsleft)
        if self.cur_latlon is None:
            msg.lat, msg.lon = float('nan'), float('nan')
        else:
            msg.lat, msg.lon = self.cur_latlon.lat, self.cur_latlon.lon
        return self.cli_mc_status.call_async(msg)

    def mc_response_status(self, fut: Future):
        if fut.cancelled():
            return
        status_ack = fut.result()

        # Obtain RTT in seconds (from microseconds)
        cur_ts  = self.clock_microseconds() / pow(10, 6)
        prev_ts = status_ack.status_timestamp / pow(10, 6)
        self.mc_last_rtt = cur_ts - prev_ts
    
    def sensor_request_scan(self) -> Future:
        if self.cur_latlon is None:
            return None
        msg = ScanRequest.Request()
        msg.lat = self.cur_latlon.lat
        msg.lon = self.cur_latlon.lon
        return self.cli_sensor_scan.call_async(msg)

    # --- SERVICES ---
    def mc_handle_command(self, request: Command.Request, response: Command.Response):
        self.mc_last_cmd_id = request.cmd_id

        cmd_id = CommandId(request.cmd_id)
        self.log(f"Received {cmd_id.name}.")

        path: List[LatLon] = [self.cur_latlon]
        if cmd_id == CommandId.RTB:
            # Unpack destination coordinates
            cmd_data = b"".join(request.cmd_data)
            dest_coords = struct.unpack("!ff", cmd_data)
            dest_latlon = LatLon(dest_coords[0], dest_coords[1])

            # Add instructions
            self.instr_queue.append(Instruction(InstructionId.MOVETO, dest_latlon))
            self.instr_queue.append(Instruction(InstructionId.LAND, dest_latlon))
            path.append(dest_latlon)
        elif cmd_id == CommandId.SEARCH_SECTOR:
            # Unpack destination coordinates
            cmd_data = b"".join(request.cmd_data)
            dest_coords = struct.unpack("!ff", cmd_data)
            dest_latlon = LatLon(dest_coords[0], dest_coords[1])

            # Initialise pathfinder
            self.pathfinder = PathfinderState(dest_latlon, None)
            for pos in self.pathfinder.cached_path:
                path.append(pos)

            # Add instructions
            self.instr_queue.append(Instruction(InstructionId.MOVETO, dest_latlon))
            self.instr_queue.append(Instruction(InstructionId.SEARCH, dest_latlon))
        elif cmd_id == CommandId.MOVE_TO:
            # Unpack destination coordinates
            cmd_data = b"".join(request.cmd_data)
            dest_coords = struct.unpack("!ff", cmd_data)
            dest_latlon = LatLon(dest_coords[0], dest_coords[1])

            # Add instructions
            self.instr_queue.append(Instruction(InstructionId.MOVETO, dest_latlon))
            path.append(dest_latlon)
        elif cmd_id == CommandId.LAND:
            self.instr_queue.clear()
            self.instr_queue.append(Instruction(InstructionId.LAND, self.cur_latlon))
        elif cmd_id == CommandId.DISCONNECT:
            self.instr_queue.clear()
            self.instr_queue.append(Instruction(InstructionId.DISCONNECT, self.cur_latlon))

        # Generate response
        path_bytes = serialise_path(path)

        response.drone_id, response.cmd_id, response.path = self.drone_id, request.cmd_id, path_bytes
        return response

    # Helper Methods
    def clock_microseconds(self) -> int:
        """ Returns the current clock in microseconds. """
        return self.get_clock().now().nanoseconds // 1000
    
    def log(self, msg: str):
        msg = f"DRONE {self.drone_id}: {msg}"
        self.get_logger().info(msg)

    def error(self, msg: str):
        self.change_state(DroneState.EXIT, f"Exiting due to error {msg}")

def main(args=None):
    rclpy.init(args=args)
    drone_node = DroneNode()

    rclpy.spin(drone_node)

    drone_node.destroy_node()

if __name__ == '__main__':
    main()