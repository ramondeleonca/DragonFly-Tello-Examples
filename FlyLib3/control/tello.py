"""# The Tello control module for FlyLib3.
Copyright (c) 2024, DragonFly Saltillo / Ramon de Leon.
"""

import os
import time
import uuid
import socket
import logging
import datetime
import threading
import numpy as np
from enum import Enum
from dataclasses import dataclass

# this class is really cool, i really like it
@dataclass
class FlyLib3TelloState:
    """### The Tello state dataclass for FlyLib3.
    This dataclass is used to store the state of the Tello drone.
    This class is not meant to be used directly, rather the methods to get individual fields in the FlyLib3Tello class should be used.
    """
    # INT fields
    pitch: int
    roll: int
    yaw: int
    vgx: int
    vgy: int
    vgz: int
    templ: int
    temph: int
    tof: int
    h: int
    bat: int
    time: int

    # FLOAT fields
    baro: float
    agx: float
    agy: float
    agz: float

class FlyLib3TelloMoveDirection(Enum):
    """### The Tello direction enum for FlyLib3.
    This enum is used to represent the directions of the Tello drone.
    """
    UP = "up"
    DOWN = "down"
    LEFT = "left"
    RIGHT = "right"
    FORWARD = "forward"
    BACKWARD = "back"

class FlyLib3TelloRotateDirection(Enum):
    """### The Tello rotation enum for FlyLib3.
    This enum is used to represent the rotation directions of the Tello drone.
    """
    CLOCKWISE = "cw"
    COUNTERCLOCKWISE = "ccw"

class FlyLib3TelloFlipDirection(Enum):
    """### The Tello flip enum for FlyLib3.
    This enum is used to represent the flip directions of the Tello drone.
    """
    LEFT = "l"
    RIGHT = "r"
    FORWARD = "f"
    BACKWARD = "b"

# this class is also really cool but it's really big and i don't like it
class FlyLib3Tello:
    """### The fully featured Tello control class for FlyLib3.
    This control class MUST be used in TEC Drone Championship competitions.
    """

    # Default Tello IP and port
    DEFAULT_CONTROL_STATE_HOST = "192.168.10.1"
    DEFAULT_CONTROL_PORT = 8889
    DEFAULT_STATE_PORT = 8890

    # Default video stream host and port
    DEFAULT_VIDEO_HOST = "0.0.0.0"
    DEFAULT_VIDEO_PORT = 11111

    # Default timeout and max retries
    DEFAULT_TIMEOUT = 5
    DEFAULT_MAX_RETRIES = 3
    DEFAULT_MIN_COMMAND_DELAY = 0.15

    # Control and state socket information
    control_state_host: str
    control_port: int
    state_port: int

    # Video socket information
    video_host: str
    video_port: int

    # Sockets and threads
    control_socket: socket.socket
    response_thread: threading.Thread

    state_socket: socket.socket
    state_thread: threading.Thread

    # Current state
    responses = []
    state: FlyLib3TelloState
    is_flying: bool = False

    # Logging configuration
    log: bool
    log_path: str
    log_to_console: bool
    log_to_file: bool
    log_level: int
    logger: logging.Logger
    log_formatter: logging.Formatter
    file_logger: logging.FileHandler = None
    console_logger: logging.StreamHandler = None
    id: str

    def __init__(
            self,

            control_state_host: str = DEFAULT_CONTROL_STATE_HOST,
            control_port: int = DEFAULT_CONTROL_PORT,
            state_port: int = DEFAULT_STATE_PORT,

            video_host: str = DEFAULT_VIDEO_HOST,
            video_port: int = DEFAULT_VIDEO_PORT,

            log: bool = True,
            log_path: str = os.path.join(os.path.expanduser("~"), "/FlyLib3/logs/", f"tello-{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"),
            log_to_console: bool = False,
            log_to_file: bool = True,
            log_level: int = logging.WARNING,

            id: str = str(uuid.uuid4())
        ):
        """### Initialize the Tello control class.
        Creates an instance with a control, state and video socket.

        :param control_state_host: The IP address of the Tello drone.
        :param control_port: The port of the Tello drone for control commands.
        :param state_port: The port of the Tello drone for state information.

        :param video_host: The host of the video stream from the Tello drone.
        :param video_port: The port of the video stream from the Tello drone.

        :param log: Whether to log the Tello control commands and state.
        :param log_path: The path to save the log file.
        :param log_to_console: Whether to log the Tello control commands and state to the console.
        :param log_level: The level of logging to use.

        :param id: The unique identifier of the Tello drone control class, used for logging and debugging.
        """
        # Logging configuration
        self.log = log
        self.log_path = log_path
        self.log_to_console = log_to_console
        self.log_to_file = log_to_file
        self.log_level = log_level

        # Set the unique identifier
        self.id = id

        # Create the logger
        self.logger = logging.getLogger("FlyLib3Tello-" + id)
        self.logger.setLevel(self.log_level)
        
        # Create the formatter
        formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")

        # Create the file handler
        if self.log_to_file:
            # Create directories if they don't exist
            os.makedirs(os.path.dirname(self.log_path), exist_ok=True)

            # Create the file logger
            self.file_logger = logging.FileHandler(self.log_path)
            self.file_logger.setLevel(0)
            self.file_logger.setFormatter(formatter)
            self.logger.addHandler(self.file_logger)

        # Create the console handler
        if self.log_to_console:
            self.console_logger = logging.StreamHandler()
            self.console_logger.setLevel(log_level)
            self.console_logger.setFormatter(formatter)
            self.logger.addHandler(self.console_logger)
        
        # Log the initialization
        self.logger.info("Initialized FlyLib3Tello control class.")

        # Set the control and state socket information
        self.control_state_host = control_state_host
        self.control_port = control_port
        self.state_port = state_port

        # Set the video socket information
        self.video_host = video_host
        self.video_port = video_port

        # Create the control and state sockets
        self.logger.info(f"Creating control socket on {control_state_host}:{control_port}")
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_socket.bind((self.control_state_host, self.control_port))

        self.logger.info(f"Creating state socket on {control_state_host}:{state_port}")
        self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_socket.bind((self.control_state_host, self.state_port))

        # Create the response and state threads
        self.logger.info("Starting response and state threads.")
        self.response_thread = threading.Thread(target=self._response_thread, daemon=True)
        self.response_thread.start()

        self.state_thread = threading.Thread(target=self._state_thread, daemon=True)
        self.state_thread.start()
    
    # * Threads - Methods
    def _response_thread(self):
        """## INTERNAL METHOD
        This method is used to receive responses from the Tello drone.
        """
        while True:
            try:
                response, addr = self.control_socket.recvfrom(1024)
                response = response.decode(encoding="utf-8").strip()
                self.logger.info(f"Received response: {response}")
                self.responses.append(response)
            except Exception as e:
                print(f"Error receiving response: {e}")
    
    def _state_thread(self):
        """## INTERNAL METHOD
        This method is used to receive the state of the Tello drone.
        """
        while True:
            try:
                state, addr = self.state_socket.recvfrom(1024)
                state = state.decode(encoding="utf-8").strip()
                self.state = self._parse_state(state)
                self.logger.info(f"Received state: {state}")
            except Exception as e:
                print(f"Error receiving state: {e}")
    
    # * State - Parsing
    @staticmethod
    def _parse_state(state: str):
        """## INTERNAL METHOD
        This method is used to parse the state of the Tello drone into a FlyLib3TelloState.
        """
        # Split the state into key-value pairs
        state = state.strip().removesuffix(";").split(';')
        state_dict = {}

        # Parse the key-value pairs
        for pair in state:
            key, value = pair.split(':')
            state_dict[key] = FlyLib3TelloState.__annotations__[key](value)
        
        return FlyLib3TelloState(**state_dict)

    # * State - Getters
    def get_state(self, as_dict: bool = False):
        """### Get the state of the Tello drone.
        This method is used to get the state of the Tello drone.
        If as_dict is True, the state is returned as a dictionary.
        
        (State readings do not consume bandwidth)
        """
        if as_dict:
            return self.state.__dict__
        return self.state

    def get_state_field(self, field: str) -> int | float:
        """### Get a specific field of the Tello state.
        This method is used to get a specific field of the Tello state.
        
        (State readings do not consume bandwidth)
        """
        return getattr(self.state, field)
    
    # * Telemetry - Rotation
    def get_pitch(self):
        """### Get the pitch of the Tello drone.
        This method is used to get the pitch of the Tello drone in degrees.
        
        (State readings do not consume bandwidth)
        """
        return self.get_state_field('pitch')
    
    def get_yaw(self):
        """### Get the yaw of the Tello drone.
        This method is used to get the yaw of the Tello drone in degrees.
        
        (State readings do not consume bandwidth)
        """
        return self.get_state_field('yaw')
    
    def get_roll(self):
        """### Get the roll of the Tello drone.
        This method is used to get the roll of the Tello drone in degrees.
        
        (State readings do not consume bandwidth)
        """
        return self.get_state_field('roll')
    
    # * Telemetry - Velocity
    def get_speed_x(self):
        """### Get the speed in the x direction of the Tello drone.
        This method is used to get the speed in the x direction of the Tello drone in cm/s.
        
        (State readings do not consume bandwidth)
        """
        return self.get_state_field('vgx')

    def get_speed_y(self):
        """### Get the speed in the y direction of the Tello drone.
        This method is used to get the speed in the y direction of the Tello drone in cm/s.
        
        (State readings do not consume bandwidth)
        """
        return self.get_state_field('vgy')

    def get_speed_z(self):
        """### Get the speed in the z direction of the Tello drone.
        This method is used to get the speed in the z direction of the Tello drone in cm/s.
        
        (State readings do not consume bandwidth)
        """
        return self.get_state_field('vgz')
    
    # * Telemetry - Acceleration
    def get_acceleration_x(self):
        """### Get the acceleration in the x direction of the Tello drone.
        This method is used to get the acceleration in the x direction of the Tello drone in cm/s^2.
        
        (State readings do not consume bandwidth)
        """
        return self.get_state_field('agx')
    
    def get_acceleration_y(self):
        """### Get the acceleration in the y direction of the Tello drone.
        This method is used to get the acceleration in the y direction of the Tello drone in cm/s^2.
        
        (State readings do not consume bandwidth)
        """
        return self.get_state_field('agy')
    
    def get_acceleration_z(self):
        """### Get the acceleration in the z direction of the Tello drone.
        This method is used to get the acceleration in the z direction of the Tello drone in cm/s^2.
        
        (State readings do not consume bandwidth)
        """
        return self.get_state_field('agz')
    
    # * Telemetry - Temperature
    def get_temperature_low(self):
        """### Get the low temperature of the Tello drone.
        This method is used to get the low temperature of the Tello drone in degrees Celsius.
        
        (State readings do not consume bandwidth)
        """
        return self.get_state_field('templ')
    
    def get_temperature_high(self):
        """### Get the high temperature of the Tello drone.
        This method is used to get the high temperature of the Tello drone in degrees Celsius.
        
        (State readings do not consume bandwidth)
        """
        return self.get_state_field('temph')
    
    def get_temperature(self):
        """### Get the temperature of the Tello drone.
        This method is used to get the temperature of the Tello drone in degrees Celsius.
        
        (State readings do not consume bandwidth)
        """
        return (self.get_temperature_low() + self.get_temperature_high()) / 2
    
    # * Telemetry - Height
    def get_height(self):
        """### Get the height of the Tello drone.
        This method is used to get the height of the Tello drone in cm.
        
        (State readings do not consume bandwidth)
        """
        return self.get_state_field('h')
    
    def get_distance_tof(self):
        """### Get the distance from the Tello drone to the floor.
        This method is used to get the distance from the Tello drone to the floor in cm.
        
        (State readings do not consume bandwidth)
        """
        return self.get_state_field('tof')
    
    def get_barometer(self):
        """### Get the barometer reading of the Tello drone.
        This method is used to get the barometer reading of the Tello drone in cm.
        Barometer readings are in meters, so the value is multiplied by 100 to get the reading in cm.
        Barometers return the absolute height above sea level.
        
        (State readings do not consume bandwidth)
        """
        return self.get_state_field('baro') * 100
    
    # * Telemetry - Time
    def get_flight_time(self):
        """### Get the flight time of the Tello drone.
        This method is used to get the flight time of the Tello drone in seconds.
        
        (State readings do not consume bandwidth)
        """
        return self.get_state_field('time')

    # * Telemetry - Battery
    def get_battery(self):
        """### Get the battery percentage of the Tello drone.
        This method is used to get the battery percentage of the Tello drone.
        
        (State readings do not consume bandwidth)
        """
        return self.get_state_field('bat')
    
    # * Control - Send Commands
    def _send_command_without_response(self, command: str):
        """## INTERNAL METHOD
        This method is used to send a command to the Tello drone without waiting for a response.
        """
        self.logger.info(f"Sending command: {command}")
        self.control_socket.send(command.encode(encoding="utf-8"))
    
    def _send_command_with_response(self, command: str, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """## INTERNAL METHOD
        This method is used to send a command to the Tello drone and wait for a response.
        ### BLOCKING METHOD
        """
        self.logger.info(f"Sending command and waiting for response: {command}")
        self.control_socket.send(command.encode(encoding="utf-8"))

        # Wait for a response
        retries = 0
        while retries < max_retries:
            if len(self.responses) > 0:
                response = self.responses.pop(0)
                return response
            retries += 1
            time.sleep(timeout)
        return None
    
    def _send_control_command_with_response(self, command: str, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """## INTERNAL METHOD
        This method is used to send a control command to the Tello drone and wait for a response.
        ### BLOCKING METHOD
        """
        self.logger.info(f"Sending control command and waiting for response: {command}")
        return "ok" in str(self._send_command_with_response(command, timeout, max_retries)).strip().lower
    
    def _send_read_command_with_response(self, command: str, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """## INTERNAL METHOD
        This method is used to send a read command to the Tello drone and wait for a response.
        ### BLOCKING METHOD
        """
        self.logger.info(f"Sending read command and waiting for response: {command}")
        return self._send_command_with_response(command, timeout, max_retries)
    
    # * Control - Connection
    def connect(self, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """### Connect to the Tello drone.
        This method is used to connect to the Tello drone and initialize SDK Mode.
        ### BLOCKING METHOD
        """
        return self._send_control_command_with_response("command", timeout, max_retries)
    
    def keep_alive(self, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """### Keep the connection alive with the Tello drone.
        This method is used to keep the connection alive with the Tello drone.
        ### BLOCKING METHOD
        """
        return self._send_control_command_with_response("keepalive", timeout, max_retries)
    
    # * Control - Motors
    def motors_on(self, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """### Turn on the motors of the Tello drone.
        This method is used to turn on the motors of the Tello drone.
        This is helpful to cool the electronics and to signal the drone is ready to take off.
        ### BLOCKING METHOD
        """
        return self._send_control_command_with_response("motoron", timeout, max_retries)
    arm = motors_on

    def motors_off(self, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """### Turn off the motors of the Tello drone.
        This method is used to turn off the motors of the Tello drone.
        This is helpful to signal the drone is disarmed and stopped.
        ### BLOCKING METHOD
        """
        return self._send_control_command_with_response("motoroff", timeout, max_retries)
    disarm = motors_off

    def emergency(self, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """### Emergency stop the Tello drone.
        This method is used to emergency stop the Tello drone.
        This is helpful to stop the motors completely if needed.
        ### ALL PROGRAMS SHOULD HAVE THIS METHOD ON STANDBY ON A SEPARATE THREAD FOR SAFETY.
        ### BLOCKING METHOD
        """
        return self._send_control_command_with_response("emergency", timeout, max_retries)
    stop = emergency
    kill = emergency

    # * Control - Takeoff and Landing
    def takeoff(self, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """### Takeoff the Tello drone.
        This method is used to takeoff the Tello drone.
        ### BLOCKING METHOD
        """
        response = self._send_control_command_with_response("takeoff", timeout, max_retries)
        self.is_flying = str(response).strip().lower() == "ok"
        return response
    
    def land(self, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """### Land the Tello drone.
        This method is used to land the Tello drone.
        ### BLOCKING METHOD
        """
        response = self._send_control_command_with_response("land", timeout, max_retries)
        self.is_flying = not str(response).strip().lower() == "ok"
        return response
    
    # * Control - Movement
    def move(self, direction: FlyLib3TelloMoveDirection, distance: int, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """### Move the Tello drone in a specific direction.
        This method is used to move the Tello drone in a specific direction.
        :distance: The distance to move in cm [20-500].
        ### BLOCKING METHOD
        """
        return self._send_control_command_with_response(f"{direction.value} {distance}", timeout, max_retries)

    def rotate(self, direction: FlyLib3TelloRotateDirection, degrees: int, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """### Rotate the Tello drone in a specific direction.
        This method is used to rotate the Tello drone in a specific direction.
        :degrees: The degrees to rotate [1, 360].
        ### BLOCKING METHOD
        """
        return self._send_control_command_with_response(f"{direction.value} {degrees}", timeout, max_retries)

    def flip(self, direction: FlyLib3TelloFlipDirection, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """### Flip the Tello drone in a specific direction.
        This method is used to flip the Tello drone in a specific direction.
        ### BLOCKING METHOD
        """
        return self._send_control_command_with_response(f"flip {direction.value}", timeout, max_retries)
    
    def go_to(self, x: int, y: int, z: int, speed: int, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """### Go to a relative position with the Tello drone.
        This method is used to go to a relative position with the Tello drone.
        :x: The x coordinate to go to [-500, 500].
        :y: The y coordinate to go to [-500, 500].
        :z: The z coordinate to go to [-500, 500].
        :speed: The speed to go to the position [10, 100].
        ### BLOCKING METHOD
        """
        return self._send_control_command_with_response(f"go {x} {y} {z} {speed}", timeout, max_retries)
    
    def curve_to(self, x1: int, y1: int, z1: int, x2: int, y2: int, z2: int, speed: int, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """### Curve to a position with the Tello drone.
        This method is used to curve to a position with the Tello drone.
        :x1: The x coordinate of the first point [-500, 500].
        :y1: The y coordinate of the first point [-500, 500].
        :z1: The z coordinate of the first point [-500, 500].
        :x2: The x coordinate of the second point [-500, 500].
        :y2: The y coordinate of the second point [-500, 500].
        :z2: The z coordinate of the second point [-500, 500].
        :speed: The speed to go to the position [10, 60].
        ### BLOCKING METHOD
        """
        return self._send_control_command_with_response(f"curve {x1} {y1} {z1} {x2} {y2} {z2} {speed}", timeout, max_retries)
    
    def set_speed(self, speed: int, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """### Set the speed of the Tello drone.
        This method is used to set the speed of the Tello drone in cm/s.
        :speed: The speed to set [10, 100].
        ### BLOCKING METHOD
        """
        return self._send_control_command_with_response(f"speed {speed}", timeout, max_retries)
    
    # * Control - RC Control
    _last_rc_command_time = 0
    def send_rc_control(self, left_right: int, forward_backward: int, up_down: int, yaw: int, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """### Send RC control commands to the Tello drone.
        This method is used to send RC control commands to the Tello drone.
        :left_right: The left-right control [-100, 100].
        :forward_backward: The forward-backward control [-100, 100].
        :up_down: The up-down control [-100, 100].
        :yaw: The yaw control [-100, 100].
        """
        if time.time() - self._last_rc_command_time > self.DEFAULT_MIN_COMMAND_DELAY:
            clamp = lambda n: max(min(100, n), -100)
            self._last_rc_command_time = time.time()
            # Command is sent without waiting for a response to prevent blocking
            return self._send_command_without_response(f"rc {clamp(left_right)} {clamp(forward_backward)} {clamp(up_down)} {clamp(yaw)}", timeout, max_retries)
        
    # * Control - Set Commands
    def set_wifi(self, ssid: str, password: str, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """### Set the WiFi of the Tello drone.
        This method is used to set the WiFi of the Tello drone.
        :ssid: The SSID of the WiFi network.
        :password: The password of the WiFi network.
        ### BLOCKING METHOD
        """
        return self._send_control_command_with_response(f"wifi {ssid} {password}", timeout, max_retries)
    
    def set_ports(self, state_port: int, video_port: int, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """### Set the ports of the Tello drone.
        This method is used to set the ports of the Tello drone.
        :control_port: The control port of the Tello drone.
        :state_port: The state port of the Tello drone.
        ### BLOCKING METHOD
        """
        return self._send_control_command_with_response(f"port {state_port} {video_port}", timeout, max_retries)
    
    # * Control - System
    def reboot(self, timeout: float = DEFAULT_TIMEOUT, max_retries: int = DEFAULT_MAX_RETRIES):
        """### Reboot the Tello drone.
        This method is used to reboot the Tello drone.
        ### BLOCKING METHOD
        """
        return self._send_control_command_with_response("reboot", timeout, max_retries)
    
    # * Class - Cleanup
    def close(self):
        """### Close the Tello control class.
        This method is used to close the Tello control class and cleanup resources.
        """
        self.logger.info("Closing FlyLib3Tello control class.")
        self.control_socket.close()
        self.state_socket.close()

        if self.file_logger:
            self.file_logger.close()
        
        if self.console_logger:
            self.console_logger.close()
    
    def __del__(self):
        """### Delete the Tello control class.
        This method is used to delete the Tello control class and cleanup resources.
        """
        self.logger.info("Deleting FlyLib3Tello control class.")
        self.close()

    
if __name__ == "__main__":
    tello_state = "pitch:31;roll:177;yaw:169;vgx:0;vgy:0;vgz:0;templ:81;temph:83;tof:308;h:0;bat:42;baro:1456.73;time:13;agx:519.00;agy:21.00;agz:874.00;"
    parsed_state = FlyLib3Tello._parse_state(tello_state)
    print(parsed_state)

    tello = FlyLib3Tello(log_to_console=False, log_level=logging.DEBUG)
    tello.connect()
    tello.arm()
    print(tello.log_path)