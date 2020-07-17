from math import radians, sin, cos
import queue
import time
from threading import Thread

from pymavlink import mavutil

from auvkit.control import PID
from auvkit.data_fetcher import DataFetcher, AtlasScientificSensor
from auvkit.data_writer import csv_write
from auvkit.helper import integrate, get_delta_time, calc_dist, angle_diff, haversine, bear_angle
from auvkit.motor_controller import set_rc_channel_pwm, set_neutral
from auvkit.ping import Ping
from auvkit.set_mode import set_mode


class PathNode:
    """
    PathNode class is a class that describes a linear trajectory node. To describe it, you need to give
    a certain displacement and change in angle, along with a vertical displacement.
    A path is made up of a queue of PathNodes.

    Attributes:
        self.dist
        self.delta_angle
        self.vertical_diff
    """

    def __init__(self, dist, delta_angle, vertical_diff):
        self.dist = dist
        self.delta_angle = delta_angle
        self.vertical_diff = vertical_diff


class Path:
    """
    Path class is a class that describes a path that we want the AUV to follow. This class
    is the main class a developer will use to control the AUV and interact with all systems.
    We are basically creating a queue of pathNode objects that are basically stating how
    much the rover should move. Altogether, the queue of the nodes will map out a route.
    The class has methods to push a node to move, to move along only one node, and to
    move the entire route. This functionality relies on the concept of linearization.
    We also store the initial heading of the rover.

    Methods:
        def __init__(self, sonar_serial_port="/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DM01MF1J-if00-port0",
                 sonar_baudrate=115200, confidence_threshold=90)
        def enable_gps(self)
        def add_atlas_sensor(self, identifier, serial_port)
        def add_atlas_sensor_object(self, atlas_object)
        def set_stabilize_mode(self)
        def set_depth_hold(self)
        def start(self)
        def start_threads(self)
        def end_threads(self)
        def end(self)
        def push(self, path_node_)
        def set_heading_pid(self, kP, kI, kD)
        def set_turning_pid(self, kP, kI, kD)
        def set_vertical_pid(self, kP, kI, kD)
        def set_allow_go_back(self, new_value)
        def move_one_node(self, curr_node, speed)
        def move_down(self, vertical_diff)
        def go_back(self)
        def move_entire_route(self)
        def update_dynamics(self)
        def heading_pid(self, speed, delta_dist)
        def turn(self, delta_degrees)
        def turn_north(self)
        def go_to(self, dest_lat, dest_lon)
    """

    def __init__(self, sonar_serial_port="/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DM01MF1J-if00-port0", 
                 sonar_baudrate=115200, confidence_threshold=90):
        """
        :param sonar_serial_port: THe serial port the Ping Echosonar is connected to.
        :param sonar_baudrate: The baudrate of the sonar sensor.
        :param confidence_threshold: The defined minimum confidence level we want to allow the sonar to give.
        """

        # Initialize sonar by checking if we are able to connect to and read from the echosonar device.
        self.has_sonar = True

        try:
            self.sonar = Ping(device_port=sonar_serial_port, baudrate=sonar_baudrate,
                              confidence_threshold=confidence_threshold)
            print("Ping device successfully initialized.")
            print("Connected on serial port " + sonar_serial_port + ".\n")

            # Set parameters to obtain better distance
            self.sonar.set_ping_interval(29)
            self.sonar.set_speed_of_sound(1500)
            self.sonar.set_gain_index(2)

            # Obtain the initial starting height from the seafloor and convert to meters.
            self.seafloor_height = self.sonar.get_distance() * 1.0/1000.0
        except Exception:
            # If an exception occurs and we are unable to read from it, set the seafloor height
            # to -1 to show that.
            print("Unable to initialize ping device. Check your configuration.\n")
            self.seafloor_height = -1

        # Path queue is the queue of nodes we want to traverse
        # while reverse queue stores the nodes we want to traverse
        # if we want to traverse the path backwards
        self.path_queue = queue.Queue()
        self.reverse_queue = queue.LifoQueue()

        # Create a MAVLink master connection. Then create a DataFetcher class to fetch data.
        self.motor_master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        self.data_fetcher = DataFetcher([])

        # This is a thread to update the dynamics of the robot as we
        # move the robot. This is useful in controlling the robot and just
        # getting data in general.
        self.update_dynamics_thread = Thread(target=self.update_dynamics)

        # initial_heading stores the heading the robot is at when we begin the path.
        # current_heading stores the current heading.
        # dest_heading stores the ideal heading the robot should currently be at as
        # it traverses a node.
        self.initial_heading = 0
        self.current_heading = 0
        self.dest_heading = 0

        # do_update is a boolean value that when true allows the code
        # to keep getting the data from the IMU and updating the relative
        # positions, velocities, and angles of the robot.
        # Previous time holds the previous time we updated the dynamics.
        # Starting time holds the starting time at which we started the path.
        # This is useful for integrating so we can calculate delta time.
        self.do_update = True
        self.starting_time = time.time()
        self.previous_time = time.time()

        # going_back is a boolean that when set True enables traversal of the path in reverse.
        # By default, it is False.
        self.going_back = False

        # These are the predicted x, y and z position (relative to the starting position)
        # the robot will ideally (IDEALLY) be at after it traverses a current node.
        self.pred_x = 0.0
        self.pred_y = 0.0
        self.pred_z = 0.0

        # Here we will define out state matrix. The matrix is as follows:
        # [x_vel, y_vel, z_vel, x_pos, y_pos, z_pos, x_angle, y_angle, z_angle]
        # The angles are based on integrating the the gyroscopic rate.
        self.state_matrix = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Define PID iterators for the different loops.
        self.heading_pid_iterator = PID(0, 0, 0)
        self.turning_pid_iterator = PID(0, 0, 0)
        self.vertical_pid_iterator = PID(0, 0, 0)

        # Set GPS capability to initially be False. Can be set true by calling enable_gps().
        self.gps_enabled = False

        self.start()

    def enable_gps(self):
        """
        This function when called, checks if GPS satellites are visible, and only then enables GPS.
        Otherwise, GPS is not enabled.
        :return: A boolean whether or not GPS is successfully enabled
        """
        time_search = 5 * self.data_fetcher.gps_refresh_rate
        start_time = time.time()
        while time.time() - start_time < time_search:
            if self.data_fetcher.get_num_gps_satellites() > 0:
                self.gps_enabled = True
                print("GPS successfully enabled.")
                return True
        print("No GPS satellites are visible. GPS functionality will not be enabled.")
        return False

    def add_atlas_sensor(self, identifier, serial_port):
        """
        This function adds an AtlasScientific Sensor to the path.
        :param identifier: A unique identifier for the AtlasScientific device.
        :param serial_port: The serial port the sensor is connected to.
        """
        self.data_fetcher.atlas_sensor_list.append(AtlasScientificSensor(identifier, serial_port))

    def add_atlas_sensor_object(self, atlas_object):
        """
        This function adds an AtlasScientific Sensor to the path.
        :param atlas_object: The AtlasScientificSensor object to add.
        """
        self.data_fetcher.atlas_sensor_list.append(atlas_object)

    def set_stabilize_mode(self):
        """
        This function enables STABILIZE mode
        """
        set_mode("STABILIZE")

    def set_depth_hold(self):
        """
        This function enables ALT_HOLD mode, which holds the depth of the AUV.
        """
        set_mode("ALT_HOLD")

    def start(self):
        """
        This function is called at the end of initialization of a Path object.
        It starts all necessary systems.
        :return: None
        """

        # Wait for the heartbeat and then arm the robot.
        print("Waiting for heartbeat")
        self.motor_master.wait_heartbeat()
        print("Heartbeat Detected.\n")

        print("Arming BlueROV.")
        self.motor_master.arducopter_arm()
        print("BlueROV Armed.")

        # Set starting and previous time to when we start updating dynamics.
        self.starting_time = time.time()
        self.previous_time = time.time()

        # Set all headings to current heading because in the beginning all
        # headings are equal to current heading.
        self.initial_heading = self.data_fetcher.get_latest_heading()
        self.current_heading = self.data_fetcher.get_latest_heading()
        self.dest_heading = self.data_fetcher.get_latest_heading()

        # Start necessary background threads.
        self.start_threads()
        return

    def start_threads(self):
        """
        This function starts the background thread to update the dynamics, i.e. localization,
        of the robot.
        :return: None
        """

        # Start updating dynamics.

        print("Starting Thread to Calculate and Update Position")
        self.do_update = True
        self.update_dynamics_thread.start()
        return

    def end_threads(self):
        """
        This function ends the background thread to update the dynamics, i.e. localization,
        of the robot.
        :return: None
        """

        # Set do update to False so the while loop in update_dynamics
        # stops looping and then join the thread with this.
        print("Final coordinates: ", self.state_matrix[3:5])
        print("Ending update dynamics thread")
        self.do_update = False
        self.update_dynamics_thread.join()
        return

    def end(self):
        """
        This is the function to call when the path is finished and we want
        to stop and disarm the robot and stop the motors.
        :return: None
        """
        # Set the motors to neutral and disarm the robot.
        print("Setting motors to neutral")
        set_neutral(self.motor_master)
        
        print("Disarming BlueROV.")
        self.motor_master.arducopter_disarm()
        print("BlueROV disarmed.")

        self.end_threads()
        return

    def write_to_csv(self, filename=None):
        """
        This function starts writing data to a CSV.
        :param filename: Optional, the filename of the CSV file. If not specified, it will be auto generated
        based on time
        :return: None
        """
        csv_write(self.data_fetcher, filename=filename)

    def push(self, path_node_):
        """
        This function adds a PathNode to the queue.
        :param path_node_: The PathNode to add.
        :return: None
        """
        self.path_queue.put(path_node_)
        return

    def set_heading_pid(self, kP, kI, kD):
        """
        This function sets the PID values for the heading PID loop.
        :param kP: The proportionality constant.
        :param kI: The integration constant.
        :param kD: The derivative constant.
        :return: None
        """
        self.heading_pid_iterator = PID(kP, kI, kD)
        return

    def set_turning_pid(self, kP, kI, kD):
        """
        This function sets the PID values for the turning PID loop.
        :param kP: The proportionality constant.
        :param kI: The integration constant.
        :param kD: The derivative constant.
        :return: None
        """
        self.turning_pid_iterator = PID(kP, kI, kD)

    def set_vertical_pid(self, kP, kI, kD):
        """
        This function sets the PID values for the vertical movement PID loop.
        :param kP: The proportionality constant.
        :param kI: The integration constant.
        :param kD: The derivative constant.
        :return: None
        """
        self.vertical_pid_iterator = PID(kP, kI, kD)

    def set_allow_go_back(self, allow_go_back):
        """
        This function sets the boolean self.allow_go_back to store if we
        want to store reverse route in case we want to go back. If this is false, we can't go back.
        :param allow_go_back: The value we want to set self.allow_go_back to.
        :return: None
        """
        self.going_back = allow_go_back
        return

    def move_one_node(self, curr_node):
        """
        This is the code to move along a PathNode.
        Specifically, we first set our predicted x, y and z positions that we will
        be at after traversing this node. Then, we change the angle first. Then, we
        move forward the given amount of meters. Then, we move vertically
        the given amount in meters.
        :param curr_node: The PathNode we want to move along.
        :return: None
        """

        # Calculate first the desired heading we want to be at, and the predicted x, y and z values.

        delta_angle = curr_node.delta_angle
        self.dest_heading = (self.dest_heading + delta_angle) % 360

        self.pred_x += curr_node.dist * sin(self.dest_heading - self.initial_heading)
        self.pred_y += curr_node.dist * cos(self.dest_heading - self.initial_heading)
        self.pred_z += curr_node.vertical_diff

        # Add nodes to reverse path queue if the option is set.
        if self.going_back:
            # Code to add the reverse path to the reverse queue. This will be helpful
            # when the rover gets lost, or when the mission is ended and we want it to return.
            reverse_angle = (curr_node.delta_angle + 180) % 360

            path_node_reverse = PathNode(curr_node.dist, reverse_angle, curr_node.vertical_diff * -1)
            self.reverse_queue.put(path_node_reverse)

        # First, we turn to our desired heading.
        print("Turning now...")
        print("Current Angle: %d" % self.current_heading)
        print("Destination Angle: %d" % self.dest_heading)
        self.turn(delta_angle)

        # Then, we move forward our desired displacement.
        print("Moving forward at %d m." % curr_node.dist)
        if not curr_node.dist == 0:
            self.heading_pid(1600, 5, curr_node.dist)

        # Finally, move the desired vertical displacement.
        vertical_diff = curr_node.vertical_diff

        if not vertical_diff == 0:
            # Code to move the rover up and down a certain distance. If the value is negative,
            # we want to move it up. If it is positive, we should move down.
            print("Moving vertically for %d m." % vertical_diff)
            self.move_down()

        return

    def move_down(self):
        """
        This is the function to move the robot up or down
        a certain amount in meters.
        :param vertical_diff: The vertical displacement we want to move
        :return: None
        """
        start = time.time()
        
        end = start + 6

        # If we have sonar, use that
        if not self.seafloor_height == -1:

            dest_z = self.seafloor_height - self.pred_z

            diff = dest_z - (self.sonar.get_distance()["distance"] * 1.0/1000.0)

            # Keep moving until within 0.25 m
            while diff > 0.25:
                if dest_z > (self.sonar.get_distance()["distance"] * 1.0/1000.0):

                    # Move up or down depending on displacement.
                    set_rc_channel_pwm(self.motor_master, 3, 1500 + -1 * self.vertical_pid_iterator.iterate(diff))

                diff = dest_z - (self.sonar.get_distance()["distance"] * 1.0/1000.0)
        else:
            # Keep moving for a given time
            while time.time() < end:
                if self.pred_z > self.state_matrix[5]:
                    # Move up or down depending on integrated Z value
                    set_rc_channel_pwm(self.motor_master, 3, 1500 + -1 *
                                       self.vertical_pid_iterator.iterate(self.pred_z - self.state_matrix[5]))
        set_rc_channel_pwm(self.motor_master, 3, 1500)

        return

    def go_back(self):
        """
        This is the function that traverses the nodes in the reverse queue.
        :return: None
        """

        # Return if reverse traversal is disabled.
        if not self.going_back:
            return

        # Traverse queue in reverse.
        while not self.reverse_queue.empty():
            self.move_one_node(self.reverse_queue.get())
        return

    def move_entire_route(self):
        """
        This is the function to move along the queue of path nodes
        We just keep moving along nodes until there are no more left in the queue.
        Then, we just use go_back() to get to our initial position if the option is enabled.
        :return: None
        """
        print("Starting Path...")

        # Sleep for 2 seconds just to let the dynamics thread start. 
        time.sleep(2)

        # Traverse the queue of PathNodes
        while not self.path_queue.empty():
            self.move_one_node(self.path_queue.get())

        # Move back if enabled
        print("Starting to move back.")
        self.go_back()
        return

    def update_dynamics(self):
        """
        This is the function that runs in a separate thread that calculates and
        updates the dynamics of the robot.
        i.e. the  current velocity and relative position of the robot and the
        current gyroscopic position of the robot. This is done using integration.
        :return: None
        """

        self.current_heading = self.data_fetcher.get_latest_heading()

        previous_imu_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        while self.do_update:
            self.current_heading = self.data_fetcher.get_latest_heading()

            # Get delta time by calculating the difference in the current time and the
            # last time we updated previous time.
            delta_time = get_delta_time(self.previous_time)

            # Get IMU data. It is of the form
            # [x_acc, y_acc, z_acc, xgyro, ygyro, zgyro]
            # The gyro is in units of mrad/s while acceleration is in
            # G which is 1 * 10 ^-2 m/s^2.
            imu_matrix = self.data_fetcher.get_imu_matrix()

            imu_matrix[2] = imu_matrix[2] - 981

            # have to divide acc by 100000 because it's in mG for some reason
            # using equation from physics: v(t) = v(0) + a(t)*t
            self.state_matrix[0] += (integrate(delta_time, (float(previous_imu_data[0] + imu_matrix[0])) / 2.0, 0.0)
                                     * sin(self.current_heading - self.initial_heading))
            self.state_matrix[1] += (integrate(delta_time, (float(previous_imu_data[1] + imu_matrix[1])) / 2.0, 0.0)
                                     * cos(self.current_heading - self.initial_heading))
            self.state_matrix[2] += (integrate(delta_time, (float(previous_imu_data[2] + imu_matrix[2])) / 20.0, 0.0))

            # Integrate velocity to get position.
            # using equation from physics: x(t) = x(0) + v(t)*t + 0.5 * a * t ^ 2
            self.state_matrix[3] += (integrate(delta_time, self.state_matrix[0],
                                               (float(previous_imu_data[0] + imu_matrix[0])) / 2.0))
            self.state_matrix[4] += (integrate(delta_time, self.state_matrix[1],
                                               (float(previous_imu_data[1] + imu_matrix[1])) / 2.0))
            self.state_matrix[5] += (integrate(delta_time, self.state_matrix[2],
                                               (float(previous_imu_data[2] + imu_matrix[2])) / 20.0))

            # Integrate gyro to get angle changes. The values from the IMU are in mrad/sec/
            self.state_matrix[6] += (integrate(delta_time,
                                               (float(previous_imu_data[3] + imu_matrix[3])) * 1.0/2000.0, 0.0))
            self.state_matrix[7] += (integrate(delta_time,
                                               (float(previous_imu_data[4] + imu_matrix[4])) * 1.0/2000.0, 0.0))
            self.state_matrix[8] += (integrate(delta_time,
                                               (float(previous_imu_data[5] + imu_matrix[5])) * 1.0/2000.0, 0.0))

            # Update previous time
            self.previous_time = time.time()

            # Store previous imu data for trapezoidal integration.
            previous_imu_data = imu_matrix
            time.sleep(0.01)
        return
     
    def heading_pid(self, speed, delta_dist):
        """
        The heading PID function moves the AUV forward while also ensuring the heading remains constant.
        :param speed: The speed to set the forward channel to.
        :param delta_dist: The desired forward displacement
        :return: None
        """
        # Set the base lateral speed and desired heading
        lateral_speed = 1500
        set_point = self.dest_heading

        # Set the forward speed
        set_rc_channel_pwm(self.motor_master, 5, speed)

        # Get starting x, y position
        (start_x_pos, start_y_pos) = (self.state_matrix[3], self.state_matrix[4])
        dist = calc_dist(start_x_pos, start_y_pos, self.state_matrix[3], self.state_matrix[4])

        while not dist >= delta_dist:

            # Calculate error in heading and account for it.
            error = angle_diff(set_point, self.current_heading)
            if error not in range(-3, 3):
                lateral_speed += self.heading_pid_iterator.iterate(error)

            if lateral_speed < 1450:
                lateral_speed = 1450
            elif lateral_speed > 1550:
                lateral_speed = 1550

            # Set the lateral speed to fix the heading
            set_rc_channel_pwm(self.motor_master, 4, lateral_speed)

            # Calculate displacement
            dist = calc_dist(start_x_pos, start_y_pos, self.state_matrix[3], self.state_matrix[4])

            # Set the forward speed
            set_rc_channel_pwm(self.motor_master, 5, speed)

            time.sleep(0.05)
        # Set the motors to neutral and reset the PID iterator.
        set_neutral(self.motor_master)
        self.heading_pid_iterator.reset()

        return

    def turn(self, delta_degrees):
        """
        This is the function for turning, i.e. changing the heading of the AUV.
        :param delta_degrees: The change in degrees from our current heading.
        :return: None
        """

        # Get the latest heading and calculate the target heading.
        heading = self.data_fetcher.get_latest_heading()

        target_heading = heading + delta_degrees
        if target_heading > 360:
            target_heading = target_heading - 360

        # Calculate the change in angle
        delta_angle = angle_diff(heading, target_heading)

        # Set the speed to a certain value and loop to set a certain speed based on the error.
        if delta_degrees > 0:
            while delta_angle not in range(-3, 4):
                set_rc_channel_pwm(self.motor_master, 4, 1575)
                delta_angle = angle_diff(self.data_fetcher.get_latest_heading(), target_heading)
        else:
            while delta_angle not in range(-2, 3):
                set_rc_channel_pwm(self.motor_master, 4, 1425)
                delta_angle = angle_diff(self.data_fetcher.get_latest_heading(), target_heading)

        print("Turn Complete")
        return

    def turn_north(self):
        """
        This function makes the AUV face towards North.
        :return: None
        """
        heading = self.data_fetcher.get_latest_heading()

        target_heading = 0
        delta_angle = angle_diff(target_heading, heading)

        self.turn(delta_angle)
        return

    def go_to(self, dest_lat, dest_lon):
        """
        This function moves the AUV to the given latitude, longitude pair. GPS must be enabled.
        :param dest_lat: The desired latitude coordinate to go to.
        :param dest_lon: The desired longitude coordinate to go to.
        :return: None
        """
        if not self.gps_enabled:
            print("GPS is not enabled. Cannot use GPS functionality.")
            return False

        # Get the current latitude, longitude pair
        curr_lat, curr_lon = self.data_fetcher.get_curr_gps()

        # Convert the latitude, longitude pair to radians.
        curr_lat = radians(curr_lat)
        curr_lon = radians(curr_lon)

        dest_lat = radians(dest_lat)
        dest_lat = radians(dest_lat)

        # Get the initial heading and calculate the bearing we need to be at.
        initial_heading = self.data_fetcher.get_latest_heading()
        bearing = bear_angle(curr_lon, curr_lat, dest_lon, dest_lat)

        # Change the angle to our desired one.
        delta_angle = bearing - initial_heading
        delta_angle = int(delta_angle)
        self.turn(delta_angle)

        # Calculate the distance to our desired latitude, longitude pair.
        dist = haversine(curr_lon, curr_lat, dest_lon, dest_lat) * 1000

        # Continue until we are within a radius of 1 meter from the desired coordinate.
        while dist not in range(-1, 1):
            # Get the current heading.
            curr_heading = self.data_fetcher.get_latest_heading()

            # Move forward
            set_rc_channel_pwm(self.motor_master, 6, 1650)

            # Reobtain current GPS coordinates and then calculate distance left to travel.
            curr_lat, curr_lon = self.data_fetcher.get_curr_gps()
            dist = haversine(curr_lon, curr_lat, dest_lon, dest_lat) * 1000

            # Adjust heading angle if needed.
            if initial_heading != curr_heading:
                delta_angle = bearing - curr_heading
                delta_angle = int(delta_angle)
                self.turn(delta_angle)
        return
