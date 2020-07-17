from pymavlink import mavutil
import serial
import time


# See MAVLINK messages documentations:
# https://mavlink.io/en/messages/common.html

# For how to read from Atlas Scientific sensors:
# https://github.com/AtlasScientific/Raspberry-Pi-sample-code

class AtlasScientificSensor:
    """
    AtlasScientificSensor class is a class that describes an Atlas Scientific sensor connected serially.

    Methods:
        __init__(self)
        read_line(self, ser)
        read_lines(self, ser)
        send_cmd(self, ser, cmd)
        read_sensor(self, ser_reader)
    """

    def __init__(self, identifier, serial_port):
        """
        :param identifier: A unique identifier that describes the sensor.
        :param serial_port: The serial port the sensor is connect to.
        """
        self.identifier = identifier
        self.serial_port = serial_port
        self.serial_communicator = serial.Serial(serial_port, 9600, timeout=0)

    def read_line(self, ser):
        """
        This function reads a line from the Atlas scientific sensor.
        :param ser: A Python Serial object.
        :return: The line buffer.
        """
        lsl = len('\r')
        line_buffer = []
        while True:
            next_char = ser.read(1)
            if next_char == '':
                break
            line_buffer.append(next_char)
            if (len(line_buffer) >= lsl and
                    line_buffer[-lsl:] == list('\r')):
                break
        return ''.join(line_buffer)

    def read_lines(self, ser):
        """
        This function reads lines from the Atlas scientific sensor.
        :param ser: A Python Serial object.
        :return: The lines or None if an exception occurs.
        """
        lines = []
        try:
            while True:
                line = read_line(ser)
                if not line:
                    break
                    ser.flush_input()
                lines.append(line)
            return lines

        except serial.SerialException as e:
            print("Error, ", e)
            return None

    def send_cmd(self, ser, cmd):
        """
        Send command to the Atlas Sensor.
        Before sending, add Carriage Return at the end of the command.
        :param ser: A Python Serial object.
        :param cmd: The command to send to the sensor
        :return: True if success, None if a SerialException occurs
        """
        buf = cmd + "\r"  # add carriage return
        try:
            ser.write(buf.encode('utf-8'))
            return True
        except serial.SerialException as e:
            print("Error, ", e)
            return None

    def read_sensor(self):
        """
        This function reads data from an Atlas scientific sensor
        :return: The lines read from the sensor
        """

        # Turn off continuous mode
        self.send_cmd(self.serial_communicator, "C,0")

        # Clear all previous data
        time.sleep(1)
        self.serial_communicator.flush()

        self.send_cmd(self.serial_communicator, "R")
        lines = self.read_lines(self.serial_communicator)
        for i in range(len(lines)):
            if lines[i][0] != '*':
                return lines[i]

        return lines


class DataFetcher:
    """
    DataFetcher class is a class that allows us to fetch data from the Pixhawk
    without overloading the communication by allowing a set refresh rate and employing
    caching. DataFetcher also eases data retrieval and makes it easy to write data to a CSV file.
    In the future, database implementations can be added.

    Methods:
        __init__(self)
        update_gps_refresh_rate(self, refresh_rate)
        update_heading_refresh_rate(self, refresh_rate)
        update_attitude_refresh_rate(self, refresh_rate)
        update_imu_refresh_rate(self, refresh_rate)
        update_scaled_pres_refresh_rate(self, refresh_rate)
        update_battery_refresh_rate(self, refresh_rate)
        update_gps(self)
        update_heading(self)
        update_attitude(self)
        update_imu(self)
        update_scaled_pres(self)
        update_battery(self)
        get_curr_gps(self)
        get_num_gps_satellites(self)
        get_latest_heading(self)
        get_latest_att(self)
        get_scaled_pres(self)
        get_temp(self)
        get_imu_data(self)
        get_xacc(self)
        get_yacc(self)
        get_zacc(self)
        get_xgyro(self)
        get_ygyro(self)
        get_zgyro(self)
        get_xmag(self)
        get_ymag(self)
        get_zmag(self)
        get_imu_matrix(self)
        get_attitude(self)
        get_roll(self)
        get_pitch(self)
        get_yaw(self)
        get_rollspeed(self)
        get_pitchspeed(self)
        get_yawspeed(self)
        get_angular_kinematics_matrix(self)
        get_battery_data(self)
        get_temperature(self)
        get_voltages(self)
        get_current_battery(self)
        get_current_consumed(self)
        get_energy_consumed(self)
        get_battery_remaining(self)
        get_time_remaining(self)
        get_charge_state(self)get_battery_matrix(self)
        get_header(self)
        __iter__(self)
        __next__(self)
    """

    def __init__(self, atlas_sensor_list):
        """
        :param atlas_sensor_list: A list of AtlasScientificSensor objects.
        """

        self.atlas_sensor_list = atlas_sensor_list

        self.ardu_master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        self.ardu_master.wait_heartbeat()

        self.index = 0
        self.mavlink_is_done = False
        self.serial_is_done = False

        # Define refresh rates and previous times where we got the data.
        self.gps_refresh_rate = 0.25
        self.last_gps_time = None

        self.heading_refresh_rate = 0.25
        self.last_heading_time = None

        self.attitude_refresh_rate = 0.25
        self.last_attitude_time = None

        self.imu_refresh_rate = 0.25
        self.last_imu_time = None

        self.scaled_pres_refresh_rate = 0.25
        self.last_scaled_pres_time = None

        self.battery_refresh_rate = 0.25
        self.last_battery_time = None

        # Messages
        self.gps_msg = None
        self.heading_msg = None
        self.attitude_msg = None
        self.imu_msg = None
        self.scaled_pres_msg = None
        self.battery_msg = None

    def update_gps_refresh_rate(self, refresh_rate):
        """
        This function updates the refresh rate for the GPS retrieval.
        :param refresh_rate: The defined refresh rate for the GPS retrieval.
        :return: None
        """
        self.gps_refresh_rate = refresh_rate

    def update_heading_refresh_rate(self, refresh_rate):
        """
        This function updates the refresh rate for the heading retrieval.
        :param refresh_rate: The defined refresh rate for the heading retrieval.
        :return: None
        """
        self.heading_refresh_rate = refresh_rate

    def update_attitude_refresh_rate(self, refresh_rate):
        """
        This function updates the refresh rate for the attitude retrieval.
        :param refresh_rate: The defined refresh rate for the attitude retrieval.
        :return: None
        """
        self.attitude_refresh_rate = refresh_rate

    def update_imu_refresh_rate(self, refresh_rate):
        """
        This function updates the refresh rate for the IMU data retrieval.
        :param refresh_rate: The defined refresh rate for the IMU data retrieval.
        :return: None
        """
        self.imu_refresh_rate = refresh_rate

    def update_scaled_pres_refresh_rate(self, refresh_rate):
        """
        This function updates the refresh rate for the SCALED_PRESSURE3 message retrieval.
        :param refresh_rate: The defined refresh rate for the SCALED_PRESSURE3 message retrieval.
        :return: None
        """
        self.scaled_pres_refresh_rate = refresh_rate

    def update_battery_refresh_rate(self, refresh_rate):
        """
        This function updates the refresh rate for the battery data retrieval.
        :param refresh_rate: The defined refresh rate for the battery data retrieval.
        :return: None
        """
        self.battery_refresh_rate = refresh_rate

    def update_gps(self):
        """
        This function updates the cached GPS message if necessary.
        :return: None
        """
        if self.last_gps_time is None or (time.time() - self.last_gps_time) >= self.gps_refresh_rate:
            self.gps_msg = self.ardu_master.recv_match(type='GPS_RAW_INT', blocking=True)

    def update_heading(self):
        """
        This function updates the cached heading message if necessary.
        :return: None
        """
        if self.last_heading_time is None or (time.time() - self.last_heading_time) >= self.heading_refresh_rate:
            self.heading_msg = self.ardu_master.recv_match(type='VFR_HUD', blocking=True)

    def update_attitude(self):
        """
        This function updates the cached attitude message if necessary.
        :return: None
        """
        if self.last_attitude_time is None or (time.time() - self.last_attitude_time) >= self.attitude_refresh_rate:
            self.attitude_msg = self.ardu_master.recv_match(type='ATTITUDE', blocking=True)

    def update_imu(self):
        """
        This function updates the cached IMU message if necessary.
        :return: None
        """
        if self.last_imu_time is None or (time.time() - self.last_imu_time) >= self.imu_refresh_rate:
            self.imu_msg = self.ardu_master.recv_match(type='SCALED_IMU2', blocking=True)

    def update_scaled_pres(self):
        """
        This function updates the cached SCALED_PRESSURE3 message if necessary.
        :return: None
        """
        if self.last_scaled_pres_time is None or (time.time() - self.last_scaled_pres_time) \
                >= self.scaled_pres_refresh_rate:
            self.scaled_pres_msg = self.ardu_master.recv_match(type='SCALED_PRESSURE3', blocking=True)

    def update_battery(self):
        """
        This function updates the cached battery message if necessary.
        :return: None
        """
        if self.last_battery_time is None or (time.time() - self.last_battery_time) >= self.battery_refresh_rate:
            self.battery_msg = self.ardu_master.recv_match(type='BATTERY_STATUS', blocking=True)

    def get_curr_gps(self):
        """
        This function retrieves the current latitude and longitude. If coordinates
        come out to (0,0), then the GPS reciever cannot connect to GPS satellites.
        :return: The current latitude and longitude as a tuple.
        """
        self.update_gps()
        return self.gps_msg.lat / 10000000.0, self.gps_msg.lon / 10000000.0

    def get_num_gps_satellites(self):
        """
        This function retrieves the current number of connect GPS satellites.
        :return: The current number of connect GPS satellites.
        """
        self.update_gps()
        return self.gps_msg.satellites_visible

    def get_latest_heading(self):
        """
        This function retrieves the current compass heading.
        :return: The current compass heading.
        """
        self.update_heading()
        return self.heading_msg.heading

    def get_latest_att(self):
        """
        This function returns the current yaw of the AUV.
        :return: The current yaw of the AUV.
        """
        self.update_attitude()
        return self.attitude_msg.yaw

    def get_scaled_pres(self):
        """
        This function returns the boot time in ms,
        current absolute pressure, differential pressure and temperature measured by the AUV.
        :param master: The MAVLink connection master.
        :return: The boot time in ms, current absolute pressure in hPa,
        differential pressure in hPa and temperature measured in cdegC.
        """
        self.update_scaled_pres()
        return [self.scaled_pres_msg.time_boot_ms, self.scaled_pres_msg.press_abs,
                self.scaled_pres_msg.press_diff, self.scaled_pres_msg.temperature]

    def get_temp(self):
        """
        This function returns the current temperature measured by the AUV.
        :return: The current measured temperature in cdegC.
        """
        self.update_scaled_pres()
        return self.scaled_pres_msg.temperature

    def get_imu_data(self):
        """
        This function returns the data for the SCALED_IMU message.
        :return: The IMU data from the SCALED_IMU message.
        """
        self.update_imu()
        return self.imu_msg

    def get_xacc(self):
        """
        This function returns the X-component of the acceleration.
        :return: the X-component of the acceleration.
        """
        self.update_imu()
        return self.imu_msg.xacc

    def get_yacc(self):
        """
        This function returns the Y-component of the acceleration.
        :return: the Y-component of the acceleration.
        """
        self.update_imu()
        return self.imu_msg.yacc

    def get_zacc(self):
        """
        This function returns the Z-component of the acceleration.
        :return: the Z-component of the acceleration.
        """
        self.update_imu()
        return self.imu_msg.zacc

    def get_xgyro(self):
        """
        This function returns the Y-component of the gyroscopic speed.
        :return: the X-component of the gyroscopic speed.
        """
        self.update_imu()
        return self.imu_msg.xgyro

    def get_ygyro(self):
        """
        This function returns the Z-component of the gyroscopic speed.
        :return: the Z-component of the gyroscopic speed.
        """
        self.update_imu()
        return self.imu_msg.ygyro

    def get_zgyro(self):
        """
        This function returns the Z-component of the gyroscopic speed.
        :return: the Z-component of the gyroscopic speed.
        """
        self.update_imu()
        return self.imu_msg.zgyro

    def get_xmag(self):
        """
        This function returns the X-component of the magnetic force.
        :return: the X-component of the magnetic force.
        """
        self.update_imu()
        return self.imu_msg.xmag

    def get_ymag(self):
        """
        This function returns the Y-component of the magnetic force.
        :return: the Y-component of the magnetic force
        """
        self.update_imu()
        return self.imu_msg.ymag

    def get_zmag(self):
        """
        This function returns the Z-component of the magnetic force.
        :return: the Z-component of the magnetic force.
        """
        self.update_imu()
        return self.imu_msg.zmag

    def get_imu_matrix(self):
        """
        Returns a list of the x, y and z components of acceleration and gyroscopic speed.
        :return: A list of the x, y and z components of acceleration and gyroscopic speed.
        """
        self.update_imu()
        return [self.imu_msg.xacc, self.imu_msg.yacc, self.imu_msg.zacc,
                self.imu_msg.xgyro, self.imu_msg.ygyro, self.imu_msg.zgyro]

    def get_attitude(self):
        """
        Updates and returns the attitude message.
        :return: The attitude of the AUV.
        """
        self.update_attitude()
        return self.attitude_msg

    def get_roll(self):
        """
        Update thee attitude if needed and return the roll.
        :return: The roll of the AUV.
        """
        self.update_attitude()
        return self.attitude_msg.roll

    def get_pitch(self):
        """
        Update thee attitude if needed and return the pitch.
        :return: The pitch of the AUV.
        """
        self.update_attitude()
        return self.attitude_msg.pitch

    def get_yaw(self):
        """
        Update thee attitude if needed and return the yaw.
        :return: The yaw of the AUV.
        """
        self.update_attitude()
        return self.attitude_msg.yaw

    def get_rollspeed(self):
        """
        Update thee attitude if needed and return the rollspeed.
        :return: The rollspeed of the AUV.
        """
        self.update_attitude()
        return self.attitude_msg.rollspeed

    def get_pitchspeed(self):
        """
        Update thee attitude if needed and return the pitchspeed.
        :return: The pitchspeed of the AUV.
        """
        self.update_attitude()
        return self.attitude_msg.pitchspeed

    def get_yawspeed(self):
        """
        Update the attitude if needed and return the yawspeed.
        :return: The yawspeed of the AUV.
        """
        self.update_attitude()
        return self.attitude_msg.yawspeed

    def get_angular_kinematics_matrix(self):
        """
        Updates the attitude and returns a list of the roll, pitch, yaw, rollspeed, pitchspeed and yawspeed
        :return: A list of the roll, pitch, yaw, rollspeed, pitchspeed and yawspeed.
        """
        self.update_attitude()
        return [self.attitude_msg.roll, self.attitude_msg.pitch, self.attitude_msg.yaw,
                self.attitude_msg.rollspeed, self.attitude_msg.pitchspeed, self.attitude_msg.yawspeed]

    def get_battery_data(self):
        """
        Update the battery message if needed and return the battery message
        :return: The battery message and corresponding data
        """
        self.update_battery()
        return self.battery_msg

    def get_temperature(self):
        """
        Update the battery message if needed and return the battery temperature.
        :return: The battery temperature.
        """
        self.update_battery()
        return self.battery_msg.temperature

    def get_voltages(self):
        """
        Update the battery message if needed and return the battery voltages.
        :return: The battery voltages.
        """
        self.update_battery()
        return self.battery_msg.voltages

    def get_current_battery(self):
        """
        Update the battery message if needed and return the battery current.
        :return: The battery current.
        """
        self.update_battery()
        return self.battery_msg.current_battery

    def get_current_consumed(self):
        """
        Update the battery message if needed and return the current consumed.
        :return: The current consumed.
        """
        self.update_battery()
        return self.battery_msg.current_consumed

    def get_energy_consumed(self):
        """
        Update the battery message if needed and return the energy consumed.
        :return: The energy consumed.
        """
        self.update_battery()
        return self.battery_msg.energy_consumed

    def get_battery_remaining(self):
        """
        Update the battery message if needed and return the battery remaining.
        :return: The battery remaining.
        """
        self.update_battery()
        return self.battery_msg.battery_remaining

    def get_time_remaining(self):
        """
        Update the battery message if needed and return the time remaining on battery.
        :return: The time remaining on battery.
        """
        self.update_battery()
        return self.battery_msg.time_remaining

    def get_charge_state(self):
        """
        Update the battery message if needed and return the charge state.
        :return: The charge state.
        """
        self.update_battery()
        return self.battery_msg.charge_state

    def get_battery_matrix(self):
        """
        Update the battery message if needed and return a list of battery temperature, voltages, battery current,
        current consumed, energy consumed and the battery remaining.
        :return: A list of battery temperature, voltages, battery current,
        current consumed, energy consumed and the battery remaining.
        """
        self.update_battery()
        return [self.battery_msg.temperature, self.battery_msg.voltages,
                self.battery_msg.current_battery, self.battery_msg.current_consumed,
                self.battery_msg.energy_consumed, self.battery_msg.battery_remaining]

    def get_header(self):
        """
        This function returns a list of the columns for a CSV.
        :return: A list of the columns for the CSV.
        """
        header_list = ["time", "latitude", "longitude", "heading", "xacc", "yacc", "zacc", "xgyro",
                       "ygyro", "zgyro", "roll", "pitch", "yaw", "rollspeed", "pitchspeed",
                       "yawspeed", "battery_temperature", "voltages", "current_battery",
                       "current_consumed", "energy_consumed", "battery_remaining", "time_boot_ms",
                       "press_abs", "press_diff", "temperature"]

        for sensor in self.atlas_sensor_list:
            header_list.append(sensor.identifier)

        return header_list

    def __iter__(self):
        self.index = 0
        self.mavlink_is_done = False
        self.serial_is_done = False
        return self

    def __next__(self):
        if self.mavlink_is_done:
            if self.index < len(self.atlas_sensor_list):
                self.return_val = self.atlas_sensor_list[self.index].read_sensor()
                self.index += 1
            else:
                raise StopIteration
        else:
            if self.index < 6:
                if self.index == 0:
                    self.return_val = self.get_curr_gps()
                elif self.index == 1:
                    self.return_val = self.get_latest_heading()
                elif self.index == 2:
                    self.return_val = self.get_imu_matrix()
                elif self.index == 3:
                    self.return_val = self.get_angular_kinematics_matrix()
                elif self.index == 4:
                    self.return_val = self.get_battery_matrix()
                else:
                    self.return_val = self.get_scaled_pres()

                self.index += 1
            else:
                self.mavlink_is_done = True
                self.index = 0
                self.return_val = self.atlas_sensor_list[self.index].read_sensor()
        return self.return_val
