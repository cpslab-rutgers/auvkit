from pymavlink import mavutil


def check_mode_set(master, mode):
    """
    This function checks whether or not a mode is set.
    :param master: The MAVLink connection master.
    :param mode: The mode we want to see is set or not.
    :return: False if the mode is not set and True otherwise.
    """
    if mode not in master.mode_mapping():
        return False
    return True


def set_mode(mode):
    """
    This function sets a MAVLink mode.
    :param mode: The mode we want to set.
    :return: True
    """
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    master.wait_heartbeat()

    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    print("Mode " + mode + " successfully set.")
    return True


def verify_mode_set(master):
    """
    This function verifies that a mode has been set.
    :param master: The MAVLink connection master.
    :return: True
    """
    while True:
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()

        if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
            continue
        return True
