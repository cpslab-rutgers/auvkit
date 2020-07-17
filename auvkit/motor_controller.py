def set_rc_channel_pwm(master, channel, pwm=1500):
    """
    This function sets a given motor channel to a given PWM speed.
    :param master: The connection master for MAVLink.
    :param channel: The Pixhawk channel to be set.
    :param pwm: The PWM speed to be set.
    :return: None
    """
    if channel < 1:
        print("Channel does not exist.")
        return
    if channel < 9:
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[channel - 1] = pwm
        master.mav.rc_channels_override_send(
            master.target_system,  # target_system
            master.target_component,  # target_component
            *rc_channel_values)  # RC channel list, in microseconds.


def set_neutral(master):
    """
    This function sets all motor channels to neutral. It is useful for
    many scenarios, such as when we want to provide a failsafe for the AUV to
    stop motors in case of an emergency.
    :param master: The connection master for MAVLink.
    :return: None
    """
    set_rc_channel_pwm(master, 1, 1500)
    set_rc_channel_pwm(master, 2, 1500)
    set_rc_channel_pwm(master, 3, 1500)
    set_rc_channel_pwm(master, 4, 1500)
    set_rc_channel_pwm(master, 5, 1500)
    set_rc_channel_pwm(master, 6, 1500)
    set_rc_channel_pwm(master, 7, 1500)
    set_rc_channel_pwm(master, 8, 1500)
    set_rc_channel_pwm(master, 9, 1500)
    return
