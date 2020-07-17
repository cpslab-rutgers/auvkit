from brping import Ping1D


class Ping:
    """
    Ping class is a wrapper around the BlueRobotics Ping1D python library.
    It adds extra features that reduce complexity on the developer so the developer
    will not have to handle low confidence values.

    Methods:
        __init__(self, device_port, baudrate=115200, confidence_threshold=90)
        set_ping_interval(self, ping_interval)
        set_speed_of_sound(self, speed_of_sound)
        set_gain_index(self, gain_index)
        set_confidence_threshold(self, confidence_threshold)
        get_distance(self)
    """

    def __init__(self, device_port, baudrate=115200, confidence_threshold=90):
        """

        :param device_port: The serial port that the Ping echosonar is connected to.
        :param baudrate: The baudrate of the serial port that we are communicating over. Default is set to 115200.
        :param confidence_threshold: The desired confidence threshold, i.e. the minimum confidence necessary
        for us to use the given value by the Ping echosonar. Default is set to 90%.
        """

        # If given confidence threshold is too high, default to 100.
        if confidence_threshold > 100:
            confidence_threshold = 100

        # If given confidence threshold is too low, default to 0.
        if confidence_threshold < 0:
            confidence_threshold = 0

        self.confidence_threshold = confidence_threshold

        self.ping = Ping1D(device_port, baudrate)

        # Raise an exception if the Ping cannot initialize.
        if self.ping.initialize() is False:
            raise Exception("Ping device failed to initialize. Verify device port is accurate\
                and baudrate is a valid value.")

    def set_ping_interval(self, ping_interval):
        """
        A function to set the desired ping interval.
        :param ping_interval: The desired ping interval.
        :return: True
        """
        self.ping.set_ping_interval(ping_interval)
        return True

    def set_speed_of_sound(self, speed_of_sound):
        """
        A function to set the speed of sound of the Ping sonar.
        :param speed_of_sound: The desired speed of sound.
        :return: True
        """
        self.ping.set_speed_of_sound(speed_of_sound)
        return True

    def set_gain_index(self, gain_index):
        """
        A function to set the gain index of the Ping sonar.
        :param gain_index: The desired gain index.
        :return: True
        """
        self.ping.set_gain_index(gain_index)
        return True

    def set_confidence_threshold(self, confidence_threshold):
        """
        A function to set the confidence threshold of the Ping sonar. The confidence threshold is the
        threshold of a confidence value below which we will disregard any values.
        :param confidence_threshold: The desired confidence threshold.
        :return: True
        """
        self.confidence_threshold = confidence_threshold
        return True

    def get_distance(self):
        """
        A function to get the distance recorded by the Ping echosonar. The function only
        returns a distance only if the confidence threshold is greater than or equal to the
        set confidence threshold.
        :return: The distance if the confidence threshold is sufficient, None otherwise
        """

        # Limit distance retrieval to 100 tries. If any of the attempts give a confidence value above our threshold,
        # then return the distance. Otherwise, continue. If 100 tries are completed without a confidence value
        # above the threshold, then return None.
        max_iterations = 100
        while max_iterations > 0:
            dist = self.ping.get_distance()
            if dist["confidence"] >= self.confidence_threshold:
                return dist["distance"]
            max_iterations -= 1
        return None
