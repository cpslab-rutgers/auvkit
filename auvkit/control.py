import time


class PID:
    """
    PID class is a class that implements a PID control loop.

    Methods:
        __init__(self, kP, kI, kD)
        iterate(self, error)
        reset(self)
    """

    def __init__(self, kP, kI, kD):
        """
        :param kP: The proportional constant for the PID.
        :param kI: The integration constant for the PID.
        :param kD: The derivative constant for the PID.
        """
        # Set the constant values.
        self.kP = kP
        self.kI = kI
        self.kD = kD

        # Initialize the integral to zero.
        self.integral_value = 0

        # Initialize the previous error and iteration time to zero.
        self.previous_error = 0
        self.previous_iteration_time = None

    def iterate(self, error):
        """
        This function iterates the PID loop
        :param error: The error between the the desired value and current value.
        :return: The calculated PID value using the proportional constant kP, the integration constant, kI
        and the derivation constant, kD.
        """

        # Handle the case where this is the first iteration of the PID loop.

        if not (self.previous_iteration_time is None):
            # If this is not the first iteration, calculate the elapsed time.
            elapsed_time = time.time() - self.previous_iteration_time
        else:
            # Else, set elapsed_time to zero.
            elapsed_time = 0

        # Calculate the integral value.
        self.integral_value = self.integral_value + (error * elapsed_time)

        # Calculate the derivative.
        derivative = (error - self.previous_error) / elapsed_time

        # Set the previous error and previous iteration time.
        self.previous_error = error
        self.previous_iteration_time = time.time()

        # Calculate the PID value using the defined constants and calculated values.
        return int(self.kP * error + self.kI * self.integral_value + self.kD * derivative)
    
    def reset(self):
        """
        This function resets the PID loop which entails setting the integral_value and previous_error
        to zero and setting the previous_iteration_time to None.
        :return: None
        """
        self.integral_value = 0
        self.previous_error = 0
        self.previous_iteration_time = None


class KalmanFilter:
    """
    KalmanFilter class is a class that implements a simple, one dimensional Kalman Filter.

    Methods:
        __init__(self)
        update(self, new_value)
        predict(self)
    """

    def __init__(self):
        # Initialize the measured (self.real_val) and estimated (self.estimated) values,
        # and the Kalman gain, all to zero.
        self.real_val = 0
        self.estimated = 0
        self.kalman_gain = 0

    def update(self, new_value):
        """
        This function updates the Kalman filter with a new measured value.
        :param new_value: The new measured value
        :return: None
        """
        # Update the real value with the new measured value
        self.real_val = new_value

        # Recalculate the Kalman gain.
        self.kalman_gain = self.estimated / (self.estimated + self.real_val)

        # Recalculate the estimated value using the new Kalman gain and measured value.
        self.estimated = self.estimated + self.kalman_gain * (self.real_val - self.estimated)
        return

    def predict(self):
        """
        This function returns the estimated value from the Kalman filter.
        :return: The estimated value from the Kalman filter.
        """
        # Return the estimated value
        return self.estimated
