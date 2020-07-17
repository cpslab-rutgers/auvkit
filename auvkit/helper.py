from math import sin, cos, atan2, degrees, sqrt, asin
import time


def integrate(dt, dy, d2y=0):
    """
    This function integrates a mathematical function, not necessarily explicitly defined, given the
    numerical values for change in time, rate of change and optionally, second derivative.
    :param dt: The change in time
    :param dy: The rate of change of the function (i.e. first derivative).
    :param d2y: The rate of change of the rate of change of the function (i.e. second derivative). Default is 0.
    :return: The integrated value of the function using the change in time, and the first and second derivative.
    """
    return dt * dy + 0.5 * d2y * (dt ** 2)


def get_delta_time(previous_time):
    """
    This function calculates the difference between current time
    and a given previous time and returns the difference in seconds.
    :param previous_time: The previous time from which we want to calculate the difference in time.
    :return: The difference in time is seconds.
    """
    delta_time = time.time() - previous_time
    return delta_time


def calc_dist(x1, y1, x2, y2):
    """
    This function calculates the Euclidean distance between two points.
    :param x1: The x coordinate of the first point
    :param y1: The y coordinate of the first point
    :param x2: The x coordinate of the second point.
    :param y2: The y coordinate of the second point.
    :return: The Euclidean distance between the two given points.
    """
    return abs(sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))


def angle_diff(set_point, current_heading):
    """
    This function calculates the angle difference between two compass heading values.
    :param set_point: The desired heading we want to set to.
    :param current_heading: The current heading we are at.
    :return: The difference in heading between the set point and the current heading.
    """
    delta_angle = set_point - current_heading
    delta_angle = (delta_angle + 180) % 360 - 180
    return delta_angle


def error_attitude(set_point, current_att):
    """
    This function calculates the error in the attitude of the AUV.
    :param set_point: The set point of the attitude.
    :param current_att: The current attitude.
    :return: The difference between the set point and current attitude.
    """
    error = set_point - current_att
    return error


def haversine(lon1, lat1, lon2, lat2):
    """
    This function implements Haversine's formula which is used to calculate the distance between two
    points on a sphere (e.g. Earth). It takes two latitude, longitude pairs and calculates the distance between the two
    points in kilometers.
    :param lon1: The longitude of the first point.
    :param lat1: The latitude of the first point.
    :param lon2: The longitude of the second point.
    :param lat2: The latitude of the second point.
    :return: The distance between the two given points in kilometers.
    """
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    r = 6371
    return c * r


def bear_angle(lon1, lat1, lon2, lat2):
    """
    This function calculates the bearing between two latitude, longitude pairs. The bearing is calculated as
    the bearing from point 1 (lat1, lon1) to point 2 (lat2, lon2).
    :param lon1: The longitude of the first point.
    :param lat1: The latitude of the first point.
    :param lon2: The longitude of the second point.
    :param lat2: The latitude of the second point.
    :return: The bearing between the two given latitude, longitude pairs.
    """
    bearing = atan2(cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1), sin(lon2-lon1)*cos(lat2))
    bearing = 90-degrees(bearing)
    return bearing
