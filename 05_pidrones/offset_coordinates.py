import numpy as np
import nvector as nv
from math import radians, degrees


def pvec_b_to_lla(forward, right, down, roll, pitch, yaw, lat, lon, alt):
    """
    returns the lat lon and alt corresponding to a p-vec in the UAV's body frame

    Parameters
    ----------
    forward: float
        The number of meters forward of the UAV
    right: float
        The number of meters to the right of the UAV
    down: float
        The number of meters below the UAV
    roll: float
        The UAV's roll angle in degrees
    pitch: float
        The UAV's pitch angle in degrees
    yaw: float
        The UAV's yaw angle in degrees
    lat: float
        The UAV's latitude in degrees
    lon: float
        The UAV's longitude in degrees
    alt: float
        The UAV's altitude in meters

    Returns
    -------
    list
        This list holds three floats representing the latitude in degrees, longitude in degrees and altitude in meters (in that order).
    """

    # create a p-vector with the forward, right and down values
    p_B = np.array([forward, right, down])

    # this matrix can transform a pvec in the body frame to a pvec in the NED frame
    rot_NB = nv.zyx2R(radians(yaw), radians(pitch), radians(roll))

    # calculate the pvec in the NED frame
    p_N = rot_NB.dot(p_B)

    # create an n-vector for the UAV
    n_UAV = nv.lat_lon2n_E(radians(lat), radians(lon))

    # this creates a matrix that rotates pvecs from NED to ECEF
    rot_EN = nv.n_E2R_EN(n_UAV)

    # find the offset vector from the UAV to the point of interest in the ECEF frame
    p_delta_E = rot_EN.dot(p_N)

    # find the p-vector for the UAV in the ECEF frame
    p_EUAV_E = nv.n_EB_E2p_EB_E(n_UAV, -alt).reshape(1, 3)[0]

    # find the p-vector for the point of interest. This is the UAV + the offset in the ECEF frame.
    p_E = p_EUAV_E + p_delta_E

    # find the n-vector for the point of interest given the p-vector in the ECEF frame.
    n_result, z_result = nv.p_EB_E2n_EB_E(p_E.reshape(3, 1))

    # convert the n-vector to a lat and lon
    lat_result, lon_result = nv.n_E2lat_lon(n_result)
    lat_result, lon_result = degrees(lat_result), degrees(lon_result)

    # convert depth to alt
    alt_result = -z_result[0]

    return [lat_result, lon_result, alt_result]