import numpy as np
import Drone as dr

def drone_power_consumption(drone, speed_rel_to_air, rho=1.3):
    # Calculate power consumption 
    # power = 1/2 * rho * A * Cd * speed^3
    # drone: instance of class Drone
    # speed_rel_to_air: float value (m.s-1)
    # rho: air density (kg/m3)
    power = 1/2 * rho * drone.acd * speed_rel_to_air ** 3
    return power

def cost_a(point_a, point_b, drone, wind):
    # Cost calculated by constant speed relative to the GROUND
    assert drone.speed > 0
    vector_displacement  = np.array((point_b.x - point_a.x, point_b.y - point_a.y ))
    distance  = np.linalg.norm(vector_displacement)
    if distance>0:
        vector_speed_drone_sol = drone.speed * vector_displacement/distance
        vector_speed_drone_air = vector_speed_drone_sol  - wind.vector
        norm_speed_relative  = np.linalg.norm(vector_speed_drone_air)
        cost = drone_power_consumption(drone, norm_speed_relative, rho=1.3) * distance / drone.speed
        return cost
    else:
        return 0


def cost_b(point_a, point_b, drone, wind, safety_factor=2):
    """
    Returns the cost (expressed in joules) from point a to point b for a given drone and a given wind.
    In this formula, the drone is considered to be moving at constant speed relative to the AIR

    safety_factor: float. must be strictly greater than 1. The drone must go safety_factor times faster than the
    wind.
    """

    # pre.place_holder(point_a, point_b, drone, wind)
    assert safety_factor > 1 

    assert drone.speed > safety_factor*wind.speed

    vector_deplacement = np.array((point_b.x - point_a.x, point_b.y - point_a.y))
    distance = np.linalg.norm(vector_deplacement)
    if distance > 0:
        vector_deplacement_norm = vector_deplacement / distance
        e = (wind.x * vector_deplacement_norm[0] + wind.y * vector_deplacement_norm[1])
        f = wind.speed**2 - drone.speed**2
        delta = 4*e**2 - 4*f
        v3 = e + (0.5 * np.sqrt(delta))
        
        return drone_power_consumption(drone, drone.speed, rho=1.3) * distance / v3
    else:
        return 0

def check_route_compatibility(route_a, route_b):
    """Checks if two routes don't have inherent incompatibilities. """
    # Right now we'll consider that two routes are compatible is they have the same depot. It might change in the
    # future with new developments.
    
    if route_a.depot == route_b.depot:
        return True
    else:
        return False


def check_delivery_compatibility(delivery_a, delivery_b):
    """Checks if two deliveries don't have inherent incompatibilities. """
    # We'll consider that two deliveries are compatible if their routes are compatibles and it they have the same drone
    # and the same wind.
    # pre.place_holder(delivery_a, delivery_b)
    if delivery_a.drone == delivery_b.drone and delivery_a.wind == delivery_b.wind and \
            check_route_compatibility(delivery_a.route, delivery_b.route):
        return True
    else:
        return False

