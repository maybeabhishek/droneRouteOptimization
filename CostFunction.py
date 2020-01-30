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


def merge_routes(route_a, route_b, must_have_common_client=False):
    """Merges route_a and route_b into a new route where the list of clients is:
    -first, the list of clients of route_a
    -then, the list of clients of route_b
    For the new route to be legal:
    - route_a and route_b must be compatible
    - the list of clients of the new route must be legal
    One subtlety:
    - the last client of route_a and the first client of route_b can be identical but it is not mandatory unless
    must_have_common_client is true. Should route_a and route_b have this common client, the list of clients of the
    merged route would, of course, not have this common client duplicated.
    Returns the new route if legal. Returns None otherwise."""
    
    if not check_route_compatibility(route_a, route_b):
        return None
    if check_route_compatibility(route_a, route_b):
        list_des_clients = []
        if must_have_common_client:
            if len(route_a.clients_list) == 0 or len(route_b.clients_list) == 0:
                return None
            if not route_a.clients_list[-1] is route_b.clients_list[0]:
                return None
            for i in range(0, len(route_a.clients_list)):
                list_des_clients.append(route_a.clients_list[i])
            for k in range(1, len(route_b.clients_list)):
                list_des_clients.append(route_b.clients_list[k])
        if not must_have_common_client:
            if len(route_a.clients_list) != 0 and len(route_b.clients_list) != 0:
                if route_a.clients_list[-1] is route_b.clients_list[0]:
                    for i in range(0, len(route_a.clients_list)):
                        list_des_clients.append(route_a.clients_list[i])
                    for k in range(1, len(route_b.clients_list)):
                        list_des_clients.append(route_b.clients_list[k])
            if not list_des_clients:
                for client in route_a.clients_list:
                    list_des_clients.append(client)
                for client2 in route_b.clients_list:
                    list_des_clients.append(client2)
        route_c = pre.Route(list_des_clients, route_a.depot)
        if route_c.is_legal:
            return route_c
        if not route_c.is_legal:
            return None


def merge_deliveries(delivery_a, delivery_b, must_have_common_client=False):
    """Merges delivery_a and delivery_b into a new delivery where the route is generated using merge_routes rules.
    delivery_a and delivery_b must be compatible.
    Returns the new delivery if legal. Returns None otherwise."""
    # pre.place_holder(delivery_a, delivery_b, must_have_common_client)
    if not check_delivery_compatibility(delivery_a, delivery_b):
        return None
    if check_delivery_compatibility(delivery_a, delivery_b):
        new_route = merge_routes(delivery_a.route, delivery_b.route, must_have_common_client)
        if new_route:
            new_delivery = pre.Delivery(new_route, delivery_a.parameters)
            if new_delivery.is_legal:
                return new_delivery
        else:
            return None


def cost_matrix(problem, parameters):
    """This function returns the cost matrix of a problem for a given parameter set.
    :param problem: Instance of class Problem
    :param parameters: Instance of class DeliveryParameters
    :return: 2-dimensional numpy array representing the cost matrix"""
    # pre.place_holder(problem, parameters)
    mat_dim = problem.number_of_clients + 1
    c_matrix = np.zeros((mat_dim, mat_dim))  # creates a square matrix (2-dimensional numpy array) filled with zeros
    if parameters.cost_fct:
        for i in range(0, len(problem.clients_list)):
            c_matrix[0][i + 1] = parameters.cost_fct(problem.depot, problem.clients_list[i],
                                                     parameters.drone, parameters.wind)
            c_matrix[i + 1][0] = parameters.cost_fct(problem.clients_list[i], problem.depot,
                                                     parameters.drone, parameters.wind)
            for k in range(0, len(problem.clients_list)):
                c_matrix[i+1][k+1] = parameters.cost_fct(problem.clients_list[i], problem.clients_list[k],
                                                         parameters.drone, parameters.wind)
    else:
        for i in range(0, len(problem.clients_list)):
            for k in range(i+1, len(problem.clients_list)+1):
                c_matrix[i][k] = c_matrix[k][i] = None
    return c_matrix


def savings_matrix(problem, parameters):
    """This function returns the savings matrix of a problem for a given parameter set.
    :param problem. Instance of class Problem
    :param parameters. Instance of class DeliveryParameters
    :return 2-dimensional numpy array representing the savings matrix"""
    # pre.place_holder(problem, parameters)
    mat_dim = problem.number_of_clients
    c_matrix = cost_matrix(problem, parameters)
    s_matrix = np.zeros((mat_dim, mat_dim))  # creates a square matrix (2-dimensional numpy array) filled with zeros
    if parameters.cost_fct:
        for i in range(0, len(problem.clients_list)):
            for k in range(0, len(problem.clients_list)):
                if k != i:
                    a = parameters.cost_fct(problem.clients_list[i], problem.depot, parameters.drone, parameters.wind)
                    b = parameters.cost_fct(problem.depot, problem.clients_list[k], parameters.drone, parameters.wind)
                    s_matrix[i][k] = a + b - c_matrix[i+1][k+1]
    return s_matrix