import numpy as np


class Drone:

    def __init__(self,capacity=100,speed=10,acd=0.01):
        self.capacaity = capacity   # Capacity I am considering to be relative so no units
        self.speed = speed   # Speed in m/s
        self.acd = acd      #  ==a*cd  Area * coeff of drag   apparently important property

    def __repr__(self):
        return "<Drone at: {}, capacity: {}, speed: {}, acd: {}>.format"
        
    def copy(self,another_drone):
        self.__init__(another_drone.capacity, another_drone.speed, another_drone.acd)
    
class Wind:

    def __init__(self, x=0., y=0.):  
        self.x = x  # x>0 means the wind is blowing from west to east
        self.y = y  # y>0 means the wind is blowing from south to north

    def __repr__(self):
        return "<Wind at {}. (x, y) = ({}, {}) ; speed = {} m.s-1>".format(hex(id(self)), self.x, self.y, self.speed)

    @property
    def vector(self):
        return np.array((self.x,self.y))
    

    @vector.setter 
    def vector(self, xy):  
        """Allows to set both self.x and self.y in one shot."""
        self.x = xy[0]
        # to be continued ...
        self.y = xy[1]

    @property
    def speed(self):
        """Returns the speed of the wind (m.s-1)."""
        return np.linalg.norm(self.vector)

    @speed.setter
    def speed(self, target_speed):
        if self.speed > 0:
            self.vector = (target_speed / self.speed) * self.vector
        else:
            self.x = target_speed

class Point:
     """This class represents a point on the map. 'x' and 'y' are the coordinates of the point."""

    def __init__(self, identifier="", x=0., y=0.):
        self.identifier = identifier  
        self.x = x  
        self.y = y  

    def __repr__(self):
        return "<Point at {}. id = {}, (x,y) = ({}, {})>".format(hex(id(self)), self.identifier, self.x, self.y)

    def __str__(self):
        return self.identifier

    def copy(self, other_point):
        self.__init__(other_point.identifier, other_point.x, other_point.y)


class Client(Point):  

    def __init__(self, identifier="Client", x=0., y=0., demand=0):
        Point.__init__(self, identifier, x, y)  # <- defines  x and y so there is no need to define them again.
        self.demand = demand  

    def __repr__(self):
        return "<Client at {}. id = {}, (x,y) = ({}, {}), demand = {}>".format(  
            hex(id(self)), self.identifier, self.x, self.y, self.demand)  

    def copy(self, other_client):
        self.__init__(other_client.identifier, other_client.x, other_client.y, other_client.demand)



class Depot(Point):

    def __init__(self, identifier="Depot", x=0., y=0.):
        Point.__init__(self, identifier, x, y)

    def __repr__(self):
        return "<Depot at {}. id = {}, (x,y) = ({}, {})>".format(hex(id(self)), self.identifier, self.x, self.y)


class Route:
    def __init__(self, clients_list, depot):
        self.clients_list = clients_list  # python list of instances of class Client
        self.depot = depot  # instance of class Depot. A route always starts and ends at a depot.

    def __repr__(self):  
        return "<Route at {}. [".format(hex(id(self))) + ", ".join([repr(cl) for cl in self.clients_list]) + \
               "]. {}.>".format(repr(self.depot))

    def __str__(self): 
        return "Route at {}. [".format(hex(id(self))) + ", ".join([str(cl) for cl in self.clients_list]) + \
               "]. {}.".format(str(self.depot))

    @property
    def total_demand(self):
        """This method returns the total demand of all the clients of the route."""
        total = 0
        for i in range(0, len(self.clients_list)):
            total += self.clients_list[i].demand
        return total

    @property
    def is_legal(self):
        """This method returns True if the route does not break any rule of the delivery problem. It returns False
        otherwise."""
        for i in range(0, len(self.clients_list) - 1):
            for j in range(i + 1, len(self.clients_list)):
                if self.clients_list[i] == self.clients_list[j]:
                    return False
        return True

    def check_same_depot(self, other_route):
        """This method returns True if this route has the same depot as other_route and False otherwise."""
        if self.depot == other_route.depot:
            return True
        else:
            return False


