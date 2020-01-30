import numpy as np

class Drone:

    def __init__(self,capacity=100,speed=10,acd=0.01):
        self.capacity = capacity   # Capacity I am considering to be relative so no units
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
    #  """This class represents a point on the map. x and y are the coordinates of the point."""

    def __init__(self, identifier="", x=0, y=0):
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


class DeliveryParameters:
    def __init__(self,drone,wind,cost_func=None):
        self.drone = drone
        self.wind = wind
        self.cost_func = None
        if cost_func is not None:
            self.cost_func = cost_func
        

class  Delivery:
    def __init__(self,route: Route, parameters: DeliveryParameters):
        self.route = route
        self.parameters = parameters

    @property
    def clients_list(self):
        self.route.clients_list
    
    @property
    def clients_list(self, new_list):
        self.route.clients_list = new_list
    
    @property
    def total_demand(self):
        self.route.depot

    @property
    def depot(self):
        self.route.depot
    
    @property
    def drone(self):
        self.parameters.drone
    
    @property 
    def wind(self):
        self.parameters.wind

    @property
    def is_legal(self):
        if(self.route.is_legal and self.route.total_demand <= self.drone.capacity):
            return True
        else:
            return False

    def cost(self):
        """ Returns cost of delivery according to cost functiion"""
        if self.parameters.cost_func is None:
            return None
        else:
            cost = 0
            a = len(self.clients_list)
            if a==0:
                return 0
            else:
                cost+=self.parameters.cost_func(self.depot,self.clients_list[0],self.drone, self.wind)
                cost+=self.parameters.cost_func(self.clients_list[-1],self.depot,self.drone, self.wind)

                if a==1:
                    return cost
                if a>1:
                    for i in range(0,len(self.clients_list)-1):
                        cost+=self.parameters.cost_func(self.clients_list[i],self.clients_list[i+1],self.drone, self.wind)
                    return cost

    def cost_and_savings(self):
        """Returns  cost and savings based on cost function"""
        if self.parameters.cost_func is None:
            return None,None
        else:
            normalcost = 0
            a = len(self.clients_list)
            if a<=1:
                return self.cost(), 0
            else:
                for i in range(0,len(self.clients_list)):
                    normalcost +=self.parameters.cost_func(self.clients_list[i], self.depot, self.drone, self.wind)
                    normalcost +=self.parameters.cost_func(self.depot, self.clients_list[i], self.drone, self.wind)
                savings = normalcost - self.cost()
                return self.cost(), savings

    def print(self):
        print("Delivery at {}".format(hex(id(self))))
        print(" ",self.route)
        print(" total demand = {}, drone capacity = {}, wind= {}, cost {}, savings {} ".format(self.total_demand, self.drone.capacity, self.wind.vector, *self.cost_and_savings()))

    
class Solution:

    def __init__(self, name = "Unnamed solution", deliveries_list = None, parameters = None):
        self.name = name
        self. deliveries_list = list()
        if deliveries_list is not None:
            self.deliveries_list = deliveries_list
        self.parameters = parameters
    
    @property
    def is_legal(self):
        a = len(self.deliveries_list)
        for i in range(0,a):
            if self.deliveries_list[i].is_legal is False:
                return False
        
        ctot = []
        for i in range(0,a):
            for j in self.deliveries_list[i].clients_list:
                ctot.append(j)
        lctot = len(ctot)

        for k in range(0, lctot - 1):
            for l in  range(k+1, lctot):
                if ctot[k]==ctot[l]:
                    return False
        return True
    
    def cost_and_savings(self):
        # Returns total cost and savings of solution
        if not self.deliveries_list:
            return 0,0
        else:
            cost = 0
            savings = 0
            length = len(self.deliveries_list)
            for i  in range(length):
                if self.deliveries_list[i].parameters.cost_func:
                    interest = self.deliveries_list[i].cost_and_savings()
                    cost += interest[0]
                    savings += interest[1]
            return cost, savings

class Problem:

    def __init__(self, depot = None, client_list = None):
        self.depot = depot
        self.client_list = list()
        if client_list is not None:
            self.client_list = client_list
        self._number_of_generated_clients  = 0
        self.solutions_list = list()


    @property
    def number_of_generated_clients(self):
        return self._number_of_generated_clients
        
    @property
    def number_of_clients(self):
        return len(self.clients_list)

    @property
    def total_demand(self):
        total = 0
        for client in self.clients_list:
            if client:
                total += client.demand
        return total

    def print_clients(self):
        for client in self.clients_list:
            print(repr(client))
    
    def print_depot(self):
        print(repr(self.depot))

    def print_solutions(self, detailed=False):
        for solution in self.solutions_list:
            solution.print(detailed)
            print("\n\n")

    def remove_solution_index(self, index):
        del self.solutions_list[index]
    
    def remove_solution_named(self, name):
        for i, solution in enumerate(self.solutions_list):
            if solution["Name"] == name:
                del self.solutions_list[i]
                break
    
    def clear_solutions(self):
        self.solutions_list.clear()
    
    def generate_random_clients(self, amount=1, x=(-10000, 10000), y=(-10000, 10000), demand=(1, 100)):
    
        for i in range(1, amount + 1):
            self._number_of_generated_clients += 1
            client_i = Client("random client {}".format(self._number_of_generated_clients),
                                ((x[1] - x[0]) * (np.random.rand(1)) + x[0])[0],
                                ((y[1] - y[0]) * (np.random.rand(1)) + y[0])[0],
                                (np.random.randint(demand[0], demand[1] + 1, 1))[0])
            self.clients_list.append(client_i)
    
    def export_csv(self, file_name, cell_separator=";"):
        with open(file_name, "w", newline='') as f:
            writer = csv.writer(f, delimiter=cell_separator)
            writer.writerow(["Delivery optimization problem"])
            writer.writerow(["type", "identifier", "x", "y", "demand"])
            writer.writerow(["depot", self.depot.identifier, self.depot.x, self.depot.y])
            for client in self.clients_list:
                writer.writerow(["client", client.identifier, client.x, client.y, client.demand])