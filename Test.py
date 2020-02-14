"""Case study - Partitioning clients' lists for pb250_b"""

import Drone as dro
import CostFunction as cost
import Visualize as viz
import matplotlib.pyplot as plt
import numpy as np

drone1 = dro.Drone(180, 10.3, 0.013)
drone2 = dro.Drone(510, 12.5, 0.024)

wind1 = dro.Wind(0, 0)

param1 = dro.DeliveryParameters(drone1, wind1, cost.cost_b)
param2 = dro.DeliveryParameters(drone2, wind1, cost.cost_b)

problem_g = dro.Problem()
problem_g.import_csv('pb250_b.csv')

best_cost = float('inf')
limit = 0
for i in range(5, 151):
    clients_list_1 = []
    clients_list_2 = []
    for client in problem_g.clients_list:
        if client.demand > i:
            clients_list_2.append(client)
        else:
            clients_list_1.append(client)
    problem1 = cost.Problem(problem_g.depot, clients_list_1)
    problem2 = cost.Problem(problem_g.depot, clients_list_2)
    init_1 = dro.clarke_and_wright_init(problem1, param1)
    init_2 = dro.clarke_and_wright_init(problem2, param2)
    deliveries_1 = dro.parallel_build_deliveries(problem1, param1, *init_1)
    deliveries_2 = dro.parallel_build_deliveries(problem2, param2, *init_2)
    solution1 = cost.Solution("drone1", deliveries_1, param1)
    solution2 = cost.Solution("drone2", deliveries_2, param2)
    if solution1.cost_and_savings()[0] + solution2.cost_and_savings()[0] < best_cost:
        best_cost = solution1.cost_and_savings()[0] + solution2.cost_and_savings()[0]
        limit = i
    print(limit, best_cost)

min = float('inf')
max = 0
for client in problem_g.clients_list:
    if client.demand < min:
        min = client.demand
    if client.demand > max:
        max = client.demand
print(min)
print(max)


viz.plot_problem_solutions(problem_g)
plt.show()
