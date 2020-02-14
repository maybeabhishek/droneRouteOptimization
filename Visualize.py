"""Implements a few functions to help visualise problems and theirs solutions."""
import numpy as np
import matplotlib.pyplot as plt  # very powerful module when it comes to plotting things



def plot_problem(problem, ax=None, **kwargs): 
    """This function creates a plot representing the problem to solve. It plots the depot and the clients (and the
    clients' demand if plot_demand is True).
    """
    if ax is None: 
        fig = plt.figure()  
        ax = fig.add_subplot(111)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Number of clients = {}, total demand = {}".format(problem.number_of_clients, problem.total_demand))
    ax.grid(False)
    

    x_depot = [problem.depot.x]
    y_depot = [problem.depot.y]
    x_clients = []
    y_clients = []
    for client in problem.clients_list:
        x_clients.append(client.x)
        y_clients.append(client.y)
    # return x_depot, y_depot, x_clients, y_clients
    ax.plot(x_depot, y_depot, marker="s", color="red", label="Depot", linestyle="None", ms=7, zorder=2)
    ax.plot(x_clients, y_clients, marker="o", color="blue", label="Clients (demand)", linestyle="None", ms=3, zorder=1)
    if kwargs.get("plot_demand", True):  
        for client in problem.clients_list:
            ax.text(client.x, client.y, str(client.demand), style="italic",
                    fontsize=kwargs.get("demand_size", 16), color="blue", ha="center", va="bottom", zorder=1)
    # plt.show()
    return ax


def plot_solution(solution, ax, color, **kwargs):
    """This function plots a solution. More precisely, it plots all the deliveries of the solution on the axis 'ax' with
    the color 'color'.
    """
    # setting up a dashed pattern. It represents a sequence of on/off ink (in points).
    dashes = [np.random.randint(12, 20+1), 2, np.random.randint(4, 6+1), 5]  
    
    for i, delivery in enumerate(solution.deliveries_list):
        label = None
        if i == 0:
            label = solution.name
        # Here, you should generate the x_list and y_list variables
        x_list = [delivery.depot.x]
        y_list = [delivery.depot.y]
        for client in delivery.clients_list:
            x_list.append(client.x)
            y_list.append(client.y)
        x_list.append(delivery.depot.x)
        y_list.append(delivery.depot.y)
        line, = ax.plot(x_list, y_list, color=color, marker="", linestyle="-", label=label, zorder=0)
        if kwargs.get('dashed', True):
            line.set_dashes(dashes)
    ax.legend(loc=0, fontsize='x-small', numpoints=1)
    # Drawing arrows representing the wind
    if kwargs.get('draw_wind', True) and solution.parameters.wind.speed > 0:
        nx = kwargs.get('nx', 10)  
        ny = kwargs.get('ny', 10)  
        rs = kwargs.get('rs', 0.1) 
        u = np.ones((nx, ny)) * solution.parameters.wind.x
        v = np.ones((nx, ny)) * solution.parameters.wind.y
        x = np.linspace(*ax.get_xbound(), nx + 2)[1:-1]
        y = np.linspace(*ax.get_ybound(), ny + 2)[1:-1]
        plt.quiver(x, y, u, v, angles='xy', units='dots', scale=rs,
                   width=2, headwidth=2, headlength=3.5, facecolor=color, edgecolor=color, zorder=-1, alpha=0.4)
    return ax


def get_random_color(color_list, **kwargs):
    """This function randomly generates a color that contrasts the most with the colors that are in color_list.
    """
    color = None
    n_color_candidates = kwargs.get('n_color_candidates', 10)
    color_candidates_matrix = np.random.rand(n_color_candidates, 3)  # creating matrix of candidate rgb values
    norm = 0.
    for i in range(n_color_candidates):
        candidate_color = color_candidates_matrix[i]
        candidate_norm = np.min([np.linalg.norm(existing_color-candidate_color) for existing_color in color_list])
        if candidate_norm > norm:
            norm = candidate_norm
            color = candidate_color
    color_list.append(color)
    return color


def plot_problem_solutions(problem, **kwargs):
    """This function plots a problem and its solutions using random colors. Returns the matplotlib axis where the plot
    has been made.
    """
    
    colors_list = [np.array([1., 1., 1.])] 
    ax = kwargs.get('ax')
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111)
    plot_problem(problem, ax, **kwargs)
    
    # ax2 = plot_problem(problem)
    # if kwargs.get("solution", False):
    #     for solution in kwargs.get("solution"):
    #         ax2 = plot_solution(solution, ax2, get_random_color(colors_list))
    for solution in problem.solutions_list:
        ax = plot_solution(solution, ax, get_random_color(colors_list, **kwargs), **kwargs)
    return ax
