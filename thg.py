from ortools.linear_solver import pywraplp
import numpy as np
import networkx as nx

def create_data_model(G, r):
    data = {}
    data["num_vars"] = int(G.order()) 
    dist_dict = dict(nx.all_pairs_shortest_path(G))
    path = nx.all_pairs_shortest_path(G)
    dpath = {x[0]:x[1] for x in path}
    node_list = list(G.nodes)

    data["constraint_coeffs"] = []
    d = np.zeros((data["num_vars"],data["num_vars"]))
    nodes = list(G.nodes())

    lengths = dict(nx.all_pairs_shortest_path_length(G))

    for i, u in enumerate(nodes):
        for j, v in enumerate(nodes):
            d[i, j] = lengths[u][v]

    for i in range(1, data["num_vars"]):
        for j in range(0,i):
            empty_vec = np.zeros(data["num_vars"])
            for k in range(0, data["num_vars"]):
                empty_vec[k] = abs(min(d[k,i],r+1) - min(d[k,j],r+1))
            data["constraint_coeffs"].append(empty_vec)

    data["obj_coeffs"] = [1]*data["num_vars"]
    data["num_constraints"] = len(data["constraint_coeffs"])
    return data

for n in range(1,51):
    G = nx.cycle_graph(n)
    min_throttling = int(G.order())
    for r in range(nx.diameter(G)+1):
        data = create_data_model(G, r)
        solver = pywraplp.Solver.CreateSolver("SCIP")

        infinity = solver.infinity()
        x = {}
        for j in range(data["num_vars"]):
            x[j] = solver.IntVar(0, 1, "x[%i]" % j)

        for i in range(data["num_constraints"]):
            constraint = solver.RowConstraint(1, infinity, "")
            for j in range(data["num_vars"]):
                constraint.SetCoefficient(x[j], data["constraint_coeffs"][i][j])

        objective = solver.Objective()
        for j in range(data["num_vars"]):
            objective.SetCoefficient(x[j], data["obj_coeffs"][j])
        objective.SetMinimization()

        status = solver.Solve()
        obj_val = solver.Objective().Value()
        min_throttling = min(min_throttling, obj_val+r)
            
    print(n, min_throttling)