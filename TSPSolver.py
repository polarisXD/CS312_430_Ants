# !/usr/bin/python3

from which_pyqt import PYQT_VER

if PYQT_VER == 'PYQT5':
    from PyQt5.QtCore import QLineF, QPointF
else:
    raise Exception('Unsupported Version of PyQt: {}'.format(PYQT_VER))

import time
import numpy as np
from TSPClasses import *
from BranchAndBound import *
from Colony import *
import heapq
import itertools


class TSPSolver:
    def __init__(self, gui_view):
        self._scenario = None

    def setupWithScenario(self, scenario):
        self._scenario = scenario

    ''' <summary>
        This is the entry point for the default solver
        which just finds a valid random tour.  Note this could be used to find your
        initial BSSF.
        </summary>
        <returns>results dictionary for GUI that contains three ints: cost of solution,
        time spent to find solution, number of permutations tried during search, the
        solution found, and three null values for fields not used for this
        algorithm</returns>
    '''

    def defaultRandomTour(self, time_allowance=60.0):
        results = {}
        cities = self._scenario.getCities()
        ncities = len(cities)
        foundTour = False
        count = 0
        bssf = None
        start_time = time.time()
        while not foundTour and time.time() - start_time < time_allowance:
            # create a random permutation
            perm = np.random.permutation(ncities)
            route = []
            # Now build the route using the random permutation
            for i in range(ncities):
                route.append(cities[perm[i]])
            bssf = TSPSolution(route)
            count += 1
            if bssf.cost < np.inf:
                # Found a valid route
                foundTour = True
        end_time = time.time()
        results['cost'] = bssf.cost if foundTour else math.inf
        results['time'] = end_time - start_time
        results['count'] = count
        results['soln'] = bssf
        results['max'] = None
        results['total'] = None
        results['pruned'] = None
        # print(bssf.route)
        return results

    def create_matrix(self, cities):
        matrix = []
        for i in range(len(cities)):
            costs_row = []
            for j in range(len(cities)):
                costs_row.append(cities[i].costTo(cities[j]))
            matrix.append(costs_row)
        # print("Initial matrix:")
        # print(np.copy(matrix))
        return np.copy(matrix)

    def reduce_matrix(self, matrix, initial_lower_bound, ncities):
        lower_bound = initial_lower_bound
        for i in range(ncities):
            min_cost = min(matrix[i, :])
            if min_cost == 0:
                continue
            elif min_cost < math.inf:
                matrix[i, :] -= min_cost
                lower_bound += min_cost
        for j in range(ncities):
            min_cost = min(matrix[:, j])
            if min_cost == 0:
                continue
            elif min_cost < math.inf:
                matrix[:, j] -= min_cost
                lower_bound += min_cost
        # print("Reduced:")
        # print(np.copy(matrix))
        # print("Lower bound:",lower_bound)
        return matrix, lower_bound

    ''' <summary>
        This is the entry point for the greedy solver, which you must implement for
        the group project (but it is probably a good idea to just do it for the branch-and
        bound project as a way to get your feet wet).  Note this could be used to find your
        initial BSSF.
        </summary>
        <returns>results dictionary for GUI that contains three ints: cost of best solution,
        time spent to find best solution, total number of solutions found, the best
        solution found, and three null values for fields not used for this
        algorithm</returns>
    '''

    def greedy(self, time_allowance=60.0):
        results = {}
        cities = self._scenario.getCities()
        ncities = len(cities)
        reduced_matrix, lower_bound = self.reduce_matrix(self.create_matrix(cities), 0, ncities)
        foundTour = False
        count = 0
        bssf = None
        start_time = time.time()
        row = 0
        path = [row]
        parent_matrix = np.copy(reduced_matrix)
        states = [(parent_matrix, lower_bound, 0, path)]  # matrix, lower_bound, which node you're coming from
        while not foundTour and time.time() - start_time < time_allowance:
            current_state = states.pop()  # should get next minimum value
            current_matrix = np.copy(current_state[0])
            current_lower_bound = current_state[1]
            current_depth = current_state[2]

            current_row = current_state[3][-1]
            current_path = current_state[3]
            if current_depth == ncities:
                if current_row == 0:
                    # found solution
                    foundTour = True
                    count += 1
                    bssf = TSPSolution([cities[x] for x in current_path[:-1]])
                else:
                    # invalid solution
                    continue
            else:
                current_matrix, current_lower_bound = self.reduce_matrix(current_matrix, current_lower_bound, ncities)
                children_costs = np.copy(current_matrix[current_row])
                children_costs = [(children_costs[i], i) for i in range(len(children_costs))]
                descending_costs = sorted(children_costs, reverse=True)  # makes minimum cost get appended last
                for cost in descending_costs:
                    child_matrix = np.copy(current_matrix)
                    if cost[0] < math.inf:
                        column = cost[1]
                        child_lower_bound = current_lower_bound + cost[0]
                        child_matrix[current_row, :] = math.inf
                        child_matrix[:, column] = math.inf
                        child_matrix[column, current_row] = math.inf
                        states.append((child_matrix, child_lower_bound, current_depth + 1, current_path + [column]))
        end_time = time.time()
        results['cost'] = bssf.cost if foundTour else math.inf
        results['time'] = end_time - start_time
        results['count'] = count
        results['soln'] = bssf
        results['max'] = None
        results['total'] = None
        results['pruned'] = None
        return results

    ''' <summary>
        This is the entry point for the branch-and-bound algorithm that you will implement
        </summary>
        <returns>results dictionary for GUI that contains three ints: cost of best solution,
        time spent to find best solution, total number solutions found during search (does
        not include the initial BSSF), the best solution found, and three more ints:
        max queue size, total number of states created, and number of pruned states.</returns>
    '''

    def branchAndBound(self, time_allowance=60.0):
        pass
        # cities = self._scenario.getCities()
        tsp = TSP()
        cities = self._scenario.getCities()
        starterNode = tsp.getStarter(cities)
        tsp.makeStateSpaceTree(cities)
        tsp.chewPriorityQueue(time_allowance)
        # cost = tsp.upperBound
        # route = tsp.path

        # print(cost)
        # print(route)
        results = {}
        solution = TSPSolution(tsp.path)
        results['cost'] = tsp.upperBound
        results['time'] = tsp.time
        results['count'] = tsp.numSolutions
        results['soln'] = solution
        results['max'] = tsp.maxQueueSize
        results['total'] = tsp.totalStates
        results['pruned'] = tsp.numPruned
        return results

    ''' <summary>
        This is the entry point for the algorithm you'll write for your group project.
        </summary>
        <returns>results dictionary for GUI that contains three ints: cost of best solution,
        time spent to find best solution, total number of solutions found during search, the
        best solution found.  You may use the other three field however you like.
        algorithm</returns>
    '''

    def fancy(self, time_allowance=60.0):
        results = {}
        finalPath = []
        finalCost = float('inf')
        cities = self._scenario.getCities()
        numAnts = len(cities) // 2
        loopTimes = 100
        startTime = time.time()
        # print(len(cities))
        colony = Colony(cities)


        for i in range(loopTimes):
            if (time.time() - startTime > time_allowance):
                break
            bestPath, lowestCost = colony.releaseTheAnts(numAnts, colony)
            if (finalCost > lowestCost):
                finalCost = lowestCost
                finalPath = bestPath

        endTime = time.time()
        solution = TSPSolution(colony.bestPathSoFar)

        if solution.cost == float('inf'):
            for city in bestPath:
                print(city._index)
            print('here')


        results['cost'] = solution.cost
        results['time'] = endTime - startTime
        results['count'] = 0
        results['soln'] = solution
        results['max'] = None
        results['total'] = None
        results['pruned'] = None
        return results
