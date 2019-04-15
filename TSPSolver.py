#!/usr/bin/python3

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
	def __init__( self, gui_view ):
		self._scenario = None

	def setupWithScenario( self, scenario ):
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

	def defaultRandomTour( self, time_allowance=60.0 ):
		results = {}
		cities = self._scenario.getCities()
		ncities = len(cities)
		foundTour = False
		count = 0
		bssf = None
		start_time = time.time()
		while not foundTour and time.time()-start_time < time_allowance:
			# create a random permutation
			perm = np.random.permutation( ncities )
			route = []
			# Now build the route using the random permutation
			for i in range( ncities ):
				route.append( cities[ perm[i] ] )
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
		#print(bssf.route)
		return results

	def create_matrix(self, cities):
		matrix = []
		for i in range(len(cities)):
			costs_row = []
			for j in range(len(cities)):
				costs_row.append(cities[i].costTo(cities[j]))
			matrix.append(costs_row)
		#print("Initial matrix:")
		#print(np.copy(matrix))
		return np.copy(matrix)

	def reduce_matrix(self, matrix, initial_lower_bound, ncities):
		lower_bound = initial_lower_bound
		for i in range(ncities):
			min_cost = min(matrix[i,:])
			if min_cost == 0:
				continue
			elif min_cost < math.inf:
					matrix[i,:] -= min_cost
					lower_bound += min_cost
		for j in range(ncities):
			min_cost = min(matrix[:,j])
			if min_cost == 0:
				continue
			elif min_cost < math.inf:
					matrix[:,j] -= min_cost
					lower_bound += min_cost
		#print("Reduced:")
		#print(np.copy(matrix))
		#print("Lower bound:",lower_bound)
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

	def greedy( self,time_allowance=60.0 ):
		results = {}
		cities = self._scenario.getCities()
		ncities = len(cities)
		reduced_matrix, lower_bound = self.reduce_matrix(self.create_matrix(cities), 0, ncities)
		count = 0
		bssf = None
		start_time = time.time()
		for i in range(ncities):
			foundTour = False
			#print("Start city:", i)
			start_city = i
			current_row = start_city
			path = [start_city]
			parent_matrix = np.copy(reduced_matrix)
			states = [(parent_matrix,lower_bound,0,path)] #matrix, lower_bound, which node you're coming from
			while not foundTour and time.time()-start_time < time_allowance:
				current_state = states[-1]
				child_matrix = np.copy(current_state[0])
				current_lower_bound = current_state[1]
				current_depth = current_state[2]
				current_path = current_state[3]
				current_row = current_path[-1]
				if current_depth == ncities:
					foundTour = True
					if current_row == start_city:
						#made it back to start city
						solution = TSPSolution([cities[x] for x in current_path[:-1]])
						if solution.cost < math.inf:
							count += 1
						#print("Cost:",solution.cost)
						if not bssf:
							bssf = solution
						elif solution.cost < bssf.cost:
							bssf = solution
					else:
						#not a valid solution
						continue
				else:
					child_matrix, current_lower_bound = self.reduce_matrix(child_matrix, current_lower_bound, ncities)
					min_pos = np.argmin(child_matrix[current_row])
					if min_pos == start_city and current_depth + 1 != ncities: #check for unvisited cities
						#get next minimum
						costs = np.copy(child_matrix[current_row])
						costs = [(costs[i],i) for i in range(len(costs))]
						min_pos = sorted(costs)[1][1]
					min_cost = child_matrix[current_row][min_pos]
					child_matrix[current_row,:] = math.inf
					child_matrix[:,min_pos] = math.inf
					child_matrix[min_pos,current_row] = math.inf
					child_lower_bound = current_lower_bound + min_cost
					states.append((child_matrix,child_lower_bound,current_depth + 1, current_path + [min_pos]))
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

	def branchAndBound( self, time_allowance=60.0 ):
		pass
		# cities = self._scenario.getCities()
		tsp = TSP()
		cities = self._scenario.getCities()
		starterNode = tsp.getStarter(cities)
		tsp.makeStateSpaceTree(cities)
		tsp.chewPriorityQueue(time_allowance)
		solution = TSPSolution(tsp.path)
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
		finalPath = []
		finalCost = float('inf')
		cities = self._scenario.getCities()
		numAnts = len(cities) // 2
		start_time = time.time()
		colony = Colony(cities)
		current_time = time.time()
		solution_count = 0

		while current_time - start_time < time_allowance:
			bestPath, lowestCost = colony.releaseTheAnts(numAnts)
			if lowestCost < finalCost:
				finalCost = lowestCost
				finalPath = bestPath
				solution_count += 1
			current_time = time.time()

		results = {}
		results['cost'] = finalCost
		results['time'] = current_time - start_time
		results['count'] = solution_count
		cities = self._scenario.getCities()
		city_path = []
		for i in range(len(finalPath)):
			city_path.append(cities[finalPath[i]])
		results['soln'] = TSPSolution(city_path)
		results['max'] = None
		results['total'] = None
		results['pruned'] = None
		return results
