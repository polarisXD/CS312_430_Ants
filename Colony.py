import numpy as np
import math
import random


class Colony:
    distanceMatrix = None
    pheromoneMatrix = None
    numCities = 0
    cities = None
    bestPathSoFar = []
    lowestCostSoFar = math.inf

    def __init__(self, cities):
        self.cities = cities
        self.numCities = len(cities)
        self.initializeMatrices(cities)

    # returns nothing
    def initializeMatrices(self, cities):
        # Convention: costMatrix[x][y] returns the cost of traveling from index x to index y
        # self.distanceMatrix = [[0 for x in range(self.numCities)] for y in range(self.numCities)]
        self.distanceMatrix = np.ndarray((self.numCities, self.numCities))
        for i in range(self.numCities):
            for j in range(self.numCities):
                self.distanceMatrix[i][j] = cities[i].costTo(cities[j])

        self.pheromoneMatrix = np.ndarray((self.numCities, self.numCities))
        self.pheromoneMatrix[:,:] = 1

    # return best Approximate solution
    def releaseTheAnts(self, numAnts):
        perm = np.random.permutation(self.numCities)
        random_cities = perm[0:numAnts]
        for i in range(len(random_cities)):
            # pick an ant
            ant = Ant(self)

            # send it out
            path = ant.findPath(random_cities[i])
            cost = ant.totalPathCost

            # evaluate its performance
            if cost < self.lowestCostSoFar:
                self.bestPathSoFar = path
                self.lowestCostSoFar = cost

        return self.bestPathSoFar, self.lowestCostSoFar


class Ant:
    currentPath = []  # list of tuples (x and y coordinates)
    totalPathCost = 0
    pheromoneBonus = 1
    colony = None

    def __init__(self, colony):
        self.currentPath = []  # list of tuples (x and y coordinates)
        self.totalPathCost = 0
        self.pheromoneBonus = 1
        self.colony = colony

    def getIndices(self, srcIndex):
        indices = []
        for i in range(len(self.colony.distanceMatrix[srcIndex])):
            if self.colony.distanceMatrix[srcIndex][i] < float('inf'):
                if i not in self.currentPath:
                    indices.append((srcIndex, i))
        return indices

    # Used for connecting the circuit
    def getIndicesWithoutPath(self, srcIndex):
        indices = []
        for i in range(len(self.colony.distanceMatrix[srcIndex])):
            if self.colony.distanceMatrix[srcIndex][i] < math.inf:
                indices.append((srcIndex, i))
        return indices

    def getCosts(self, indexTuples):
        distances = []
        for tuple in indexTuples:
            x = tuple[0]
            y = tuple[1]
            distance = self.colony.distanceMatrix[x][y]
            distances.append(distance)
        return distances

    def getPheromones(self, indexTuples):
        pheromones = []
        for tuple in indexTuples:
            x = tuple[0]
            y = tuple[1]
            pheromone = self.colony.pheromoneMatrix[x][y]
            pheromones.append(pheromone)
        return pheromones

    def getDesire(self, distance, pheromone):
        a = 1.0
        b = 1.0

        if distance == 0:
            return float('inf')
        else:
            inverseDistance = 1.0 / distance
            desire = (math.pow(pheromone, a) * math.pow(inverseDistance, b))

            return desire

    # Converts the individual desires into probability-friendly numbers (0 <= desire <= 1)
    def normalizeDesires(self, desires):
        totalDesires = sum(desires)

        if totalDesires == float('inf'):
            for i in range(len(desires)):
                if desires[i] == float('inf'):
                    desires[i] = 1
                else:
                    desires[i] = 0
        else:
            for i in range(len(desires)):
                desires[i] = (desires[i] / totalDesires)

        return desires

    # return the index of the value in "desires" (not the index of the city) that the ant wants to go to most.
    def chooseFromDesires(self, desires):
        # FIXME: Pheromones may be dominating after one round (the ants keep going back to the same path every time)
        # Solution would probably be to modify "a" in getDesire, but I don't know what a proper exponent would be
        choice = random.randint(1, 1000)
        choice = choice / 1000

        threshold = 0
        for i in range(len(desires)):
            threshold = threshold + desires[i]  # Make sure normalizeDesires is run before this
            if choice <= threshold:
                return i
        # # It shouldn't run this line of code, but it may need to
        return 0

    def moveToNext(self, currentCityIndex):

        # see where we can go
        indices = self.getIndices(currentCityIndex)

        distances = self.getCosts(indices)
        pheromones = self.getPheromones(indices)

        # determine how much we want to go there
        desires = []
        for i in range(len(distances)):
            distance = distances[i]
            pheromone = pheromones[i]

            desire = self.getDesire(distance, pheromone)
            desires.append(desire)

        # decide where we want to go
        desires = self.normalizeDesires(desires)
        if len(desires) > 0:
            nextCityIndex = indices[self.chooseFromDesires(desires)][1]

            # update the cost and path for going there
            cost = self.colony.distanceMatrix[currentCityIndex][nextCityIndex]
            self.totalPathCost = self.totalPathCost + cost
            self.currentPath.append(nextCityIndex)

            return nextCityIndex

        else:
            return None

    # returns a path (the currentPath of the ant at the end of its traversal)
    def findPath(self, srcCityIndex):

        # initialize variables for loop
        pathNotFound = True
        notDeadEnd = True

        newIndex = srcCityIndex
        self.currentPath.append(newIndex)

        # generate a path
        while notDeadEnd and pathNotFound:
            # print(len(self.currentPath))

            # move to next city
            newIndex = self.moveToNext(newIndex)

            # check for dead end
            if newIndex is None:
                notDeadEnd = False

            # check if we have completed the loop
            if len(self.currentPath) == self.colony.numCities:
                pathNotFound = False

        # update pheromones
        if not pathNotFound and notDeadEnd:
            path_end = self.currentPath[-1]
            indices = self.getIndicesWithoutPath(path_end)
            path_start = self.currentPath[0]
            circuit_found = False  # This variable says whether or not the end of the path connects to the start
            for i in range(len(indices)):
                if indices[i][1] == path_start:
                    circuit_found = True
            if circuit_found:
                self.totalPathCost += self.colony.distanceMatrix[path_end][path_start]

                for i in range(
                        len(self.currentPath) - 2):  # stop when we have connected the second-to-last node to last node
                    x = self.currentPath[i]
                    y = self.currentPath[i + 1]
                    self.colony.pheromoneMatrix[x][y] = self.colony.pheromoneMatrix[x][y] + self.pheromoneBonus

                # update the step from the last node to the start node
                lastIndex = self.currentPath[-1]
                firstIndex = self.currentPath[0]
                pheromonesFromLastToFirst = self.colony.pheromoneMatrix[lastIndex][firstIndex]
                self.colony.pheromoneMatrix[lastIndex][firstIndex] = pheromonesFromLastToFirst + self.pheromoneBonus
            else:
                self.totalPathCost = math.inf
                pathNotFound = True
                notDeadEnd = False
        else:
            self.totalPathCost = math.inf

        print(self.currentPath)
        return self.currentPath
