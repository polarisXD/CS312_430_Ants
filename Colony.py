import numpy as np
import math

class Colony:
    distanceMatrix = None
    pharmoneMatrix = None
    numCities = 0
    cities = None
    bestPathSoFar = []
    lowestCostSoFar = 0

    def __init__(self, cities):
        self.cities = cities
        self.numCities = len(cities) - 1
        self.initializeMatrices(cities)

    # returns nothing
    def initializeMatrices(self, cities):
        # Convention: costMatrix[x][y] returns the cost of traveling from index x to index y
        self.distanceMatrix = [[0 for x in range(self.numCities)] for y in range(self.numCities)]

        for i in range(self.numCities):
            for j in range(self.numCities):
                self.distanceMatrix[i][j] = cities[i].costTo(cities[j])

        self.pharmoneMatrix = [[0 for x in range(self.numCities)] for y in range(self.numCities)]

    # return best Approximate solution
    def releaseTheAnts(self, numAnts, colony):
        for i in range(numAnts):
            # pick an ant
            ant = Ant(colony)

            # send it out
            path = ant.findPath(i)
            cost = ant.totalPathCost

            # evaluate its performance
            if cost < self.lowestCostSoFar:
                self.bestPathSoFar = path
                self.lowestCostSoFar = cost

        return self.bestPathSoFar, self.lowestCostSoFar


class Ant():
    currentPath = []  # list of tuples (x and y coordinates)
    totalPathCOst = 0
    pharmoneBonus = 1
    colony = None

    def __init__(self, colony):
        self.colony = colony

    def getIndices(self, srcIndex):
        indices = []
        for i in range(len(self.colony.distanceMatrix[srcIndex])):
            if self.colony.distanceMatrix[srcIndex][i] < float('inf'):
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

    def getPharmones(self, indexTuples):
        pharmones = []
        for tuple in indexTuples:
            x = tuple[0]
            y = tuple[1]
            pharmone = self.colony.pharmoneMatrix[x][y]
            pharmones.append(pharmone)
        return pharmones

    def getDesire(self, distance, pharmone):
        a = 1.0
        b = 1.0
        if distance > 0:
            inverseDistance = (1.0) / distance
        else:
            inverseDistance = 0 # FIXME: This might not be the best fix
        total = float(distance + pharmone)

        if total > 0:
            desire = (math.pow(pharmone, a) * math.pow(inverseDistance,b)) / total
        else:
            desire = 1 # FIXME: This might not be the best fix

        print(distance)
        print(desire)
        print()


        return desire

    def chooseFromDesires(self, desires):
        # FIXME: FILL IN PROBABILISTIC FUNCTION HERE

        pass

    def moveToNext(self, currentCityIndex):

        # see where we can go
        indices = self.getIndices(currentCityIndex)

        distances = self.getCosts(indices)
        pharmones = self.getPharmones(indices)

        # determine how much we want to go there
        desires = []
        for i in range(len(distances)):
            distance = distances[i]
            pharmone = pharmones[i]

            desire = self.getDesire(distance, pharmone)
            desires.append(desire)

        # decide where we want to go
        nextCityIndex = self.chooseFromDesires(desires)

        # update the cost and path for going there
        cost = self.colony.distanceMatrix[currentCityIndex][nextCityIndex]
        self.totalPathCost = self.totalPathCost + cost
        self.currentPath.append(nextCityIndex)

        return nextCityIndex

    # returns a path (the currentPath of the ant at the end of its traversal)
    def findPath(self, srcCityIndex):

        # initialize variables for loop
        pathNotFound = True
        notDeadEnd = True

        newIndex = srcCityIndex
        self.currentPath.append(newIndex)

        # generate a path
        while notDeadEnd and pathNotFound:
            # move to next city
            newIndex = self.moveToNext(newIndex)

            # check for dead end
            if (len(set(self.currentPath)) == len(self.currentPath)):
                notDeadEnd = False

            # check if we have completed the loop
            if len(self.currentPath) == Colony.numCities:
                pathNotFound = False

        # update pharmones
        if not pathNotFound and notDeadEnd:

            for i in range(
                    len(self.currentPath) - 2):  # stop when we have connected the second-to-last node to last node
                x = self.currentPath[i]
                y = self.currentPath[i + 1]
                self.colony.pharmoneMatrix[x][y] = self.colony.pharmoneMatrix[x][y] + self.pharmoneBonus

            # update the step from the last node to the start node
            lastIndex = self.currentPath[-1]
            firstIndex = self.currentPath[0]
            costFromLastToFirst = self.colony.pharmoneMatrix[lastIndex][firstIndex]
            self.colony.pharmoneMatrix[lastIndex][firstIndex] = costFromLastToFirst + self.pharmoneBonus

        return self.currentPath
