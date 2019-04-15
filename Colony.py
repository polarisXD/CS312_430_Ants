import numpy as np
import math
import random


class Colony:

    def __init__(self, cities):
        self.cities = cities
        self.numCities = len(cities)
        self.initializeMatrices(cities)
        self.lowestCostSoFar = float("inf")

    # returns nothing
    def initializeMatrices(self, cities):
        # Convention: costMatrix[x][y] returns the cost of traveling from index x to index y
        # self.distanceMatrix = [[0 for x in range(self.numCities)] for y in range(self.numCities)]
        self.distanceMatrix = np.ndarray((self.numCities, self.numCities))
        for i in range(self.numCities):
            for j in range(self.numCities):
                self.distanceMatrix[i][j] = cities[i].costTo(cities[j])

        self.pharmoneMatrix = np.ndarray((self.numCities, self.numCities))
        self.pharmoneMatrix[:,:] = 1


    # return best Approximate solution
    def releaseTheAnts(self, numAnts, colony):
        for i in range(numAnts):
            # pick an ant
            ant = Ant(colony)
            cost = None
            path = None

            # find a solution
            noSolutionFound = True
            while noSolutionFound: # FIXME
                ant.currentPath = []
                ant.totalPathCost = 0

                # send it out
                index = random.randint(1, self.numCities - 1)
                path = ant.findPath(index)
                cost = ant.totalPathCost

                # check that it is a complete path
                if (len(path) == colony.numCities) and cost != float('inf'):
                    noSolutionFound = False

            # evaluate path length
            if cost < self.lowestCostSoFar:
                self.bestPathSoFar = []
                for cityIndex in path:
                    self.bestPathSoFar.append(self.cities[cityIndex])
                self.lowestCostSoFar = cost

        # return best path so far
        return self.bestPathSoFar, self.lowestCostSoFar


class Ant():
    currentPath = []  # list of tuples (x and y coordinates)
    totalPathCost = 0
    pharmoneBonus = 1
    colony = None

    def __init__(self, colony):
        self.currentPath = []  # list of tuples (x and y coordinates)
        self.totalPathCost = 0
        self.pharmoneBonus = 1
        self.colony = colony

    def getIndices(self, srcIndex):
        indices = []
        for i in range(len(self.colony.distanceMatrix[srcIndex])):
            if self.colony.distanceMatrix[srcIndex][i] < float('inf'):
                if i not in self.currentPath:
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

        if distance == 0:
            return float('inf')
        else:
            inverseDistance = 1.0 / distance
            desire = (math.pow(pharmone, a) * math.pow(inverseDistance, b))

            return desire

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
        # FIXME: Pharmones may be dominating after one round (the ants keep going back to the same path every time)
        choice = (random.randint(1, 100) / 100)

        threshold = 0
        for i in range(len(desires)):
            threshold = threshold + desires[i]
            if choice < threshold:
                return i
        # # It shouldn't run this line of code, but it may need to
        return 0

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

            # move to next city
            newIndex = self.moveToNext(newIndex)

            # check for dead end
            if newIndex == None:
                notDeadEnd = False

            # check if we have completed the loop
            if len(self.currentPath) == self.colony.numCities:
                # check that we can get back to the start city
                self.totalPathCost = self.totalPathCost + self.colony.distanceMatrix[self.currentPath[len(self.currentPath) - 1]][self.currentPath[0]]
                if self.totalPathCost != float('inf'):
                    pathNotFound = False
                else:

                    break



        # update pharmones
        if not pathNotFound and notDeadEnd:
            # FIXME: THIS ISN"T UPDATING
            for i in range(
                    len(self.currentPath) - 2):  # stop when we have connected the second-to-last node to last node
                x = self.currentPath[i]
                y = self.currentPath[i + 1]
                self.colony.pharmoneMatrix[x][y] = self.colony.pharmoneMatrix[x][y] + self.pharmoneBonus

            # update the step from the last node to the start node
            lastIndex = self.currentPath[-1]
            firstIndex = self.currentPath[0]
            pharmonesFromLastToFirst = self.colony.pharmoneMatrix[lastIndex][firstIndex]
            self.colony.pharmoneMatrix[lastIndex][firstIndex] = pharmonesFromLastToFirst + self.pharmoneBonus

        return self.currentPath
