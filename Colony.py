class Colony:
    distanceMatrix = None
    pharmoneMatrix = None
    numCities = 0
    ants = []  # list of Ant objects
    bestPathSoFar = []
    lowestCostSoFar = 0

    def __init__(self, cities):
        self.initializeMatrices(cities)
        self.numCities = len(cities) - 1
        pass

    # returns nothing
    def initializeMatrices(self, cities):
        # Convention: costMatrix[x][y] returns the cost of traveling from index x to index y
        self.distanceMatrix = [[0 for x in range(self.numCities)] for y in range(self.numCities)]
		for i in range(self.numCities):
			for j in range(self.numCities):
				self.distanceMatrix[i][j] = cities[i].costTo(cities[j])
        self.pharmoneMatrix = [[0 for x in range(self.numCities)] for y in range(self.numCities)]

    # return best Approximate solution
    def releaseTheAnts(self, numAnts):
        for i in range(numAnts):
            # pick an ant
            ant = self.ants[i]

            # send it out
            path = ant.findPath(i)
            cost = ant.totalPathCost

            # evaluate its performance
            if cost < self.lowestCostSoFar:
                self.bestPathSoFar = path
                self.lowestCostSoFar = cost

        return self.bestPathSoFar, self.lowestCostSoFar



class Ant(Colony):
    currentPath = []  # list of tuples (x and y coordinates)
    totalPathCOst = 0
    pharmoneBonus = 1

    def __init__(self, cities):
        super().__init__(cities)  # allows us to access the Colony matrices by inheriting from it

    def getIndices(self, srcIndex):
        indices = []
        for i in range(self.distanceMatrix[srcIndex]):
            if self.distanceMatrix[srcIndex][i] < float('inf'):
                indices.append((srcIndex,i))
        return indices

    def getCosts(self, indexTuples):
        distances = []
        for tuple in indexTuples:
            x = indexTuples[0]
            y = indexTuples[1]
            distance = self.distanceMatrix[x][y]
            distances.append(distance)
        return distances

    def getPharmones(self, indexTuples):
        pharmones = []
        for tuple in indexTuples:
            x = indexTuples[0]
            y = indexTuples[1]
            pharmone = self.pharmoneMatrix[x][y]
            pharmones.append(pharmone)
        return pharmones


    def getDesire(self, distance, pharmone):
        a = 1
        b = 1

        inverseCost = 1 / distance
        total = distance + pharmone

        desire = ((pharmone ^ a) * (inverseCost ^ b)) / total

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
        cost = self.distanceMatrix[currentCityIndex][nextCityIndex]
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
            if len(self.currentPath) == self.numCities:
                pathNotFound = False

        # update pharmones
        if not pathNotFound and notDeadEnd:

            for i in range(len(self.currentPath) - 2): # stop when we have connected the second-to-last node to last node
                x = self.currentPath[i]
                y = self.currentPath[i + 1]
                self.pharmoneMatrix[x][y] = self.pharmoneMatrix[x][y] + self.pharmoneBonus

            # update the step from the last node to the start node
            lastIndex = self.currentPath[-1]
            firstIndex = self.currentPath[0]
            costFromLastToFirst = self.pharmoneMatrix[lastIndex][firstIndex]
            self.pharmoneMatrix[lastIndex][firstIndex] = costFromLastToFirst + self.pharmoneBonus
            
        return self.currentPath

