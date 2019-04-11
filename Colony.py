class Colony:
    distanceMatrix = None
    pharmoneMatrix = None
    ants = []  # list of Ant objects
    bestPathSoFar = []

    def __init__(self, cities):
        self.initializeMatrices(cities)
        pass

    # returns nothing
    def initializeMatrices(self, cities):
        pass

    # return best Approximate solution
    def releaseTheAnts(self, numAnts):
        for i in range(len(self.ants)):
            break

        pass


class Ant(Colony):
    currentPath = []  # list of tuples (x and y coordinates)
    totalPathCOst = 0

    def __init__(self, cities):
        super().__init__(cities)  # allows us to access the Colony matrices by inheriting from it

    def getIndices(self, srcIndex):
        indices = []
        for i in range(self.distnaceMatrix[srcIndex]):
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

    def moveToNext(self, currentCityIndex):

        # see where we can go
        indices = self.getIndices(currentCityIndex)

        distances = self.getCosts(indices)
        pharmones = self.getPharmones(indices)

        # determine how much we want to go where
        desires = []
        for i in range(len(distances)):
            distance = distances[i]
            pharmone = pharmones[i]

            desire = self.getDesire(distance, pharmone)
            desires.append(desire)

        # decide where we want to go

    # returns a path (the currentPath of the ant at the end of its traversal)
    def findPath(self, srcCityIndex):
        # find path
        pathNotFound = True
        while pathNotFound:
            break
        # update pharmones
        pass
