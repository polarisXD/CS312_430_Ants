class Colony:
    distanceMatrix = None
    phermoneMatrix = None
    ants = [] # list of Ant objects
    bestPathSoFar = []

    def __init__(self):
        pass
    
    # returns nothing
    def initializeMatrices(self):
        pass

    # return best Approximate solution
    def releaseTheAnts(self, numAnts):
        for i in range(len(ants)):
            break
        
        pass

class Ant(Colony):
    currentPath = [] # list of tuples (x and y coordinates)
    totalPathCOst = 0

    def __init__(self):
        super() # allows us to access the Colony matrices by inheriting from it
        
        
    def getIndices(self, srcIndex, destIndex):
        pass
    def getCosts(self, indexTuples):
        pass
    def getPharmones(self, indexTuples):
        pass

    def getDesire(self, cost, pharmone, time):
        a = 1
        b = 1

        inverseCost = 1 / cost
        total = cost + pharmone

        desire = ((pharmone^a) * (inverseCost^b)) / total

        return desire

    def moveToNext(self, currentCityIndex):

        # see where we can go
        indices = self.getIndices(currentCityIndex)

        costs = self.getCosts(indices)
        pharmones = self.getPharmones(indices)
        times = self.getTimes(indices)

        # determine how much we want to go where
        desires = []
        for i in range(len(costs)):
            cost = costs[i]
            pharmone = pharmones[i]
            time = times[i]

            desire = self.getDesire(cost, pharmone, time)
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
