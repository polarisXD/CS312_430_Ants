import numpy as np
import heapq as hq
import time

class sstNode:
    def __init__(self):
        self.index = None # the row it came from (the index on the graph)
        self.parentIndex = None
        # its children can be computed from the non-inf values on that row
        self.cost = float('inf')
        self.priorityCost = float('inf') # includes the depth of the tree
        self.pqid = None
        self.previouslyCalledTuples = []
        self.matrix = None

    def __lt__(self, other): # <
        if len(self.previouslyCalledTuples) > len(other.previouslyCalledTuples):
            return False
        else:
            return True
    def __le__(self, other): # <=
        if len(self.previouslyCalledTuples) > len(other.previouslyCalledTuples):
            return False
        else:
            return True

    def __eq__(self, other):
        if len(self.previouslyCalledTuples) > len(other.previouslyCalledTuples):
            return False
        else:
            return True
    def __ne__(self, other):
        if len(self.previouslyCalledTuples) > len(other.previouslyCalledTuples):
            return False
        else:
            return True
    def __gt__(self, other):
        if len(self.previouslyCalledTuples) > len(other.previouslyCalledTuples):
            return True
        else:
            return False
    def __ge__(self, other):
        if len(self.previouslyCalledTuples) > len(other.previouslyCalledTuples):
            return True
        else:
            return False


    def getParent(self):
        return self.parent

    def getIndex(self):
        return self.index

class TSP:
    def __init__(self):
        self.costMatrix = None
        self.minimumCost = float('inf')
        self.upperBound = float('inf')
        self.cities = None
        self.visitedLocations = None # a list of cities we have been to
        self.priorityQueue = [] # contains sstNodes
        self.solutions = {} # maps a list of cities to a cost
        self.pqidCount = 0
        self.optimumFound = False
        self.numSolutions = 0
        self.totalStates = 0
        self.maxQueueSize = 0
        self.numPruned = 0
        self.path = []
        self.time = None

    def getStarter(self, cities):
        # find the node with the most outgoing edges (with the most possibility to be pruned)
        # O(n^2) because we look through every node, and at every node, we look through every other node
        self.cities = cities
        self.costMatrix = np.ndarray((len(self.cities),len(self.cities)))

        numOutToCity = {}
        largestNumOut = 0
        i = 0
        for startCity in self.cities:
            numOutgoing = 0
            j = 0
            for endCity in self.cities:
                cost = startCity.costTo(endCity)
                if cost != float('inf'):
                    numOutgoing = numOutgoing + 1
                self.costMatrix[i,j] = cost
                j = j + 1
            if numOutgoing >= largestNumOut:
                numOutToCity[numOutgoing] = [startCity, i]
                largestNumout = numOutgoing
            i = i + 1

        # get the maximum node
        maximum = max(list(numOutToCity.keys()))

        # save that as the starter node
        starterNode = sstNode()
        starterNode.index = numOutToCity[maximum][1]
        starterNode.pqid = self.pqidCount
        self.pqidCount = self.pqidCount + 1
        self.totalStates = self.totalStates + 1

        # return the starter node
        return starterNode

    def getMins(self, matrix):
        # find the minimum of every row
        horizontalMins = []
        i = 0
        while i < matrix.shape[0]:
            minimum = np.min(matrix[i,:])
            if minimum == float('inf'):
                horizontalMins.append(0)
            else:
                horizontalMins.append(minimum)
            i = i + 1

        # subtract the minimum from every row
        i = 0
        for min in horizontalMins:
            matrix[i,:] -= min
            i = i + 1

        # find the minimum of every column
        verticalMins = []
        j = 0
        while j < matrix.shape[1]:
            minimum = np.min(matrix[:,j])
            if minimum == float('inf'):
                verticalMins.append(0)
            else:
                verticalMins.append(minimum)
            j = j + 1

        # subtract the minimum from every column
        j = 0
        for min in verticalMins:
            matrix[:,j] -= min
            j = j + 1

        reductionCost = np.sum(horizontalMins + verticalMins)
        if reductionCost == float('inf'):
            reductionCost = 0

        return reductionCost, matrix

    # initialize the Tree (including setting up the priority queue
    # assuming that nodes in inputGraph are id'd by number, arbitrarily
    def makeStateSpaceTree(self, cities): # or "initializePriorityQueue(inputGraph)

        # grab the node with the largest degree, make the initial cost matrix
        starterNode = self.getStarter(cities)

        # reduce the cost matrix
        self.minimumCost, self.costMatrix = self.getMins(self.costMatrix)
        starterNode.cost = self.minimumCost
        starterNode.priorityCost = starterNode.cost
        starterNode.matrix = self.costMatrix

        # add the new node to the queue
        hq.heappush(self.priorityQueue, (starterNode.priorityCost,starterNode))
        self.maxQueueSize = 1

    def makeNewNode(self, childIndex, parentNode):
        newNode = sstNode()
        newNode.index = childIndex
        newNode.pqid = self.pqidCount
        self.pqidCount = self.pqidCount + 1
        newNode.parentIndex = parentNode.index
        newNode.previouslyCalledTuples = parentNode.previouslyCalledTuples.copy()
        newNode.previouslyCalledTuples.append((parentNode.index, newNode.index))
        self.totalStates = self.totalStates + 1

        return newNode

    def getSpecificMatrix(self, sstChild, sstParent):
        matrix = sstParent.matrix.copy()

        # cross out row of parent, column of child
        parentIndex, childIndex = sstChild.previouslyCalledTuples[-1]
        rootIndex = sstChild.previouslyCalledTuples[0][0]

        matrix[parentIndex,:] = float('inf')
        matrix[:,childIndex] = float('inf')

        # don't return back to the root index unless this is the last city before returning
        if len(sstChild.previouslyCalledTuples) != (len(self.cities) - 1):
            matrix[childIndex,rootIndex] = float('inf')

        return matrix


    # expanding a node takes its children and adds them to the priority queue
    def expandNode(self, sstNode):
        # Get the children of the sstNode
        childrenIndices = []
        childrenCosts = sstNode.matrix[sstNode.index,:]
        i = 0
        for childCost in childrenCosts:
            if childCost != float('inf'):
                childrenIndices.append(i)
            i = i + 1

        # If we cannot go anywhere, return a cost of sstNode.cost, and update the maximum, check to see if it is minimum
        if len(childrenIndices) == 0:
            finalCost = sstNode.cost
            if len(sstNode.previouslyCalledTuples) == len(self.cities):

                if finalCost < self.upperBound:
                    if self.upperBound != float('inf'):
                        self.numSolutions = self.numSolutions + 1
                    self.upperBound = finalCost # effectively prunes the tree
                    # Update the path
                    self.path = []
                    # grab the first city in every path
                    for tuple in sstNode.previouslyCalledTuples:
                        self.path.append(self.cities[tuple[0]])
                    # append the last visited place
                    # self.path.append(self.cities[sstNode.previouslyCalledTuples[-1][1]])
                    if finalCost == self.minimumCost:
                        self.optimumFound = True
            # else: # we are at a dead end; do nothing.


        minChildCost = float('inf')
        minChildIndex = 0
        for childIndex in childrenIndices:

            newNode = self.makeNewNode(childIndex, sstNode)

            # make a specific matrix for newNode
            childMatrix = self.getSpecificMatrix(newNode, sstNode)

            # reduce that specific matrix
            cost, childMatrix = self.getMins(childMatrix)


            # store the cost and matrix
            if sstNode.matrix[sstNode.index,newNode.index] != float('inf'):
                newNode.cost = cost + sstNode.matrix[sstNode.index,newNode.index] + sstNode.cost
                if len(newNode.previouslyCalledTuples) != 0:
                    newNode.priorityCost = newNode.cost / len(newNode.previouslyCalledTuples)
                else:
                    newNode.priorityCost = newNode.cost
            else:
                newNode.cost = cost + sstNode.cost
                newNode.priorityCost = newNode.cost / len(newNode.previouslyCalledTuples)
            newNode.matrix = childMatrix

            # add to priority queue
            if newNode.cost < self.upperBound:
                hq.heappush(self.priorityQueue, (newNode.priorityCost, newNode))
                if len(self.priorityQueue) > self.maxQueueSize:
                    self.maxQueueSize = len(self.priorityQueue)
            else:
                self.numPruned = self.numPruned + 1

            if newNode.cost < minChildCost:
                minChildCost = newNode.cost
                minChildIndex = newNode.index

        return minChildCost, minChildIndex


    # chew up the priority queue by expanding children nodes
    def chewPriorityQueue(self, allowedTime):
        startTime = time.time()
        while (not self.optimumFound) and (time.time() - startTime < allowedTime):
            if len(self.priorityQueue) == 0:
                break
            node = hq.heappop(self.priorityQueue)[1]
            if node.cost < self.upperBound: # effectively prunes the tree
                # check to see if we hit the end yet
                if self.optimumFound != True:
                    self.expandNode(node)
                else:
                    self.numPruned = self.numPruned + len(self.priorityQueue)
                    break
            else:
                self.numPruned = self.numPruned + 1
            self.time = time.time() - startTime

