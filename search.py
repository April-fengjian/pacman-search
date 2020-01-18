# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    mark = []
    path = []
    fringe = util.Stack()

    #get the start state, make it to node and put into the fringe
    node = Node(problem.getStartState())
    fringe.push(node)
    state = node.getState()

    #mark the start state by put it into the mark list
    mark.append(state)
    nextnode = None
    parent = fringe.pop()

    #find the successors for the given state, and link the successors with their parent
    while (problem.isGoalState(state) != True):
        successor = problem.getSuccessors(state)
        for index in range(len(successor)):
            if(successor[index][0] not in mark):
                node = Node(successor[index][0],parent,successor[index][1])
                fringe.push(node)

        #mark the node when the node is pop from the stack
        nextnode = fringe.pop()
        parent = nextnode
        state = nextnode.getState()
        mark.append(state)

    #when finish searching, the nextnode is the goal,
    #Use the nextnode.getParent until get the start state can get a path from goal to start
    while(nextnode.getAction()!=None):
        path.append(nextnode.getAction())
        nextnode = nextnode.getParent()

    #reverse the path, then can get a correct path from start to goal
    return list(reversed(path))

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    mark = []
    path = []
    fringe = util.Queue()

    #get the start state, make it to node and put into the fringe
    node = Node(problem.getStartState())
    fringe.push(node)

    #mark the start state by put it into the mark list
    state = node.getState()
    mark.append(state)
    nextnode = None
    parent = fringe.pop()

    #find the successor of the given state
    #push the successor into the queue and mark it
    while (problem.isGoalState(state) != True):

        successor = problem.getSuccessors(state)
        for index in range(len(successor)):
            if(successor[index][0] not in mark):
                node = Node(successor[index][0],parent,successor[index][1])

                fringe.push(node)
                mark.append(successor[index][0])
        nextnode = fringe.pop()
        parent = nextnode
        state = nextnode.getState()

    #when finish searching, get the path from goal to start
    while(nextnode.getAction()!=None):
        path.append(nextnode.getAction())
        nextnode = nextnode.getParent()
    #reverse the path to get the correct path from start to goal
    return list(reversed(path))


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    mark = []
    path = []
    fringe = util.PriorityQueue()

    #get the start state, make it to node and put into the fringe
    node = Node(problem.getStartState())
    fringe.push(node,0)

    state = node.getState()
    mark.append(state)
    nextnode = None
    parent = fringe.pop()

    #find the successors of the given state
    while (problem.isGoalState(state) != True):
        successor = problem.getSuccessors(state)
        for index in range(len(successor)):
            #if it is a valid successor
            #make a node to store the information of its parent, state, action and cost
            if(successor[index][0] not in mark ):
                cost = parent.getCost()
                node = Node(successor[index][0],parent,successor[index][1],successor[index][2]+cost)

                #if the successor is the goal, use update() to keep the lowest cost of the goal node
                if(problem.isGoalState(successor[index][0]) == True):
                    fringe.update(node, successor[index][2]+cost)
                else:
                    fringe.push(node, successor[index][2]+cost)
                    mark.append(successor[index][0])
        nextnode = fringe.pop()
        parent = nextnode
        state = nextnode.getState()

    #getting the path
    while(nextnode.getAction()!=None):
        path.append(nextnode.getAction())
        nextnode = nextnode.getParent()

    return list(reversed(path))

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    mark = []
    path = []
    fringe = util.PriorityQueue()

    #get the start state, make it to node and put into the fringe
    node = Node(problem.getStartState())
    fringe.push(node,heuristic(problem.getStartState(),problem))

    state = node.getState()
    mark.append(state)
    nextnode = None
    parent = fringe.pop()

    #get the successor
    while (problem.isGoalState(state) != True):
        successor = problem.getSuccessors(state)
        for index in range(len(successor)):
            if(successor[index][0] not in mark ):
                cost = parent.getCost()
                node = Node(successor[index][0],parent,successor[index][1],successor[index][2]+cost)
                #successor[index][2]+cost is the actual cost for move
                #The newCost is calculate for the fringe to choose where to move
                newCost = successor[index][2]+cost+ heuristic(successor[index][0],problem)
                if(problem.isGoalState(successor[index][0]) == True):
                    fringe.update(node, newCost)
                else:
                    fringe.push(node, newCost)
                    mark.append(successor[index][0])

        nextnode = fringe.pop()
        parent = nextnode
        state = nextnode.getState()

    #get the path
    while(nextnode.getAction()!=None):
        path.append(nextnode.getAction())
        nextnode = nextnode.getParent()

    return list(reversed(path))

#This class is uesed to create a node in order to store more inforation
class Node:
    def __init__(self, state, parent = None, action = None,cost = 0):
        self.parent = parent
        self.state = state
        self.action = action
        self.cost = cost
    def getParent(self):
        return self.parent
    def setChild(self, node):
        self.child = node
    def getState(self):
        return self.state
    def getAction(self):
        return self.action
    def getCost(self):
        return self.cost


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
