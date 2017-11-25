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

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    #   Dictionary to hold a state as a key and the path that led to it
    actions = {}
    #   A stack to hold the nodes being in the frontier waiting to be explored
    frontier = util.Stack()
    frontier.push(problem.getStartState())
    #   A set to hold the explored nodes through the algorithm
    explored = set()
    actions[problem.getStartState()] = []
    while not frontier.isEmpty():
        #   Remove the node being investigated and add it to explored list
        state = frontier.pop()
        explored.add(state)
        #   Check if the node is a goal state
        if problem.isGoalState(state):
            return actions[state]
        # Generate state successors
        for s in problem.getSuccessors(state):
            if s[0] not in explored:
                frontier.push(s[0])
                # Update the path to successor s
                actions[s[0]] = actions[state] + [s[1]]
    return actions[state]





def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    #   Dictionary to hold a state as a key and the path that led to it
    actions = {}
    #   A queue to hold the nodes being in the frontier waiting to be explored
    frontier = util.Queue()
    frontier.push(problem.getStartState())
    #   A set to hold the explored nodes through the algorithm
    explored = set()
    actions[problem.getStartState()] = []
    explored.add(problem.getStartState())
    while not frontier.isEmpty():
        #   Remove the node being investigated
        state = frontier.pop()
        #   Check if the node is a goal state
        if problem.isGoalState(state):
            return actions[state]
        #   Generate state successors
        for s in problem.getSuccessors(state):
            if s[0] not in explored:
                frontier.push(s[0])
                #   Update the path to successor s and add it to explored set
                actions[s[0]] = actions[state] + [s[1]]
                explored.add(s[0])
    return state[1]

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    #   Dictionary to hold a state as a key and the path that led to it with the cost
    actions = {}
    #   A priority queue to hold the nodes being in the frontier waiting to be explored
    frontier = util.PriorityQueue()
    frontier.push(problem.getStartState(), 0)
    #   A set to hold the explored nodes through the algorithm
    explored = set()
    explored.add(problem.getStartState())
    actions[problem.getStartState()] = ([], 0)
    while not frontier.isEmpty():
        #   Remove the node being investigated
        state = frontier.pop()
        #   Check if the node is a goal state
        if problem.isGoalState(state):
            return actions[state][0]
        #   Generate state successors
        for s in problem.getSuccessors(state):
            #   Check that node is not in the explored set nor in the queue
            if s[0] not in explored and s[0] not in [x[2] for x in frontier.heap]:
                #   Compute priority as the cost from the parent state plus the cost of the successor
                priority = actions[state][1] + s[2]
                frontier.push(s[0], priority)
                #   Update the path and cost to successor s and add it to explored set
                actions[s[0]] = (actions[state][0] + [s[1]], priority)
                explored.add(s[0])
            #   Check if the node is already in the queue
            elif s[0] in [x[2] for x in frontier.heap]:
                frontier.update(s[0], actions[state][1] + s[2])
                #   Compute priority as the cost from the parent state plus the cost of the successor
                priority = actions[state][1] + s[2]
                #   Update the path and cost if the new cost is smaller
                if actions[s[0]][1] > priority:
                    actions[s[0]] = (actions[state][0] + [s[1]], priority)
                explored.add(s[0])
    return actions[state][0]

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    #   Dictionary to hold a state as a key and the path that led to it with the cost
    actions = {}
    #   A priority queue to hold the nodes being in the frontier waiting to be explored
    frontier = util.PriorityQueue()
    #   Compute the heuristic function for start state
    h = heuristic(problem.getStartState(), problem)
    frontier.push(problem.getStartState(), h)
    #   A set to hold the explored nodes through the algorithm
    explored = set()
    explored.add(problem.getStartState())
    actions[problem.getStartState()] = ([], 0)
    while not frontier.isEmpty():
        #   Remove the node being investigated
        state = frontier.pop()
        #   Check if the node is a goal state
        if problem.isGoalState(state):
            return actions[state][0]
        #   Generate state successors
        for s in problem.getSuccessors(state):
            #   Compute heuristic for the successor state
            h = heuristic(s[0], problem)
            cost = actions[state][1] + s[2]
            #   Check that node is not in the explored set nor in the queue
            if s[0] not in explored and s[0] not in [x[2] for x in frontier.heap]:
                #   Push successor with f = g + h as priority
                frontier.push(s[0], cost + h)
                actions[s[0]] = (actions[state][0] + [s[1]], cost)
                explored.add(s[0])
            #   Check if the node is already in the queue
            elif s[0] in [x[2] for x in frontier.heap]:
                #   Update successor with f = g + h as priority
                frontier.update(s[0], cost + h)
                #   Update the path if the new cost is smaller
                if actions[s[0]][1] > cost:
                    actions[s[0]] = (actions[state][0] + [s[1]], cost)
                explored.add(s[0])
    return actions[state][0]


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
