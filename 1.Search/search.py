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
    """

    actions = []
    pred = dict()
    visited = dict()

    s = problem.getStartState()
    s1 = (s,'',1)
    q = util.Stack()
    q.push(s1)
    last = None

    while(not q.isEmpty() and last == None):

        c = q.pop()
        visited[c[0]] = True

        for su in problem.getSuccessors(c[0]):
            if su[0] in visited: continue
            pred[su[0]] = c
            q.push(su)
            # Is goal
            if problem.isGoalState(su[0]):
                last = su

    # Return empty
    if last == None: return []

    # Reverse actions list
    while last is not s1:
        actions.append(last[1])
        last = pred[last[0]]

    actions.reverse()
    return actions
    #util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    pred, expanded = dict(), dict()
    s = problem.getStartState()
    s1 = (s,'',1)

    q = util.Queue()
    q.push(s1)
    last = None

    while(not q.isEmpty() and last == None):
        c = q.pop()
        if c[0] in expanded: continue

        expanded[c[0]] = True
        for su in problem.getSuccessors(c[0]):
            if su[0] in pred: continue
            pred[su[0]] = c
            q.push(su)
            # Is goal
            if problem.isGoalState(su[0]):
                last = su

    # Return empty
    if last == None: return []

    # Reverse actions list
    actions = []
    while last is not s1:
        actions.append(last[1])
        last = pred[last[0]]
        print last

    print actions
    actions.reverse()
    return actions

    #util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    pred, expanded, cost = dict(), dict(), dict()
    s = problem.getStartState()
    s1 = (s,'None')

    q = util.PriorityQueue()
    cost[s] = 0
    q.push(s1, 0)
    last = None

    while(not q.isEmpty() and last == None):
        state, action = q.pop()
        if state in expanded: continue
        expanded[state] = True
        # Is goal
        if problem.isGoalState(state):
            last = (state, action)
        else:
            for n_state, n_action, n_cost in problem.getSuccessors(state):
                if n_state in expanded: continue
                if n_state in cost and cost[n_state] <= n_cost+cost[state]: continue
                cost[n_state] = n_cost + cost[state]
                pred[n_state] = (state, action)
                q.push((n_state, n_action), cost[n_state])

    # Return empty
    if last == None: return []

    # Create actions list
    actions = []
    while last != s1:
        actions.append(last[1])
        last = pred[last[0]]
        print actions
    actions.reverse()

    return actions
    #util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    pred, expanded, cost = dict(), dict(), dict()
    s = problem.getStartState()
    s1 = (s,'None')

    q = util.PriorityQueue()
    cost[s] = 0
    q.push(s1, 0)
    last = None

    while(not q.isEmpty() and last == None):
        state, action = q.pop()
        if state in expanded: continue
        expanded[state] = True
        # Is goal
        if problem.isGoalState(state):
            last = (state, action)
        else:
            for n_state, n_action, n_cost in problem.getSuccessors(state):
                if n_state in expanded: continue
                if n_state in cost and cost[n_state] <= n_cost+cost[state]: continue
                cost[n_state] = n_cost + cost[state]
                pred[n_state] = (state, action)
                h = heuristic(n_state, problem)
                q.push((n_state, n_action), cost[n_state] + h)

    # Return empty
    if last == None: return []

    # Create actions list
    actions = []
    while last != s1:
        actions.append(last[1])
        last = pred[last[0]]
        print actions
    actions.reverse()

    return actions


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
