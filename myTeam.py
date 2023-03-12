from pacai.util import reflection
from pacai.agents.capture.capture import CaptureAgent
from pacai.core import distanceCalculator
import random


def createTeam(firstIndex, secondIndex, isRed,
        # first = 'pacai.agents.capture.dummy.DummyAgent',
        # second = 'pacai.agents.capture.dummy.DummyAgent'):
        first = 'pacai.agents.capture.offense.OffensiveReflexAgent',
        second = 'pacai.agents.capture.defense.DefensiveReflexAgent'):
    """
    This function should return a list of two agents that will form the capture team,
    initialized using firstIndex and secondIndex as their agent indexed.
    isRed is True if the red team is being created,
    and will be False if the blue team is being created.
    """
    # firstAgent = CustomOffensiveAgent(firstIndex, isRed, secondIndex)
    # secondAgent = CustomOffensiveAgent(secondIndex, isRed, firstIndex)
    firstAgent = reflection.qualifiedImport(first)(firstIndex)
    # secondAgent = reflection.qualifiedImport(second)(secondIndex)
    secondAgent = CustomDefensiveAgent(secondIndex, isRed, firstIndex)

    return [firstAgent, secondAgent]
    """return [
        firstAgent(firstIndex),
        # firstAgent,
        secondAgent(secondIndex),
    ]"""

# =====================================================
# DEFENSIVE AGENT
# =====================================================

# saved TestAgent code
class CustomDefensiveAgent(CaptureAgent):

    def __init__(self, index, isRed, teammate, **kwargs):
        super().__init__(index, **kwargs)
        self.isPacman = False
        self.isRed = isRed
        self.index = index
        self.teammate = teammate
        self.pairs = []

    def getTransitions(self, state):
        walls = state.getWalls()
        height = int(state.getInitialLayout().getHeight())
        width = state.getInitialLayout().getWidth()
        half = int(width / 2) - 1
        pairs = []
        for i in range(height):
            # (our half,  their half)
            if not walls[half][i] and not walls[half + 1][i]:
                if (self.isRed):
                    pairs.append(((int(half), int(i)), (int(half + 1), int(i))))
                else:
                    pairs.append(((int(half + 1), int(i)), (int(half), int(i))))
        return pairs

    def registerInitialState(self, gameState):
        """
        This method handles the initial setup of the agent and populates useful fields,
        such as the team the agent is on and the `pacai.core.distanceCalculator.Distancer`.
        IMPORTANT: If this method runs for more than 15 seconds, your agent will time out.
        """
        super().registerInitialState(gameState)

        # Your initialization code goes here, if you need any.
        self.pairs = self.getTransitions(gameState)

    def chooseAction(self, gameState):

        # currentObservation=self.getCurrentObservation()
        # agentstate=currentObservation.getAgentState(0)
        # print(agentstate.getPosition())

        # check if adjacent to eatable ghost. if so, eat it.
        for action in gameState.getLegalActions(self.index):
            successor = gameState.generateSuccessor(self.index, action)
            successorPos = successor.getAgentPosition(self.index)
            successorPos = (int(successorPos[0]), int(successorPos[1]))
            for enemy in self.getOpponents(gameState):
                epos = gameState.getAgentPosition(enemy)
                epos = (int(epos[0]), int(epos[1]))
                if epos == successorPos:
                    if (not self.inEnemyTerritory(gameState, epos)
                    or gameState.getAgentState(enemy).isScared()):
                        if (action in gameState.getLegalActions(self.index)):
                            return action

        enemyIndices = self.getOpponents(gameState)
        enemies = []
        for index in enemyIndices:
            enemies.append(gameState.getAgentState(index))
            """dist = self.distancer.getDistance(gameState.getAgentPosition(self.index), epos)
            if dist < closestEnemy[0]:
                closestEnemy = (dist, index)"""
        invaders = []
        outsideEnemies = []
        for enemy in enemies:
            if not self.inEnemyTerritory(gameState, enemy.getPosition()):
                invaders.append(enemy)
            else:
                outsideEnemies.append(enemy)
        if (len(invaders) > 0):
            closestEnemy = (float('inf'), None)
            for invader in invaders:
                dist = self.distancer.getDistance(gameState.getAgentPosition(self.index),
                invader.getPosition())
                if dist < closestEnemy[0]:
                    closestEnemy = (dist, invader.getPosition())
            # print("SEEKING ", closestEnemy[1])
            returnval = self.BFSSeek(gameState, closestEnemy[1])
            if (returnval in gameState.getLegalActions(self.index)):
                return returnval
        else:
            closestEnemy = (float('inf'), None)
            for enemy in outsideEnemies:
                dist = self.distancer.getDistance(gameState.getAgentPosition(self.index),
                enemy.getPosition())
                if dist < closestEnemy[0]:
                    closestEnemy = (dist, enemy.getPosition())
            closestPair = (float('inf'), None)
            for pair in self.pairs:
                dist = self.distancer.getDistance(closestEnemy[1], pair[1])
                if dist < closestPair[0]:
                    closestPair = (dist, pair)
            returnval = self.BFSSeek(gameState, closestPair[1][0])
            if (returnval in gameState.getLegalActions(self.index)):
                return returnval
        actions = gameState.getLegalActions(self.index)
        return random.choice(actions)

    def inEnemyTerritory(self, state, pos):
        width = state.getInitialLayout().getWidth()
        if (self.isRed):
            if (pos[0] > width / 2):
                return True
            else:
                return False
        elif not self.isRed:
            if (pos[0] < width / 2):
                return True
            else:
                return False

    def BFSSeek(self, state, pos):
        node = state
        startingPosition = state.getAgentPosition(self.index)
        if int(startingPosition[0]) == int(pos[0]) and int(startingPosition[1]) == int(pos[1]):
            return "Stop"
        frontier = [node]
        explored = []
        parents = dict()
        arrivalActions = dict()
        while len(frontier) > 0:
            node = frontier.pop()
            explored.append(node)
            for action in node.getLegalActions(self.index):
                neighbor = node.generateSuccessor(self.index, action).getAgentPosition(self.index)
                exploredPositions = [p.getAgentPosition(self.index) for p in explored]
                frontierPositions = [p.getAgentPosition(self.index) for p in frontier]
                if neighbor not in exploredPositions and neighbor not in frontierPositions:
                    if int(neighbor[0]) == int(pos[0]) and int(neighbor[1]) == int(pos[1]):  # GOAL
                        parents[neighbor] = node.getAgentPosition(self.index)
                        arrivalActions[neighbor] = action
                        lastAction = None
                        pathfinder = neighbor
                        pathlength = 0
                        while pathfinder != startingPosition:
                            lastAction = arrivalActions[pathfinder]
                            pathfinder = parents[pathfinder]
                            pathlength += 1
                        return lastAction
                    else:
                        frontier.insert(0, node.generateSuccessor(self.index, action))
                        parents[neighbor] = node.getAgentPosition(self.index)
                        arrivalActions[neighbor] = action
        return "Stop"


# =====================================================
# OFFENSIVE AGENT
# =====================================================

class CustomOffensiveAgent(CaptureAgent):

    def __init__(self, index, isRed, teammate, **kwargs):
        super().__init__(index, **kwargs)
        self.index = index
        self.isPacman = True
        self.isRed = isRed
        self.teammate = teammate
        self.distancer = None
        self.alphaBetaDistance = 4
        self.maxAlphaBetaDepth = 4
        self.foodTime = dict()
        self.enemyPositions = []

        # print(index)

    def registerInitialState(self, gameState):
        """
        This method handles the initial setup of the agent and populates useful fields,
        such as the team the agent is on and the `pacai.core.distanceCalculator.Distancer`.
        IMPORTANT: If this method runs for more than 15 seconds, your agent will time out.
        """
        super().registerInitialState(gameState)

        # Your initialization code goes here, if you need any. only runs at beginning
        self.distancer = distanceCalculator.Distancer(gameState.getInitialLayout())

    def chooseAction(self, gameState):
        """
        [UNDER CONSTRUCTION]
        Plan is to first detect distance to closest enemy with BFS.
        If the closest enemy is within a certain distance, use alpha-beta minimax.
        Otherwise, go for the closest food with BFS
        """

        # Detect closest enemy
        # First, get enemy positions
        friendPositions = []
        friendPositions.append(gameState.getAgentPosition(self.index))
        friendPositions.append(gameState.getAgentPosition(self.teammate))
        enemies = gameState.getAgentStates().copy()
        for enemy in enemies:
            if (enemy.getPosition() in friendPositions):
                enemies.remove(enemy)
        closestEnemy = (float('inf'), None)
        enemyIndices = self.getOpponents(gameState)
        for index in enemyIndices:
            epos = gameState.getAgentPosition(index)
            dist = self.distancer.getDistance(gameState.getAgentPosition(self.index), epos)
            if dist < closestEnemy[0]:
                closestEnemy = (dist, index)

        # check if adjacent to eatable ghost. if so, eat it.
        for action in gameState.getLegalActions(self.index):
            successor = gameState.generateSuccessor(self.index, action)
            successorPos = successor.getAgentPosition(self.index)
            for enemy in enemies:
                if enemy.getPosition() == successorPos:
                    if self.inEnemyTerritory(successor, enemy.getPosition()) or enemy.isScared():
                        if (action in gameState.getLegalActions(self.index)):
                            return action
                        else:
                            return random.choice(gameState.getLegalActions(self.index))

        returnval = self.BFSClosestFood(gameState, self.index)

        if (returnval not in gameState.getLegalActions(self.index)):
            return random.choice(gameState.getLegalActions(self.index))
        
        # prevent it from taking an action that would kill itself
        successor = gameState.generateSuccessor(self.index, returnval)
        dead = False
        deadAction = None
        opponents = self.getOpponents(successor)
        for opp in opponents:
            dist = self.distancer.getDistance(successor.getAgentPosition(self.index),
            successor.getAgentPosition(opp))
            if dist < 2 and not gameState.getAgentState(opp).isScared():
                dead = True
        if (dead):
            legalActions = gameState.getLegalActions(self.index).copy()
            if deadAction in legalActions:
                legalActions.remove(deadAction)
                if (len(legalActions) > 0):
                    return random.choice(legalActions)
                else:
                    return deadAction

        if (returnval in gameState.getLegalActions(self.index)):
            return returnval
        else:
            return random.choice(gameState.getLegalActions(self.index))

    def inEnemyTerritory(self, state, pos):
        width = state.getInitialLayout().getWidth()
        if (self.isRed):
            if (pos[0] > 2 * width / 3):
                return True
            else:
                return False
        elif not self.isRed:
            if (pos[0] < width / 3):
                return True
            else:
                return False

    def BFSClosestFood(self, state, agent):
        # print("FOOD: ", len(self.getFood(state)), "|", len(self.getFood(state)[0]))
        node = state
        startingPosition = state.getAgentPosition(agent)
        if self.getFood(state)[startingPosition[0]][startingPosition[1]]:
            # print(0)
            return "Stop"
        frontier = [node]
        explored = []
        parents = dict()
        arrivalActions = dict()
        while len(frontier) > 0:
            node = frontier.pop()
            explored.append(node)
            for action in node.getLegalActions(agent):
                neighbor = node.generateSuccessor(agent, action).getAgentPosition(agent)
                exploredPositions = [p.getAgentPosition(agent) for p in explored]
                frontierPositions = [p.getAgentPosition(agent) for p in frontier]
                if neighbor not in exploredPositions and neighbor not in frontierPositions:
                    if self.getFood(state)[neighbor[0]][neighbor[1]] or (neighbor[0],
                    neighbor[1]) in self.getCapsules(state):  # FOUND GOAL
                        parents[neighbor] = node.getAgentPosition(agent)
                        arrivalActions[neighbor] = action
                        lastAction = None
                        pathfinder = neighbor
                        pathlength = 0
                        while pathfinder != startingPosition:
                            lastAction = arrivalActions[pathfinder]
                            pathfinder = parents[pathfinder]
                            pathlength += 1
                        # print(pathlength)
                        return lastAction
                    else:
                        frontier.insert(0, node.generateSuccessor(agent, action))
                        parents[neighbor] = node.getAgentPosition(agent)
                        arrivalActions[neighbor] = action
                        # costs[neighbor] = costs[node]+1
        return "Stop"

# =====================================================
# TEST AGENT
# =====================================================

# saved TestAgent code
class TestAgent(CaptureAgent):
    """
    A Dummy agent to serve as an example of the necessary agent structure.
    You should look at `pacai.core.baselineTeam` for more details about how to create an agent.
    """

    def __init__(self, index, isRed, **kwargs):
        super().__init__(index, **kwargs)
        self.isPacman = False
        self.isRed = isRed

        print(index)

    def registerInitialState(self, gameState):
        """
        This method handles the initial setup of the agent and populates useful fields,
        such as the team the agent is on and the `pacai.core.distanceCalculator.Distancer`.
        IMPORTANT: If this method runs for more than 15 seconds, your agent will time out.
        """
        super().registerInitialState(gameState)

        # Your initialization code goes here, if you need any.

    def chooseAction(self, gameState):
        """
        Randomly pick an action.
        """
        # currentObservation=self.getCurrentObservation()
        # agentstate=currentObservation.getAgentState(0)
        # print(agentstate.getPosition())
        if self.isPacman:
            return self.chooseAction_Pacman(gameState)
        else:
            return self.chooseAction_Ghost(gameState)

    def chooseAction_Pacman(self, gameState):
        actions = gameState.getLegalActions(self.index)
        return random.choice(actions)

    def chooseAction_Ghost(self, gameState):
        actions = gameState.getLegalActions(self.index)
        return random.choice(actions)
