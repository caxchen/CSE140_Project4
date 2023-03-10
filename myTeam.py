from pacai.util import reflection
from pacai.agents.capture.capture import CaptureAgent
import random

class CustomOffensiveAgent(CaptureAgent):
    """
    A Dummy agent to serve as an example of the necessary agent structure.
    You should look at `pacai.core.baselineTeam` for more details about how to create an agent.
    """

    def __init__(self, index, isRed, **kwargs):
        super().__init__(index, **kwargs)
        self.index = index
        self.isPacman = True
        self.isRed = isRed

        # print(index)

    def registerInitialState(self, gameState):
        """
        This method handles the initial setup of the agent and populates useful fields,
        such as the team the agent is on and the `pacai.core.distanceCalculator.Distancer`.
        IMPORTANT: If this method runs for more than 15 seconds, your agent will time out.
        """
        super().registerInitialState(gameState)

        # Your initialization code goes here, if you need any. only runs at beginning

    def chooseAction(self, gameState):
        """
        Randomly pick an action.
        """
        # currentObservation=self.getCurrentObservation()
        # agentstate=currentObservation.getAgentState(0)
        
        # so gets foods from the state.  then, makes it so all the foods
        # in friendly territory are false, so BFS won't go for them.
        # then, returns BFS action to closest food.
        isPacman = gameState.getAgentState(self.index).isPacman()
        foods = gameState.getFood()
        length = foods.getWidth()
        # first, falsify friendly foods
        falsifyFrom = 0
        falsifyTo = length
        if (self.isRed):
            falsifyTo = length/2
        elif (not self.isRed):
            falsifyFrom = length/2
        for x in range(foods.getWidth()):
            if (x >= falsifyFrom and x <= falsifyTo):
                for y in range(foods.getHeight()):
                    foods[x][y] = False
        # then, return BFS action to closest food.
        if isPacman:
            return self.BFSClosestFood(gameState, self.index, foods)
            # return self.chooseAction_Pacman(gameState)
        else:
            return self.BFSClosestFood(gameState, self.index, foods)
            # return self.chooseAction_Ghost(gameState)

    def chooseAction_Pacman(self, gameState):
        actions = gameState.getLegalActions(self.index)
        return random.choice(actions)

    def chooseAction_Ghost(self, gameState):
        actions = gameState.getLegalActions(self.index)
        return random.choice(actions)
    
    def BFSClosestFood(self, state, agent, foods):
        node = state
        startingPosition = state.getAgentPosition(agent)
        if foods[startingPosition[0]][startingPosition[1]]:
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
                    if foods[neighbor[0]][neighbor[1]]:  # FOUND GOAL
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
    print("color", isRed)
    firstAgent = CustomOffensiveAgent(firstIndex, isRed)
    # secondAgent = TestAgent(secondIndex, isRed)
    # firstAgent = reflection.qualifiedImport(first)
    secondAgent = reflection.qualifiedImport(second)(secondIndex)

    return [firstAgent, secondAgent]
    """return [
        firstAgent(firstIndex),
        # firstAgent,
        secondAgent(secondIndex),
    ]"""

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
        if self.isPacman :
            return self.chooseAction_Pacman(gameState)
        else:
            return self.chooseAction_Ghost(gameState)

    def chooseAction_Pacman(self, gameState):
        actions = gameState.getLegalActions(self.index)
        return random.choice(actions)
    def chooseAction_Ghost(self, gameState):
        actions = gameState.getLegalActions(self.index)
        return random.choice(actions)
