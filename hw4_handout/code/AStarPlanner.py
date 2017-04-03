import sets
import collections
import IPython

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def heuristic (self, start_id, end_id, multiple = 5):
        cost = self.planning_env.ComputeHeuristicCost(start_id, end_id)
        cost = cost * multiple
        return cost

    def distance (self, start_id, goal_id):
        dist = self.planning_env.ComputeDistance(start_id, goal_id)
        return dist
    
    def getConfig (self, iid):
        config = self.planning_env.discrete_env.NodeIdToConfiguration(iid)
        return config

    def getLowestId (self, openList, costs): 
        idx = 1
        for iid in openList:
            if idx == 1:
                lowestId = iid
                idx += 1
            elif costs[iid] < costs[lowestId]:
              lowestId  = iid
        return lowestId

    def Plan(self, start_config, goal_config):

        plan = []
        planAction = []
        
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        # Open /closed list of nodes / ids
        openList = collections.deque([start_id])
        closedList = collections.deque()
        # Dictionary of edges
        edges = {}
        actions = {}
        # evaluation cost
        ecosts = {start_id : 0}
        # operating cost
        ocosts = {start_id : 0}

        curr_id = start_id
        while openList:
            # Get lowest evaluation cost ID
            curr_id = self.getLowestId (openList, ecosts)
            # Remove the best curr_id from open, add to closed
            openList.remove(curr_id)
            closedList.append(curr_id)
            # Get neighbors
            neighbors = self.planning_env.GetSuccessors(curr_id)

            # early exit
            if curr_id == goal_id:
              break

            for neighbor in neighbors:
                neighborID = neighbor[0]
                neighborAction = neighbor[1]
                #if not self.planning_env.no_collision(self.planning_env.discrete_env.NodeIdToConfiguration(neighborID)):
                    #IPython.embed()
                # If neighborID is not visited yet; otherwise, move to next neighborID
                if neighborID not in closedList:
                    # If neighborID isn't in openlist, add it, precompute evaluation function, operating cost, connect to graph
                    if neighborID not in openList:                        
                        
                        ocosts[neighborID] = self.distance(curr_id, neighborID) + ocosts[curr_id]
                        ecosts[neighborID] = ocosts[neighborID] + self.heuristic(neighborID, goal_id)
                        openList.append(neighborID)
                        edges[neighborID] = curr_id
                        actions[neighborID] = neighborAction # corresponding action
                        
                        # plot edge
                        if self.visualize:
                            self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(curr_id),\
                                                       self.planning_env.discrete_env.NodeIdToConfiguration(neighborID), 'k')

                    # if neighbor is in open list, then see if it is closer to beginning now than before
                    else:
                        currcost = ocosts[neighborID]
                        updatecost = ocosts[curr_id] + self.distance(curr_id, neighborID)
                        if updatecost < currcost:
                            edges[neighborID] = curr_id
                            actions[neighborID] = neighborAction # corresponding action

                     
        plan_id = goal_id
        print "nodes expanded: %d" % len(edges)
        while plan_id != start_id:
            config = self.getConfig(plan_id)
            plan.append(config)
            planAction.append(actions[plan_id])
            plan_id = edges[plan_id]
             
            
        plan.append(start_config)
        plan = plan[::-1]
        planAction = planAction[::-1]
        plan.append(goal_config)
        self.planning_env.herb.SetCurrentConfiguration(start_config)
        return planAction
