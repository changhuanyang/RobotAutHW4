from Queue import PriorityQueue
import time
import numpy

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()


    def Plan(self, start_config, goal_config):

        plan = []
        start_time = time.time()

        if( self.visualize and hasattr(self.planning_env, 'InitializePlot')):
            self.planning_env.InitializePlot(goal_config)

        #a_map = PriorityQueue()
        a_map = [];
        
        action_traj = {}
        trajectory = {}
        overall_cost = {}
        total_vertices = 1

        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        trajectory[start_id] = None
        #there is no action exeacute, to make robot move to start config
        action_traj[start_id] = None
        
        overall_cost[start_id] = 0
        path_found = False

        #a_map.put(start)
        a_map.append([start_id,0])
        #Herutist weight
        k = 1

        #while not a_map.empty():
        while len(a_map):
            #current_node = a_map.get()
            current_node = a_map.pop(0)[0]
            current_node_config =  self.planning_env.discrete_env.NodeIdToConfiguration(current_node)       
            #print "pop_id = ", current_node
            #if(total_vertices%2000 == 0):
            #    print "vertices = ", total_vertices
            
            if current_node == goal_id:
                path_found = True
                if(self.visualize):
                        n_node_config = self.planning_env.discrete_env.NodeIdToConfiguration(neighbour)
                        self.planning_env.PlotEdge(current_node_config,goal_config)
                break


            Successors = self.planning_env.GetSuccessors(current_node)
            #neibhbour type = [n_id,action]
            for neighbour in Successors:
                n_id = neighbour[0]
                n_action = neighbour[1]

                n_config = self.planning_env.discrete_env.NodeIdToConfiguration(n_id)
                
                new_cost = overall_cost[current_node] + self.planning_env.ComputeDistance(current_node, n_id)
                #check this_id is not visited or the cost less than last time visted
                if (n_id not in overall_cost or new_cost < overall_cost[n_id]) :
                    overall_cost[n_id] = new_cost
                    H_cost = k*self.planning_env.ComputeHeuristicCost(goal_id, n_id)
                    print('hcost=',H_cost)
                    priority_cost = new_cost + H_cost
                    #a_map.put(neighbour, priority_cost)
                    a_map.append([n_id,priority_cost])
                    trajectory[n_id] = current_node
                    action_traj[n_id] = n_action
                    total_vertices += 1
                    if(self.visualize):
                        n_node_config = self.planning_env.discrete_env.NodeIdToConfiguration(n_id)
                        self.planning_env.PlotEdge(current_node_config,n_node_config)
                    # self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(current_node),self.planning_env.discrete_env.NodeIdToConfiguration(neighbour))
                    # print "cost: " cost_so_far[next],"came from: "came_from[next] 
            a_map.sort(key=lambda x: x[1])
            #print('len of map =',len(a_map))
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
                
        current_id = goal_id
        plan = []
        plan_action = []
        if path_found :
            print "Path found"
            while(trajectory[current_id] != start_id):
                plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(current_id))
                plan_action.append(action_traj[current_id])
                if(self.visualize):
                    self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(current_id),self.planning_env.discrete_env.NodeIdToConfiguration(trajectory[current_id]))
                current_id = trajectory[current_id]
        else:
            print "No path found"
        
        #plan[0] = goal_config
        #plan.append(start_config)
        #inverse plan
        plan = plan[::-1]
        plan_action = plan_action[::-1]
        #print for time, path length and total tree vertices
        total_time = time.time() - start_time
        print "total plan time = ",total_time
        
        print "total path length",self.planning_env.ComputePathLength(plan)

        print "total visited vertices", total_vertices



        return plan_action
