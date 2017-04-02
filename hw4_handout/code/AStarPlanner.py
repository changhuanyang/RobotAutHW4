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
        
        trajectory = {}
        overall_cost = {}
        total_vertices = 1

        start = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        end = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        trajectory[start] = None
        overall_cost[start] = 0
        path_found = False

        #a_map.put(start)
        a_map.append([start,0])
        #Herutist weight
        k = 5

        #while not a_map.empty():
        while len(a_map):
            #current_node = a_map.get()
            current_node = a_map.pop(0)[0]
            current_node_config =  self.planning_env.discrete_env.NodeIdToConfiguration(current_node)       
            #print "pop_id = ", current_node
            if(total_vertices%2000 == 0):
                print "vertices = ", total_vertices
            if current_node == end:
                path_found = True
                trajectory[end] = current_node
                if(self.visualize):
                        n_node_config = self.planning_env.discrete_env.NodeIdToConfiguration(neighbour)
                        self.planning_env.PlotEdge(current_node_config,goal_config)
                break

            for neighbour in self.planning_env.GetSuccessors(current_node):
                n_config = self.planning_env.discrete_env.NodeIdToConfiguration(neighbour)
                #if(self.planning_env.no_collision(n_config)):
                #    new_cost = overall_cost[current_node] + self.planning_env.ComputeDistance(current_node, neighbour)
                #else:
                #    new_cost = float("inf")
                new_cost = overall_cost[current_node] + self.planning_env.ComputeDistance(current_node, neighbour)
                if (neighbour not in overall_cost or new_cost < overall_cost[neighbour]) :
                    overall_cost[neighbour] = new_cost
                    H_cost = k*self.planning_env.ComputeHeuristicCost(end, neighbour)
                    priority_cost = new_cost + H_cost
                    #a_map.put(neighbour, priority_cost)
                    a_map.append([neighbour,priority_cost])
                    trajectory[neighbour] = current_node
                    total_vertices += 1
                    if(self.visualize):
                        n_node_config = self.planning_env.discrete_env.NodeIdToConfiguration(neighbour)
                        self.planning_env.PlotEdge(current_node_config,n_node_config)
                    # self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(current_node),self.planning_env.discrete_env.NodeIdToConfiguration(neighbour))
                    # print "cost: " cost_so_far[next],"came from: "came_from[next] 
            a_map.sort(key=lambda x: x[1])
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
                
        current_id = end
        plan = []
        if path_found :
            print "Path found"
            while(trajectory[current_id] != start):
                if current_id == end :
                    plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(end))
                else:
                    plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(current_id))    
                if(self.visualize):
                    self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(current_id),self.planning_env.discrete_env.NodeIdToConfiguration(trajectory[current_id]))
                current_id = trajectory[current_id]
        else:
            print "No path found"
        
        #plan[0] = goal_config
        plan.append(start_config)
        plan = plan[::-1]
        #print for time, path length and total tree vertices
        total_time = time.time() - start_time
        print "total plan time = ",total_time
        
        print "total path length",self.planning_env.ComputePathLength(plan)

        print "total visited vertices", total_vertices



        return plan
