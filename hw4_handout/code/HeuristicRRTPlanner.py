import numpy
from RRTTree import RRTTree
import geometric_planner as gp
import time

class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        #           key = index, value  =cost
        self.cost = dict()
        self.pmin = 0.3
    def Plan(self, start_config, goal_config, epsilon = 0.01):
        print('start arm plan')
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        start_time = time.time()
        self.cost[0] = 0
        opt_cost = self.planning_env.ComputeDistance_continue(start_config, goal_config)
                #prevent divid by 0
        max_cost = opt_cost + 0.3
        pr = 0.01
        mdist = 1
        self.planning_env.SetGoalParameters(goal_config, pr)
        while(1):           
            #import IPython
            #IPython.embed()
            #drop one valid random point that only connect with start tree
            point_drop = self.planning_env.GenerateRandomConfiguration()

            nearest_index, nearest_point = tree.GetNearestVertex(point_drop)
            point_drop = gp.replace(nearest_point, point_drop, mdist*(len(tree.vertices)/200+1))

            
            point_chosen_from_s = self.planning_env.Extend(tree.vertices[nearest_index], point_drop)
            point_chosen_from_g = self.planning_env.Extend(goal_config, point_drop)

            #Check whether the point_drop can be directly used for connecting both

            if(point_chosen_from_s == None or point_chosen_from_g == None): continue

            dist_s = self.planning_env.ComputeDistance_continue(point_chosen_from_s, point_drop)
            dist_g = self.planning_env.ComputeDistance_continue(point_chosen_from_g, point_drop)
            #print "dist_s {} dist_g {}".format(dist_s, dist_g)
            #find the path from start to end
            if (dist_s <= epsilon and dist_g <= epsilon):
                point_addon_s_final = tree.AddVertex(point_chosen_from_s)
                tree.AddEdge(nearest_index,point_addon_s_final)
                goal_id = tree.AddVertex(goal_config)
                tree.AddEdge(point_addon_s_final,goal_id)

                break

            #If point_drop can only connect to start tree or connect to goal point
            else:
                c_path  = self.cost[nearest_index] + self.planning_env.ComputeDistance_continue(point_chosen_from_s, nearest_point)
		              
		#          current path cost  + heuristic distance
                c_vertex = c_path + self.planning_env.ComputeDistance_continue(point_chosen_from_s, goal_config)
                #print('h_cost_in_arm',self.planning_env.ComputeDistance_continue(point_chosen_from_s, goal_config))
                m_q = 1- (c_vertex-opt_cost)/(max_cost-opt_cost)                
                p = max(m_q,self.pmin)
                #print "prob: {} c_vertex: {} vertex number: {}".format(p, c_vertex, len(tree.vertices))
                r = numpy.random.random_sample()
                if(r < p):
                    point_addon_s = tree.AddVertex(point_chosen_from_s)
                    tree.AddEdge(nearest_index,point_addon_s)
                    self.cost[point_addon_s] = c_path
                    max_cost = max(max_cost,c_vertex)
                    if(self.visualize):
                        self.planning_env.PlotEdge(nearest_point,point_chosen_from_s)
        # Find the path 
        total_time = time.time() - start_time
        path_start_tree = self.find_path(tree, 0, goal_id)
        path_start_tree.reverse()
        for i in path_start_tree:
            plan.append(i)
        plan.append(goal_config)

        dist_plan = self.planning_env.ComputePathLength(plan)
        print "total plan distance"
        print dist_plan

        print "total vertices in tree = "
        print len(tree.vertices)

        print "total plan time = "
        print total_time


        return plan



        plan.append(start_config)
        plan.append(goal_config)
        
        return plan
    def find_path(self, tree, start_id, end_id): #[start_id, end_id)
        id_next_v = end_id
        path = []
        while(id_next_v != start_id):
            id_next_v = tree.edges[id_next_v]
            path.append(tree.vertices[id_next_v])
        return path
