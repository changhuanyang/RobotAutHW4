import numpy as np
import operator
import IPython
from RRTTree import RRTTree
import geometric_planner as gp

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.2):
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        self.planning_env.SetGoalParameters(goal_config, p = 0.05)
        direct_to_goal = False
        mdist = 1 #epsilon*500
        error = 0.1 #1e-12
        goalError = 0.5
        while True:
            addNewNode = False
            rand_config = self.planning_env.GenerateRandomConfiguration()
            sid1, f_nearest_config = ftree.GetNearestVertex(rand_config)
            #rand_config = gp.replace(f_nearest_config, rand_config, mdist)
            #new1_config = self.planning_env.Extend(f_nearest_config, final_config, epsilon)
            f_new_config = self.planning_env.Extend(f_nearest_config, rand_config, epsilon)
            print "ftree new node error: {} len: {}".format(self.planning_env.ComputeDistance(f_nearest_config, f_new_config), len(ftree.vertices)) 
            if (self.planning_env.ComputeDistance(f_nearest_config, f_new_config) > error):
                addNewNode = True
                eid1 = ftree.AddVertex(f_new_config)
                ftree.AddEdge(sid1, eid1)
                if (self.planning_env.ComputeDistance(f_new_config, goal_config) < goalError):
                    direct_to_goal = True
                    print "Direct to Goal"
                    break
            else:
                eid1 = sid1
            # check from other set
            sid2, r_nearest_config = rtree.GetNearestVertex(rand_config)
            #new2_config = gp.replace(nearest_config, rand_config, mdist)
            r_new_config = self.planning_env.Extend(r_nearest_config, rand_config, epsilon)
            print "rtree new node error: {} len: {}".format(self.planning_env.ComputeDistance(r_nearest_config, r_new_config), len(rtree.vertices)) 
            if (self.planning_env.ComputeDistance(r_new_config, r_nearest_config) > error):
                addNewNode = True
                eid2 = rtree.AddVertex(r_new_config)
                rtree.AddEdge(sid2, eid2)
            else:
                eid2 = sid2

            if(self.planning_env.ComputeDistance(r_new_config, f_new_config) < goalError):
                eid2 = rtree.AddVertex(r_new_config)
                rtree.AddEdge(sid2, eid2) 
                break
            #sid2, r_nearest_config = rtree.GetNearestVertex(f_new_config)
            min = 10000
            if addNewNode:
                for vertex in ftree.vertices:
                    sid2, r_nearest_config = rtree.GetNearestVertex(vertex)
               
                    if min > self.planning_env.ComputeDistance(r_nearest_config, vertex):
                        min = self.planning_env.ComputeDistance(r_nearest_config, vertex)
                        
                    if(self.planning_env.ComputeDistance(r_nearest_config, vertex) < goalError):
                        eid2 = sid2
                        print "Connect Both Tree"
                        break
                print "min distance: {}".format(min)

            #print "new node: {} connect: {}".format(f_new_config, self.planning_env.ComputeDistance(f_new_config, r_nearest_config))

            #rtree, ftree = ftree, rtree
            

        #make sure ftree is always the start config set
        #if (self.planning_env.ComputeDistance(ftree.vertices[ftree.GetRootId()], goal_config) < goalError):
        #    rtree, ftree = ftree, rtree
        #    eid2, eid1 = eid1, eid2

        if not direct_to_goal:
            iid = rtree.GetRootId()
            fid = rtree.edges[eid2]
            cid = fid 
            while cid != iid:
                plan.append(np.copy(rtree.vertices[cid]))
                cid = rtree.edges[cid]
        plan.append(goal_config)
        plan.reverse()

        iid = ftree.GetRootId()
        fid = eid1
        cid = fid
        while cid != iid:
            plan.append(np.copy(ftree.vertices[cid]))
            cid = ftree.edges[cid]

        plan.append(start_config)

        plan.reverse()
        print plan
        return plan
