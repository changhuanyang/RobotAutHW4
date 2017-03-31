import numpy
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        #
        #Peter: we have deal with the close to upper limit in DiscreteEnviroment.py/GridCoordToConfiguration
        #so this line will cause confusion
        #upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        #upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        #for idx in range(len(upper_config)):
        #    self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    
    def GetSuccessors(self, node_id):
        successors = []
        node_coord = numpy.array(self.discrete_env.NodeIdToGridCoord(node_id))

        #for each direction there are two type neighbor
        for idx in range(self.discrete_env.dimension):
                #positive direction
            if( (node_coord[idx] + 1) <= (self.discrete_env.num_cells[idx]-1) ):
                n_node_coord = numpy.copy(node_coord)
                n_node_coord[idx] += 1
                n_node_config = self.discrete_env.GridCoordToConfiguration(n_node_coord)
                if(self.no_collision(n_node_config)):
                    n_node_id = self.discrete_env.GridCoordToNodeId(n_node_coord)
                    successors.append(n_node_id)
            #negation direction
            if( (node_coord[idx] - 1) >= 0 ):
                n_node_coord = numpy.copy(node_coord)
                n_node_coord[idx] -= 1
                n_node_config = self.discrete_env.GridCoordToConfiguration(n_node_coord)
                if(self.no_collision(n_node_config)):
                    n_node_id = self.discrete_env.GridCoordToNodeId(n_node_coord)
                    successors.append(n_node_id)
        return successors

    def ComputeDistance(self, start_id, end_id):
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        #calculate distance
        dist = numpy.linalg.norm(numpy.array(start_config) - numpy.array(end_config))
        return dist
    def ComputeDistance_continue(self, start_id, end_id):
        dist = numpy.linalg.norm(numpy.array(start_id) - numpy.array(end_id))

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by te two node ids
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        cost = 0
        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        cost = self.ComputeDistance(start_id,goal_id)
        return cost
    def no_collision(self, n_config):
        with self.robot:
        #change the current position valuse wiht n_config
            self.robot.SetActiveDOFValues(numpy.array(n_config))
        #move robot to new positi
        #print "checkcollision = ",self.robot.GetEnv().CheckCollision(self.robot)
        #print "selfcheck= ",self.robot.CheckSelfCollision()      i 
            flag = self.robot.GetEnv().CheckCollision(self.robot) or self.robot.CheckSelfCollision()
            flag = not flag
        return flag
    def ComputePathLength(self, path):
        path_length = 0
        for milestone in range(1,len(path)):
            dist =  numpy.linalg.norm(numpy.array(path[milestone-1])-numpy.array(path[milestone]))
            path_length = path_length + dist
        return path_length

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())

        #
        # TODO: Generate and return a random configuration
        #
    # Brad: Generate and return a random, collision-free, configuration
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        collisionFlag = True
        while collisionFlag is True:
            for dof in range(len(self.robot.GetActiveDOFIndices())):
                config[dof] = lower_limits[dof] + (upper_limits[dof] - lower_limits[dof]) * numpy.random.random_sample()
                self.robot.SetActiveDOFValues(numpy.array(config))
            if(self.robot.GetEnv().CheckCollision(self.robot)) is False:
                if(self.robot.CheckSelfCollision()) is False:
                    collisionFlag = False
            #else:
                #print "Self Collision detected in random configuration"
        #else:
            #print "Collision Detected in random configuration" 
        return numpy.array(config)
    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
    # Brad: Extend from start to end configuration and return sooner if collision or limits exceeded
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        resolution = 100
    
    #Calculate incremental configuration changes
        config_inc = [0] * len(self.robot.GetActiveDOFIndices())
        for dof in range(len(self.robot.GetActiveDOFIndices())):
            config_inc[dof] = (end_config[dof] - start_config[dof]) / float(resolution)

    #Set initial config state to None to return if start_config violates conditions
        config = None
 
    #Move from start_config to end_config
        for step in range(resolution+1):
            prev_config = config
            config = [0] * len(self.robot.GetActiveDOFIndices())
            for dof in range(len(self.robot.GetActiveDOFIndices())):
            #Calculate new config
                config[dof] = start_config[dof] + config_inc[dof]*float(step)

            #Check joint limits
                if config[dof] > upper_limits[dof]:
                    print "Upper joint limit exceeded"
                    return prev_config
                if config[dof] < lower_limits[dof]:
                    print "Lower joint limit exceeded"
                    return prev_config

        #Set config and check for collision
        #CHECK: Lock environment?
            self.robot.SetActiveDOFValues(numpy.array(config))
            if(self.robot.GetEnv().CheckCollision(self.robot)) is True:
            #print "Collision Detected in extend"
                return prev_config
            if(self.robot.CheckSelfCollision()) is True:
            #print "Self Collision Detected in extend"
                return prev_config
        return end_config
