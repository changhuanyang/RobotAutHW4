import numpy, openravepy
import pylab as pl
import IPython
import copy
from DiscreteEnvironment import DiscreteEnvironment

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        lower_limits, upper_limits = self.boundary_limits
        #print(lower_limits)
        #print(upper_limits)
        #print(resolution)
        self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)
        self.resolution = resolution
        self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = start_config.copy()
        footprint = [numpy.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance
                
            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*numpy.array([xdot, ydot, tdot])
            if config[2] > numpy.pi:
                config[2] -= 2.*numpy.pi
            if config[2] < -numpy.pi:
                config[2] += 2.*numpy.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize
            
        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        return footprint

    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure(1)
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        i = 0;
        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            if(i==0):
                pl.plot(xpoints, ypoints,'-r')
            if(i==1):
                pl.plot(xpoints, ypoints,'-b')
            if(i==2):
                pl.plot(xpoints, ypoints,'-y')
            if(i==3):
                pl.plot(xpoints, ypoints,'-w')
            i += 1
            if(i == 4):
                i = 0
        pl.ion()
        pl.show()
        #help function
    def ComputePathLength(self, path):
        length = 0

        for milestone in range(1,len(path)):
            dist =  numpy.linalg.norm(numpy.array(path[milestone-1])-numpy.array(path[milestone]))
            length += dist
        return length
    def ConstructActions(self):

        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        self.actions = dict()
              
        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)
        
        #Peter
        Omega_left  = 1
        Omega_right = 1
        Duration = 1.1
        
        #4type_control setting 
        C_forward = Control(Omega_left,Omega_right,Duration);
        C_backward = Control(-Omega_left,-Omega_right,Duration);

        #C_rightturn = Control(-Omega_left, Omega_right, Duration);
        #C_leftturn = Control(Omega_left, -Omega_right, Duration);

        C_rightturn = Control(-Omega_left*(self.resolution[2]/0.8),Omega_right*(self.resolution[2]/0.8),Duration);
        C_leftturn = Control(Omega_left*(self.resolution[2]/0.8),-Omega_right*(self.resolution[2]/0.8),Duration);
        # Iterate through each possible starting orientation
        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = numpy.array(self.discrete_env.GridCoordToConfiguration(grid_coordinate))

            # TODO: Here you will construct a set of actions
            #  to be used during the planning process

            #generate the footprint-template,start form(o=0,0,idx) to somewhere
            fp_forward = self.GenerateFootprintFromControl(start_config,C_forward,0.01)
            fp_backward = self.GenerateFootprintFromControl(start_config,C_backward,0.01)
            fp_rightturn = self.GenerateFootprintFromControl(start_config,C_rightturn,0.01)
            fp_leftturn = self.GenerateFootprintFromControl(start_config,C_leftturn,0.01)
            #generate the Action-template
            Ac_forward = Action(C_forward,numpy.array(fp_forward))
            Ac_backward = Action(C_backward,numpy.array(fp_backward))
            Ac_rightturn = Action(C_rightturn,numpy.array(fp_rightturn))
            Ac_leftturn = Action(C_leftturn,numpy.array(fp_leftturn))

            #storage in actions
            self.actions[idx] = [Ac_forward,Ac_backward,Ac_rightturn,Ac_leftturn]

            #self.PlotActionFootprints(idx)
    #help function
    def no_collision(self, config):

        pose =  openravepy.quatFromAxisAngle([0, 0, config[2]])
        pose = numpy.append(pose, [config[0], config[1], 0])
        self.robot.SetTransform(pose)
        #IPython.embed()
        collision = self.robot.GetEnv().CheckCollision(self.robot)

        return not collision


    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes
        coord = numpy.array(self.discrete_env.NodeIdToGridCoord(node_id))
        #print config
        
        #get currentconfig
        start_config = self.discrete_env.NodeIdToConfiguration(node_id)
        #print "start_config {}".format(start_config)
        #get current angle
        current_angle = coord[2]

        #use library to get the action template
        action_set_of_this_angle = self.actions[current_angle]
        # the structure of successor
        # [id_forward      id_backward      id_rightturn   id_leftturn ]
        for i in range(len(action_set_of_this_angle)):
            #get the control and footprint of this action
            this_action = action_set_of_this_angle[i];
            foot_print = this_action.footprint
            #print "foot_print {}".format(foot_print)
            #this_action.footprint = [[x[0]+start_config[0], x[1]+start_config[1], x[2]] for x in this_action.footprint]
            #print "foot_print {}".format(foot_print)
            #get the snap config from footprint(the last one)
            #new_config = numpy.array(this_action.footprint[-1]) 
            #print "foot_print {}".format(this_action.footprint[-1])
            increase_step = foot_print[-1]
            new_config = numpy.array(start_config).copy()
            new_config[2] = 0
            new_config = new_config + numpy.array(increase_step) 
            #wrap to pi
            nid = self.discrete_env.ConfigurationToNodeId(new_config)
            #collision check
            if (self.no_collision(new_config)):
                successors.append([nid, this_action])
                #IPython.embed()
        #print('number of successors = ',len(successors))
        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        start_config = numpy.array(self.discrete_env.NodeIdToConfiguration(start_id))
        goal_config = numpy.array(self.discrete_env.NodeIdToConfiguration(end_id))
        #only use (x,y) to count the dist

        dist = numpy.linalg.norm(goal_config- start_config)
        dist = float(dist)

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = self.ComputeDistance(start_id, goal_id)

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        
        
        return cost

