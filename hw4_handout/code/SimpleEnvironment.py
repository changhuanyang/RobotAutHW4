import numpy, openravepy
import pylab as pl
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
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        
        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')
                     
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
        Duration = 0.2
        
        #4type_control setting 
        C_forward = Control(Omega_left,Omega_right,Duration);
        C_backward = Control(-Omega_left,-Omega_right,Duration);

        C_rightturn = Control(-Omega_left,Omega_right,Duration);
        C_leftrurn = Control(Omega_left,Omega_right,Duration);

        # Iterate through each possible starting orientation
        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = self.discrete_env.GridCoordToConfiguration(grid_coordinate)

            # TODO: Here you will construct a set of actions
            #  to be used during the planning process

            #generate the footprint-template,start form(o=0,0,idx) to somewhere
            fp_forward = GenerateFootprintFromControl(start_config,C_forward,0.01)
            fp_backward = GenerateFootprintFromControl(start_config,C_backward,0.01)
            fp_rightturn = GenerateFootprintFromControl(start_config,C_rightturn,0.01)
            fp_leftturn = GenerateFootprintFromControl(start_config,C_leftturn,0.01)
            #generate the Action-template
            Ac_forward = Action(C_forward,fp_forward)
            Ac_backward = Action(C_backward,fp_backward)
            Ac_rightturn = Action(C_rightturn,fp_rightturn)
            Ac_leftturn = Action(C_leftturn,fp_leftturn)

            #storage in actions
            self.actions[idx] = [Ac_forward,Ac_backward,Ac_rightturn,Ac_leftturn]



    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes
        coord = numpy.array(self.discrete_env.NodeIdToGridCoord(node_id))
        #print config
        
        


        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        start_config = numpy.array(self.discrete_env.NodeIdToConfiguration(start_id))
        goal_config = numpy.array(self.discrete_env.NodeIdToConfiguration(goal_id))
        #only use (x,y) to count the dist
        dist = numpy.linalg.norm(goal_config[0:1:1] - start_config[0:1:1])

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = ComputeDistance(start_id, end_id)

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        
        
        return cost

