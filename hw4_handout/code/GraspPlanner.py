import logging, numpy, openravepy, time, copy
import IPython

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
       
        self.robot = robot
        self.env = self.robot.GetEnv()
        self.manip = self.robot.GetActiveManipulator()
        self.base_planner = base_planner
        self.arm_planner = arm_planner

            
    def GetBasePoseForObjectGrasp(self, obj):

        # Load grasp database
        # change gmodel to sel.gmodel since eval_grasp need it
        self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
        if not self.gmodel.load():
            self.gmodel.autogenerate()

        basePose = None
        graspConfig = None
        self.graspindices = self.gmodel.graspindices
        self.grasps = self.gmodel.grasps
        self.order_grasps()
        #TODO uncomment orgConfig
        #orgConfig = self.base_planner.planning_env.herb.GetCurrentConfiguration()
        # get the highest score grasping pose
        self.show_grasp(self.grasps_ordered[0]) # visualize solution, seems ok
        #print "top grasping {}".format(self.grasps_ordered[0])
        graspTransform = self.gmodel.getGlobalGraspTransform(self.grasps_ordered[0], collisionfree = True)
      
        irModel = openravepy.databases.inversereachability.InverseReachabilityModel(self.robot) 
        if not irModel.load():
            irModel.autogenerate()
        loaded = irModel.load()
        print "irModel loaded? {}".format(loaded)
        densityFN, samplerFN, bounds = irModel.computeBaseDistribution(graspTransform)
        #print "bounds {}".format(bounds)
        poses, joint = samplerFN(500)
        #print "pose number: {}".format(len(poses))
        #print "pose:  {}".format(poses[0])
        #print "jointstate: {}".format(jointstate[0])
        #angle = openravepy.axisAngleFromQuat(poses[0])
        #print "angle:  {}".format(angle)
        #trans = openravepy.matrixFromPose(poses[0])
        #print "trans:  {}".format(trans)
        
        for pose in poses:
    	    self.robot.SetTransform(pose) # pose format [s, vx, vy, vz, x, y, z]
            angle = openravepy.axisAngleFromQuat(pose)
            continuousPose = copy.deepcopy([pose[4], pose[5], angle[2]]) # 2D location with orientation
            #TODO convert continuous pose to discrete pose
            #node = self.base_planner.planning_env.discrete_env.ConfigurationToNodeId(continuousPose)
            #discretePose = self.base_planner.planning_env.discrete_env.NodeIdToConfiguration(node)
            discretePose = continuousPose #TODO need to be commented out
            basePose = openravepy.quatFromAxisAngle([0, 0, discretePose[2]])
            basePose = numpy.append(basePose, [discretePose[0], discretePose[1], 0])

            obstacles = self.robot.GetEnv().GetBodies()

            self.robot.SetTransform(basePose)
            #get grasp joing config from IK
            graspConfig = self.manip.FindIKSolution(graspTransform,filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions.IgnoreEndEffectorCollisions)
            if self.robot.GetEnv().CheckCollision(self.robot, obstacles[1]) != True and graspConfig != None:
                print "basePose:  {}".format(basePose)
                print "graspConfig:  {}".format(graspConfig)
                #IPython.embed()
                #TODO restore robot position before return
                #self.base_planner.planning_env.herb.SetCurrentConfiguration(orgConfig)
                return basePose, graspConfig
 
        print "Fail to find solution!!!"
        return basePose, graspConfig

    #copy from hw1
    def order_grasps(self):
        self.grasps_ordered = self.grasps.copy() #you should change the order of self.grasps_ordered
        for grasp in self.grasps_ordered:
            grasp[self.graspindices.get('performance')] = self.eval_grasp(grasp)

        # sort!
        order = numpy.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
        order = order[::-1]
        self.grasps_ordered = self.grasps_ordered[order]
    
    def eval_grasp(self, grasp):
        with self.robot:
        #contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
            try:
                contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)

                obj_position = self.gmodel.target.GetTransform()[0:3,3]
                # for each contact
                G = numpy.array([]) #the wrench matrix

                for c in contacts:
                    pos = c[0:3] - obj_position
                    #what is dir? Guess it's force direction
                    dir = -c[3:] #this is already a unit vector
                    moment  = numpy.cross(pos, dir)
                    G = numpy.append(G, numpy.append(dir, moment))

                G = numpy.reshape(G, (-1, 6))
                #1. use minimal magnitude 
                UMatrix, Singular, VMatrix = numpy.linalg.svd(G)
                quality = Singular[-1]
                #2. use the volume of the ellipsoid in the wrench space
                #quality = numpy.linalg.det(numpy.dot(G, G.transpose()))

                return quality

            except openravepy.planning_error,e:
                #you get here if there is a failure in planning
                #example: if the hand is already intersecting the object at the initial position/orientation
                return  -100.00 # way smaller than most of the scores

      #displays the grasp
    def show_grasp(self, grasp, delay=2):
        with openravepy.RobotStateSaver(self.gmodel.robot):
            with self.gmodel.GripperVisibility(self.gmodel.manip):
                time.sleep(0.1) # let viewer update?
                try:
                    with self.env:
                        contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=True)
                        #if mindist == 0:
                        #  print 'grasp is not in force closure!'
                        contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
                        self.gmodel.robot.GetController().Reset(0)
                        self.gmodel.robot.SetDOFValues(finalconfig[0])
                        self.gmodel.robot.SetTransform(finalconfig[1])
                        self.env.UpdatePublishedBodies()
                        time.sleep(delay)
                except openravepy.planning_error,e:
                    print 'bad grasp!',e

    def PlanToGrasp(self, obj):

        # Next select a pose for the base and an associated ik for the arm
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()

        # Now plan to the base pose
        start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        # Now plan the arm to the grasp configuration
        start_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipultion.CloseFingers()
    
