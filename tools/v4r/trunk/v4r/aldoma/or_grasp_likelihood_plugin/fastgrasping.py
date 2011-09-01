#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Rosen Diankov (rosen.diankov@gmail.com)
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Finds the first feasible grasp for an object without generating a grasp database.

.. examplepre-block:: fastgrasping

Description
-----------

This type of example is suited for object geometries that are dynamically created from sensor data.

.. examplepost-block:: fastgrasping
"""

from __future__ import with_statement # for python 2.5
__author__ = 'Aitor Aitoma & Beatriz Leon'

from itertools import izip
from openravepy import *
from numpy import *
from openravepy.misc import OpenRAVEGlobalArguments


class FastGrasping:
    """Computes a valid grasp for a given object as fast as possible without relying on a pre-computed grasp set
    """
    class GraspingException(Exception):
        def __init__(self,args):
            self.args=args

    def __init__(self,robot,target,env, prob):
        self.robot = robot
        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()
        self.gmodel = databases.grasping.GraspingModel(robot,target)
        self.gmodel.init(friction=0.4,avoidlinks=[])
        self.env = env
        self.prob = prob
        self.target = target
        
        with self.env:
            self.initialvalues = self.robot.GetDOFValues(self.gmodel.manip.GetArmIndices())

    def checkgraspfn(self, contacts, finalconfig, grasp, info):
        print "Force Closure Grasp, checking likelihood"
        # For each approach ray 
        
        # check if grasp can be reached by robot
        Tglobalgrasp = self.gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
        
        # have to set the preshape since the current robot is at the final grasp!
        self.gmodel.setPreshape(grasp)
        sol = self.gmodel.manip.FindIKSolution(Tglobalgrasp,True)
        if sol is not None:
            print "IK solution found" 
            
            likelihood = -1
            
            #build contact points string...
            cpoints = ''
            for x in range(len(contacts)):
                for i in range(3):
                    cpoints += str(contacts[x][i]) + ' '
            
            #print cpoints
            likelihood = self.prob.SendCommand('computeLikelihood ' + str(self.gmodel.target.GetEnvironmentId()) + ' ' + cpoints[:-1])
            
            likelihood_threshold = 0.5
            print "The contacts' likelihood is:", likelihood, "Accepted:", float(likelihood) >= likelihood_threshold
            #raw_input('Print any key')
            if float(likelihood) >= likelihood_threshold:
                jointvalues = array(finalconfig[0])
                jointvalues[self.gmodel.manip.GetArmIndices()] = sol
                raise self.GraspingException([grasp,jointvalues,contacts,likelihood])
            
        else:
            print "IK solution NOT found"
        #Should it not be false?
        return True
    
    def computeApprochRaysAndLikelihood(self):
        approachraysOriginal = self.gmodel.computeBoxApproachRays(delta=0.02,normalanglerange=0.5) # rays to approach object
        print "Number of approach rays", len(approachraysOriginal)
        approachrays = array([approachraysOriginal[1],approachraysOriginal[7]])
        approachrays = approachraysOriginal
        #send approach rays to plugin and sort them according to the overlap metric of its intersection point with the mesh
        #take into account only the intersection point, not the whole grasp!
        
        #Approach rays into a string
        rays_string = ''
        for x in range(len(approachrays)):
          for i in range(6):
            rays_string += str(approachrays[x][i]) + ' '
        
        rays_sort_string = self.prob.SendCommand('computeRaysOverlapLikelihood ' + str(self.gmodel.target.GetEnvironmentId()) + ' ' + rays_string[:-1])
        
        numbers = rays_sort_string.split(" ");
        for i in xrange(len(numbers)):
          numbers[i] = float(numbers[i])
        
        numbers = reshape(numbers, (int(len(numbers) / 6.0), 6))
        approachrays = numbers
        return approachrays


    def computeGrasp(self,env,updateenv=True):
        print "Compute grasp is called"
        
        approachrays = self.computeApprochRaysAndLikelihood()
        
        # standoffs
        standoffs = [0]
        
        # rolls
        # roll discretization (increases the number of possible grasps by 4 with 0.5*pi as step)
        rolls = arange(0,2*pi,0.5*pi) 
        
        # preshapes
        # initial preshape for robot is the released fingers
        with self.gmodel.target:
            self.gmodel.target.Enable(False)
            taskmanip = interfaces.TaskManipulation(self.robot)
            final,traj = taskmanip.ReleaseFingers(execute=False,outputfinal=True)
            preshapes = array([final])
        
        #Grasp
        try:          
            #generate grasps
            self.gmodel.disableallbodies=False
            self.gmodel.generate(preshapes=preshapes,standoffs=standoffs,rolls=rolls,approachrays=approachrays,checkgraspfn=self.checkgraspfn)
            return None,None,None,None # did not find anything
        except self.GraspingException, e:
            print "Grasp with selected likelihood Found!"
            selected_grasp = e.args[0]
            selected_jointvalues =  e.args[1]
            selected_contacts = e.args[2]    
            selected_likelihood =  e.args[3]    
            
            return selected_grasp,selected_jointvalues,selected_contacts,selected_likelihood
        
    def executeGrasp(self,validgrasp):
        print 'choosing a random grasp and move to its preshape'
        basemanip = interfaces.BaseManipulation(self.robot)
    
        try:
            self.gmodel.moveToPreshape(validgrasp)
            print 'move robot arm to grasp'
            Tgrasp = self.gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True)
            basemanip.MoveToHandPosition(matrices=[Tgrasp])
        except planning_error,e:
            print 'try again: ',e
    
        self.robot.WaitForController(10)
        taskmanip = interfaces.TaskManipulation(self.robot)
        taskmanip.CloseFingers()
        self.robot.WaitForController(10)
        with self.env:
            self.robot.Grab(self.target)
        raw_input('press any key to release')
        taskmanip.ReleaseFingers(target=self.target)
        self.robot.WaitForController(10)
        print 'initial values'
        basemanip.MoveManipulator(self.initialvalues)
        self.robot.WaitForController(10)
    

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    if options.manipname is not None:
        robot.SetActiveManipulator(options.manipname)
    # find an appropriate target
    bodies = [b for b in env.GetBodies() if not b.IsRobot() and linalg.norm(b.ComputeAABB().extents()) < 0.2]
    print "Body to grasp:", bodies[0].GetName()
    
    #Loading or_grasp_likelihood problem
    RaveLoadPlugin('/home/len/workspace/or_grasp_likelihood_plugin/libor_grasp_likelihood_plugin.so')
    prob = RaveCreateProblem(env,'orgrasplikelihood')
    env.LoadProblem(prob,args='')
    
    #Send the body to the problem -> replace this for the iv
    cmdout = prob.SendCommand('load /home/len/workspace/or_grasp_likelihood_plugin/file.vtk')
    #Check if the file has been load correctly. The cmdout is not doing anything now
#    if cmdout is None:
#        raveLogWarn('Fail to load File!')
    
    print "Initializing FastGrasping"
    self = FastGrasping(robot,target=bodies[0],env=env, prob = prob)
    
    print "FastGrasping.computeGrasp"
    grasp,jointvalues,contacts,contactsLikelihood = self.computeGrasp(env)
        
    if grasp is not None:
        print 'Showing selected grasp'
        print "Contacts_likelihood", contactsLikelihood
        self.gmodel.drawContacts(contacts)
        self.gmodel.showgrasp(grasp)
        #self.robot.SetDOFValues(jointvalues)
        print "Sending good grasp to the robot"
        self.executeGrasp(grasp)
        raw_input('press any key to finish')
    else:
        print "There wasn't any good grasp with acceptable likelihood"
    env.Destroy()

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@with_destroy 
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Example showing how to compute a valid grasp as fast as possible without computing a grasp set, this is used when the target objects change frequently.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene', action="store",type='string',dest='scene',default='data/tombatossals.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--manipname', action="store",type='string',dest='manipname',default=None,
                      help='Choose the manipulator to perform the grasping for')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    RaveInitialize(load_all_plugins=False)
    main(env,options)

if __name__ == "__main__":
    run()
