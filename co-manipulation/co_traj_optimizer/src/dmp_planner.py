#!/usr/bin/env python
import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from dmp.srv import *
from dmp.msg import *

from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

#Learn a DMP from demonstration data


class dmp_generator():
    def __init__(self):

        #Create a DMP from a 3-D trajectory
        dims = 3                
        dt = 0.5                
        K = 100                 
        D = 2.0 * np.sqrt(K)      
        num_bases = 20  # 15-20   

        points = np.arange(0,1,0.05)  # 20 points over range 200
        x_traj = self.make_xy(points, 0.5)
        y_traj = self.make_xy(points, 1)
        z_traj = self.make_z(points, 4)
        traj = [[x_traj[i], y_traj[i], z_traj[i]] for i in range(20)] 
        print("TRAINING TRAJECTORY ", traj)
        # self.plotTrajectory(traj)
        self.resp = self.makeLFDRequest(dims, traj, dt, K, D, num_bases)

        # #Set it as the active DMP
        self.makeSetActiveRequest(self.resp.dmp_list)
        print(self.resp)

        # #Now, generate a plan
        # x_0 = [5.0,3.0,3.0]          #Plan starting at a different point than demo 
        # goal = [0.0,7.0,2.0]         # Get this from AR tag location for the primitive
        
        # plan = self.generatePlan(x_0, goal)

        # print (plan)  # Write this to a file and pass to moveit
        # print(plan.plan.points)
        # new_traj = []
        # for point in plan.plan.points:
        #     new_traj.append([point.positions[0], point.positions[1], point.positions[2]])
        # print(new_traj)
        # self.plotTrajectory(new_traj)



    def generatePlan(self, x_0, goal):
        x_dot_0 = [0.0,0.0,0.0]   
        t_0 = 0
        goal_thresh = [0.2,0.2,0.2]
        seg_length = -1          #Plan until convergence to goal
        tau = 2 * self.resp.tau       #Desired plan should take twice as long as demo
        dt = 0.5
        integrate_iter = 5       #dt is rather large, so this is > 1  
        plan = self.makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                            seg_length, tau, dt, integrate_iter)
        return plan


    def plotTrajectory(self, traj):
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        nptraj = np.array(traj)
        print(nptraj.shape)
        x = nptraj[:, 0]
        y = nptraj[:, 1]
        z = nptraj[:, 2]
        ax.plot3D(x, y, z, 'gray')

        plt.show()


    def makeLFDRequest(self, dims, traj, dt, K_gain, 
                    D_gain, num_bases):
        demotraj = DMPTraj()
            
        for i in range(len(traj)):
            pt = DMPPoint()
            pt.positions = traj[i]
            demotraj.points.append(pt)
            demotraj.times.append(dt*i)
                
        k_gains = [K_gain]*dims
        d_gains = [D_gain]*dims
            
        print "Starting LfD..."
        rospy.wait_for_service('learn_dmp_from_demo')
        try:
            lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
            resp = lfd(demotraj, k_gains, d_gains, num_bases)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "LfD done"    
                
        return resp


    #Set a DMP as active for planning
    def makeSetActiveRequest(self, dmp_list):
        try:
            sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
            sad(dmp_list)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    #Generate a plan from a DMP
    def makePlanRequest(self, x_0, x_dot_0, t_0, goal, goal_thresh, 
                        seg_length, tau, dt, integrate_iter):
        print "Starting DMP planning..."
        rospy.wait_for_service('get_dmp_plan')
        try:
            gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
            resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "DMP planning done"   
                
        return resp

    def make_xy(self, points, scalar):
        output = list()
        for point in points:
            output.append(point * scalar)
        return output


    def make_z(self, points, scalar):
        output = list()
        for point in points:
            sample = -scalar*(point - 0.5)**2 + 1
            output.append(sample)
        return output


if __name__ == '__main__':
    rospy.init_node('dmp_test_node')
    dmp_gen = dmp_generator()