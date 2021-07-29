#! /usr/bin/env python
# call roscore
# $ roscore
#
# If start in manual
# $ rosrun cbf_ros cbf_controller.py
import rospy
import sys
import argparse
import re
import numpy as np
from scipy.integrate import odeint
from sympy import symbols, Matrix, sin, cos, lambdify, exp, sqrt, log
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import cvxopt as cvxopt

# ROS msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetWorldProperties, GetModelState, GetModelStateRequest

# ROS others
import tf

DEBUG = False

def orientation2angular(orientation):
        quaternion = (  orientation.x,
                        orientation.y,
                        orientation.z,
                        orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        angular = Vector3(
                euler[0],
                euler[1],
                euler[2]
        )
        return angular

def cvxopt_solve_qp(P, q, G=None, h=None, A=None, b=None):
    P = .5 * (P + P.T)  # make sure P is symmetric
    args = [cvxopt.matrix(P), cvxopt.matrix(q)]
    if G is not None:
        args.extend([cvxopt.matrix(G), cvxopt.matrix(h)])
        if A is not None:
            args.extend([cvxopt.matrix(A), cvxopt.matrix(b)])
    cvxopt.solvers.options['show_progress'] = False
    cvxopt.solvers.options['maxiters'] = 100
    sol = cvxopt.solvers.qp(*args)
    if 'optimal' not in sol['status']:
        return None
    return np.array(sol['x']).reshape((P.shape[1],))

def plottrajs(trajs):
        if plotanimation:
                for j in range(len(trajs.hsr)):
                        plt.axis([-10,10,-10,10],color ="black")
                        plt.plot([-1.4,-1.4],[-7,7],color ="black")
                        plt.plot([1.3,1.3],[-7,-1.5],color ="black")
                        plt.plot([1.3,1.3],[1.4,7],color ="black")
                        plt.plot([1.3,7],[1.4,1.4],color ="black")
                        plt.plot([1.3,7],[-1.5,-1.5],color ="black")

                        plt.plot(trajs.hsr[j][1],-trajs.hsr[j][0],color ="green",marker = 'o')
                        plt.arrow(float(trajs.hsr[j][1]),float(-trajs.hsr[j][0]), float(2*trajs.commands[j][0]*sin(trajs.hsr[j][2])), float(-2*trajs.commands[j][0]*cos(trajs.hsr[j][2])), width = 0.05)               
                        for k in range(len(trajs.actors[j])):
                                plt.plot(trajs.actors[j][k][1],-trajs.actors[j][k][0],color ="red",marker = 'o')
                        plt.draw()
                        plt.pause(np.finfo(float).eps)
                        plt.clf()
        plt.ion()
        plt.axis([-10,10,-10,10],color ="black")
        plt.plot([-1.4,-1.4],[-7,7],color ="black")
        plt.plot([1.3,1.3],[-7,-1.5],color ="black")
        plt.plot([1.3,1.3],[1.4,7],color ="black")
        plt.plot([1.3,7],[1.4,1.4],color ="black")
        plt.plot([1.3,7],[-1.5,-1.5],color ="black")
        for j in range(len(trajs.hsr)):
                plt.axis([-10,10,-10,10])
                plt.plot(trajs.hsr[j][1],-trajs.hsr[j][0],color ="green",marker = 'o')
                for k in range(len(trajs.actors[j])):
                        plt.plot(trajs.actors[j][k][1],-trajs.actors[j][k][0],color ="red",marker = 'o')
        plt.draw()
        plt.pause(np.finfo(float).eps)
        plt.ioff()


        fig, axs = plt.subplots(3)
        axs[0].set_title('controls: velocity (green), angular velocity (red)')
        # axs[1].set_title('risk')
        # axs[2].set_title('min Dist')
        axs[0].set(ylabel = 'controls')
        axs[1].set(ylabel = 'risk')
        axs[2].set(xlabel = 'time', ylabel = 'min Dist')

        for k in range(len(trajs.time)):
                axs[0].plot(trajs.time[k], trajs.commands[k][0],color ="green",marker = 'o')
                axs[0].plot(trajs.time[k], trajs.commands[k][1],color ="red",marker = 'o')
                if trajs.risk[k]<risk:
                        axs[1].plot(trajs.time[k], trajs.risk[k],color ="green",marker = 'o')
                else:
                        axs[1].plot(trajs.time[k], trajs.risk[k],color ="red",marker = 'o')
                axs[2].plot(trajs.time[k], trajs.minDist[k],color ="green",marker = 'o')


        plt.draw()
        plt.pause(60)
        1
        # plt.ioff()
        # plt.figure(3)
        # for k in range(len(trajs.time)):
        #         plt.plot(trajs.time[k], trajs.risk[k],color ="green",marker = 'o')
        # plt.draw()
        # 1



class robot(object):
        def __init__(self,l):
               #Symbolic Variables
                # t = symbols('t')
                # when robot is bicycle model [x,y,theta], obstacles are linear models [x,y]:
                xr1,xr2,xr3,xo1,xo2 = symbols('xr1 xr2 xr3 xo1 xo2')
                # v w inputs of robot:
                u1,u2 = symbols('u1,u2')
                vx,vy = symbols('vx,vy')
                # Vector of states and inputs:
                self.x_r_s = Matrix([xr1,xr2,xr3])
                self.x_o_s = Matrix([xo1,xo2])
                self.u_s = Matrix([u1,u2])
                self.u_o = Matrix([vx,vy]) 

                self.f = Matrix([0,0,0])
                self.g = Matrix([[cos(self.x_r_s[2]), -l*sin(self.x_r_s[2])], [sin(self.x_r_s[2]), l*cos(self.x_r_s[2])], [0, 1]])
                self.f_r = self.f+self.g*self.u_s 
                self.l = l #approximation parameter for bicycle model
                self.Real_x_r = lambdify([self.x_r_s], self.x_r_s-Matrix([l*cos(self.x_r_s[2]), l*sin(self.x_r_s[2]), 0]))

                # Obstacle SDE, not needed if we want to use Keyvan prediction method
                self.f_o = self.u_o
                # self.f_o = Matrix([0.1, 0.1])
                self.g_o = Matrix([0.2, 0.2])

                # self.f_o_fun = lambdify([self.x_o_s], self.f_o)
                # self.g_o_fun = lambdify([self.x_o_s], self.g_o)

        def GoalFuncs(self,GoalCenter,rGoal):
                Gset = (self.x_r_s[0]-GoalCenter[0])**2+(self.x_r_s[1]-GoalCenter[1])**2-rGoal
                GoalInfo = type('', (), {})()
                GoalInfo.set = lambdify([self.x_r_s],Gset)
                GoalInfo.Lyap = lambdify([self.x_r_s,self.u_s],Gset.diff(self.x_r_s).T*self.f_r)
                return GoalInfo

        def UnsafeFuncs(self,gamma,UnsafeRadius):   #based on the SDE formulation, needs slight change for regular BF
                UnsafeInfo = type('', (), {})()
                Uset = (self.x_r_s[0]-self.x_o_s[0])**2+(self.x_r_s[1]-self.x_o_s[1])**2-(UnsafeRadius+self.l)**2
                CBF = exp(-gamma*Uset)
                CBF_d = CBF.diff(Matrix([self.x_r_s,self.x_o_s]))
                CBF_d2 =  CBF.diff(self.x_o_s,2)
                UnsafeInfo.set = lambdify([self.x_r_s,self.x_o_s], Uset)
                UnsafeInfo.CBF = lambdify([self.x_r_s,self.x_o_s], CBF)
                UnsafeInfo.ConstCond = lambdify([self.x_r_s,self.x_o_s,self.u_o] , CBF_d.T*Matrix([self.f,self.f_o])+0.5*(self.g_o.T*Matrix([[Matrix(CBF_d2[0,0]),Matrix(CBF_d2[1,0])]])*self.g_o))
                UnsafeInfo.multCond = lambdify([self.x_r_s,self.x_o_s,self.u_s], CBF_d.T*Matrix([self.g*self.u_s, Matrix(np.zeros((len(self.x_o_s),1)))]))
                return UnsafeInfo

        def MapFuncs(self,env_bounds):
                MapInfo = type('', (), {})()
                MapInfo.set = []
                MapInfo.CBF = []
                MapInfo.setDer = []
                # x_min = getattr(env_bounds, "x_min", undefined)
                # x_max = getattr(env_bounds, "x_max", undefined)
                # y_min = getattr(env_bounds, "y_min", undefined)
                # y_max = getattr(env_bounds, "y_max", undefined)
                if hasattr(env_bounds,'x_min'):
                        Uset = (-self.x_r_s[0]+env_bounds.x_min)
                        CBF = exp(gamma*Uset)
                        MapInfo.set.append(lambdify([self.x_r_s], Uset))
                        MapInfo.CBF.append(lambdify([self.x_r_s],CBF))
                        MapInfo.setDer.append(lambdify([self.x_r_s,self.u_s] , CBF.diff(self.x_r_s).T*self.f_r))
                if hasattr(env_bounds,'x_max'):
                        Uset = (self.x_r_s[0]-env_bounds.x_max)
                        CBF = exp(gamma*Uset)
                        MapInfo.set.append(lambdify([self.x_r_s], Uset))
                        MapInfo.CBF.append(lambdify([self.x_r_s],CBF))
                        MapInfo.setDer.append(lambdify([self.x_r_s,self.u_s] , CBF.diff(self.x_r_s).T*self.f_r))
                if hasattr(env_bounds,'y_min'):
                        Uset = (-self.x_r_s[1]+env_bounds.y_min)
                        CBF = exp(gamma*Uset)
                        MapInfo.set.append(lambdify([self.x_r_s], Uset))
                        MapInfo.CBF.append(lambdify([self.x_r_s],CBF))
                        MapInfo.setDer.append(lambdify([self.x_r_s,self.u_s] , CBF.diff(self.x_r_s).T*self.f_r))
                if hasattr(env_bounds,'y_max'):
                        Uset = (self.x_r_s[1]-env_bounds.y_max)
                        CBF = exp(gamma*Uset)
                        MapInfo.set.append(lambdify([self.x_r_s], Uset))
                        MapInfo.CBF.append(lambdify([self.x_r_s],CBF))
                        MapInfo.setDer.append(lambdify([self.x_r_s,self.u_s] , CBF.diff(self.x_r_s).T*self.f_r))
                if hasattr(env_bounds,'f'):
                        pass #To be filled later

                return MapInfo






class CBF_CONTROLLER(object):
	def __init__(self,robot,GoalInfo,UnsafeInfo,MapInfo):
                # publisher to send vw order to HSR
                self.vw_publisher = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
                # subscriber for Gazebo info.
                rospy.wait_for_service ('/gazebo/get_model_state')
                self.get_model_pro = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
                self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                self.tOdometry_subscriber = rospy.Subscriber('/hsrb/odom_ground_truth', Odometry, self.tOdometry_callback, queue_size=10)
                self.tOdometry = Odometry()
                self.odometry_subscriber = rospy.Subscriber('/global_pose', PoseStamped, self.odometry_callback, queue_size=10)
                self.poseStamped = PoseStamped()
                # listener of tf.
                self.tfListener = tf.TransformListener()

                self.actors = []
                trajs = type('', (), {})()
                trajs.hsr = []
                trajs.actors = []
                trajs.commands = []
                trajs.time = []
                trajs.risk = []
                trajs.minDist = []
                self.trajs = trajs
                self.robot = robot
                self.GoalInfo = GoalInfo
                self.UnsafeInfo = UnsafeInfo
                self.MapInfo = MapInfo
                self.flag = 0
                self.count = 0 # num of times control_callback is called

        def __del__(self):
                pass

        def tOdometry_callback(self, odometry):
                self.odometry = odometry # this odometry's coodination is \map

        def odometry_callback(self, poseStamped):
                self.poseStamped = poseStamped

        def gazebo_pos_transformPose(self, frame_id, gazebo_pose):
                gazebo_pose_temp = PoseStamped()
                gazebo_pose_temp.header = gazebo_pose.header
                gazebo_pose_temp.header.frame_id = 'map'
                gazebo_pose_temp.pose = gazebo_pose.pose
                while not rospy.is_shutdown():
                        try:
                                gazebo_pos_trans = self.tfListener.transformPose(frame_id, gazebo_pose_temp)
                                break
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                                continue

                return gazebo_pos_trans




        def controller_loop_callback(self, event):
                # this controller loop call back.
                self.count += 1
                now = rospy.get_rostime()
                self.trajs.time.append(now.secs+now.nsecs*pow(10,-9))
                if DEBUG:
                        rospy.loginfo('Current time %i %i', now.secs, now.nsecs)
                        rospy.loginfo('tOdometry\n %s', self.odometry)
                # get human model state from Gazebo
                if self.count==1:
                        model_properties = self.get_model_pro()
                        for model_name in model_properties.model_names:
                                if re.search('actor*', model_name) and not model_name in self.actors:  # if the model name is actor*, it will catch them.
                                        self.actors.append(model_name)
                actors_data = []
                for actor in self.actors:
                        model_actor = GetModelStateRequest()
                        model_actor.model_name = actor
                        model_actor = self.get_model_srv(model_actor) # the pose date is based on /map
                        # actor_base_footprint_pose = self.gazebo_pos_transformPose('base_footprint', model_actor) # trasfer /map->/base_footprint
                        angular = orientation2angular(model_actor.pose.orientation)      # transfer orientaton(quaternion)->agular(euler)
                        p = model_actor.pose.position
                        actors_data.append([p.x,p.y, angular.z])
                        if DEBUG:
                                rospy.loginfo('%s in timestamp:\n%s', actor, model_actor.header.stamp) # time stamp is here.
                                rospy.loginfo('%s in base_footprint\nposition:\n%s\nangular:\n%s', actor, actor_base_footprint_pose.pose.position, angular)
                self.trajs.actors.append(actors_data)


                # get hsr model state from odometry
                model_hsr = self.odometry
                p = model_hsr.pose.pose.position
                angular = orientation2angular(model_hsr.pose.pose.orientation)      # transfer orientaton(quaternion)->agular(euler)
                x_r = [p.x,p.y,angular.z]
                self.trajs.hsr.append(x_r)

                # making vw data and publish it.
                vel_msg = Twist()
                # Compute controller
                if abs(p.x)<1.5 and self.flag == 0:
                        self.flag = 1
                        env_bounds = type('', (), {})()
                        env_bounds.x_max = 1.2
                        env_bounds.x_min = -1.3
                        self.MapInfo = self.robot.MapFuncs(env_bounds)
                        GoalCenter = np.array([0, 5.5])
                        self.GoalInfo = self.robot.GoalFuncs(GoalCenter,rGoal)



                u = self.cbf_controller_compute()
                vel_msg.linear.x  = u[0]
                vel_msg.angular.z = u[1]
                self.vw_publisher.publish(vel_msg)
                self.trajs.commands.append([u[0],u[1]])


                if self.count > 1000:
                        rospy.loginfo('reach counter!!')
                        rospy.signal_shutdown('reach counter')
                elif self.GoalInfo.set(x_r)<0:
                        rospy.loginfo('reached Goal set!!')
                        rospy.signal_shutdown('reached Goal set')

        def cbf_controller_compute(self):
                x_r = np.array(self.trajs.hsr[len(self.trajs.hsr)-1])
                x_o = np.array(self.trajs.actors[len(self.trajs.actors)-1])
                u_s = self.robot.u_s
                if self.count>2:
                        x_o_pre = np.array(self.trajs.actors[len(self.trajs.actors)-3])
                        dt = self.trajs.time[len(self.trajs.time)-1]-self.trajs.time[len(self.trajs.time)-3]
                        u_o = (x_o[:,0:2]-x_o_pre[:,0:2])/dt
                else:
                        u_o = np.zeros((len(x_o),len(self.robot.u_o)))

                Unsafe = self.UnsafeInfo
                Goal = self.GoalInfo
                Map = self.MapInfo
                UnsafeList = []
                Dists = np.zeros((len(x_o)))
                for j  in range(len(x_o)):
                        Dists[j] = Unsafe.set(x_r,  x_o[j][0:2])
                        if Dists[j]<UnsafeInclude:
                                UnsafeList.append(j)
                ai = 1
                if min(Dists)<0:
                        InUnsafe = 1
                else:
                        InUnsafe = 0
                minDist = min(Dists)
                minJ = np.where(Dists == minDist)


                if findBestCommandAnyway:
                        #Ax<=b, x = [v, w , b1,bh1 b2, bh2..., bn, b'1, b'2,b'm, delta ]
                        # where b is constant in Eq (14) of paper "Risk-bounded  Control  using  Stochastic  Barrier  Functions"
                        #b' is the slack variable for map constraints
                        # delta is for lyapunov function
                        A = np.zeros((2*len(UnsafeList)+2*len(u_s)+len(Map.set)+2,len(u_s)+2*len(UnsafeList)+len(Map.set)+1))
                        b =np.zeros((2*len(u_s)+2*len(UnsafeList)+len(Map.set)+2))
                        for j in range(len(UnsafeList)):
                                # CBF Constraints
                                A[2*j,np.append(np.arange(len(u_s)),[len(u_s)+2*j])] = [Unsafe.multCond(x_r,  x_o[UnsafeList[j]][0:2],[1, 0]), Unsafe.multCond(x_r,x_o[UnsafeList[j]][0:2],[0, 1]), -1] # multiplier of u , bi
                                b[2*j] = -ai* Unsafe.CBF(x_r, x_o[UnsafeList[j]][0:2])- Unsafe.ConstCond(x_r,  x_o[UnsafeList[j]][0:2],u_o[UnsafeList[j]]) 
                                # Constraints on bi to satisfy pi risk
                                A[2*j+1,len(u_s)+2*j] = 1; A[2*j+1,len(u_s)+2*j+1] = -1
                                if Unsafe.CBF(x_r, x_o[UnsafeList[j]][0:2])<1:
                                        b[2*j+1] = min(ai, -1/T*log((1-risk)/(1-Unsafe.CBF(x_r, x_o[UnsafeList[j]][0:2]))))
                                else:
                                        b[2*j+1] = 0




                        # Adding U constraint
                        A[2*len(UnsafeList),0] = 1; b[2*len(UnsafeList)] = U[0,1]
                        A[2*len(UnsafeList)+1,0] = -1;  b[2*len(UnsafeList)+1] = -U[0,0]
                        A[2*len(UnsafeList)+2,1] = 1; b[2*len(UnsafeList)+2] = U[1,1]
                        A[2*len(UnsafeList)+3,1] = -1; b[2*len(UnsafeList)+3] = -U[1,0]

                        # Adding map constraints
                        for j in range(len(Map.set)):
                                A[2*len(UnsafeList)+2*len(u_s)+j,np.append(np.arange(len(u_s)),[len(u_s)+2*len(UnsafeList)+j])] = [Map.setDer[j](x_r,[1, 0]), Map.setDer[j](x_r,[0, 1]), -1]
                                b[2*len(UnsafeList)+2*len(u_s)+j] = -Map.CBF[j](x_r)

                        # Adding Goal based Lyapunov !!!!!!!!!!!!!!!!! Needs to be changed for a different example 
                        A[2*len(UnsafeList)+2*len(u_s)+len(Map.set),0:2] = [Goal.Lyap(x_r,[1,0]), Goal.Lyap(x_r,[0, 1])]
                        A[2*len(UnsafeList)+2*len(u_s)+len(Map.set),-1] = -1
                        b[2*len(UnsafeList)+2*len(u_s)+len(Map.set)] = 0
                        A[2*len(UnsafeList)+2*len(u_s)+len(Map.set)+1,-1] = 1
                        b[2*len(UnsafeList)+2*len(u_s)+len(Map.set)+1] = np.finfo(float).eps+1

                        H = np.zeros((len(u_s)+2*len(UnsafeList)+len(Map.set)+1,len(u_s)+2*len(UnsafeList)+len(Map.set)+1))
                        H[0,0] = 0
                        H[1,1] = 0

                        ff = np.zeros((len(u_s)+2*len(UnsafeList)+len(Map.set)+1,1))
                        for j in range(len(UnsafeList)):
                                ff[len(u_s)+2*j] = 65
                                H[len(u_s)+2*j+1,len(u_s)+2*j+1] = 10000
                                # ff[len(u_s)+2*j+1] = 50* Unsafe.CBF(x_r, x_o[minJ[0][0]][0:2])

                        ff[len(u_s)+2*len(UnsafeList):len(u_s)+2*len(UnsafeList)+len(Map.set)] = 20
                        ff[-1] = np.ceil(self.count/100.0)
                else:
                        #Ax<=b, x = [v, w , b1, b2,..., bn, b'1, b'2,b'm, delta ]
                        # where b is constant in Eq (14) of paper "Risk-bounded  Control  using  Stochastic  Barrier  Functions"
                        #b' is the slack variable for map constraints
                        # delta is for lyapunov function
                        A = np.zeros((2*len(UnsafeList)+2*len(u_s)+len(Map.set)+2,len(u_s)+len(UnsafeList)+len(Map.set)+1))
                        b =np.zeros((2*len(u_s)+2*len(UnsafeList)+len(Map.set)+2))
                        for j in range(len(UnsafeList)):
                                # CBF Constraints
                                A[2*j,np.append(np.arange(len(u_s)),[len(u_s)+j])] = [Unsafe.multCond(x_r,  x_o[UnsafeList[j]][0:2],[1, 0]), Unsafe.multCond(x_r,x_o[UnsafeList[j]][0:2],[0, 1]), -1] # multiplier of u , bi
                                b[2*j] = -ai* Unsafe.CBF(x_r, x_o[UnsafeList[j]][0:2])- Unsafe.ConstCond(x_r,  x_o[UnsafeList[j]][0:2],u_o[UnsafeList[j]]) 
                                # Constraints on bi to satisfy pi risk
                                A[2*j+1,len(u_s)+j] = 1
                                if Unsafe.CBF(x_r, x_o[UnsafeList[j]][0:2])<1:
                                        b[2*j+1] = min(ai, -1/T*log((1-risk)/(1-Unsafe.CBF(x_r, x_o[UnsafeList[j]][0:2]))))
                                else:
                                        b[2*j+1] = 0
                        # Adding U constraint
                        A[2*len(UnsafeList),0] = 1; b[2*len(UnsafeList)] = U[0,1]
                        A[2*len(UnsafeList)+1,0] = -1;  b[2*len(UnsafeList)+1] = -U[0,0]
                        A[2*len(UnsafeList)+2,1] = 1; b[2*len(UnsafeList)+2] = U[1,1]
                        A[2*len(UnsafeList)+3,1] = -1; b[2*len(UnsafeList)+3] = -U[1,0]

                        # Adding map constraints
                        for j in range(len(Map.set)):
                                A[2*len(UnsafeList)+2*len(u_s)+j,np.append(np.arange(len(u_s)),[len(u_s)+len(UnsafeList)+j])] = [Map.setDer[j](x_r,[1, 0]), Map.setDer[j](x_r,[0, 1]), -1]
                                b[2*len(UnsafeList)+2*len(u_s)+j] = -Map.CBF[j](x_r)

                        # Adding Goal based Lyapunov !!!!!!!!!!!!!!!!! Needs to be changed for a different example 
                        A[2*len(UnsafeList)+2*len(u_s)+len(Map.set),0:2] = [Goal.Lyap(x_r,[1,0]), Goal.Lyap(x_r,[0, 1])]
                        A[2*len(UnsafeList)+2*len(u_s)+len(Map.set),-1] = -1
                        b[2*len(UnsafeList)+2*len(u_s)+len(Map.set)] = 0
                        A[2*len(UnsafeList)+2*len(u_s)+len(Map.set)+1,-1] = 1
                        b[2*len(UnsafeList)+2*len(u_s)+len(Map.set)+1] = np.finfo(float).eps+1

                        H = np.zeros((len(u_s)+len(UnsafeList)+len(Map.set)+1,len(u_s)+len(UnsafeList)+len(Map.set)+1))
                        H[0,0] = 0
                        H[1,1] = 0

                        ff = np.zeros((len(u_s)+len(UnsafeList)+len(Map.set)+1,1))
                        ff[len(u_s):len(u_s)+len(UnsafeList)] = 20
                        ff[len(u_s)+len(UnsafeList):len(u_s)+len(UnsafeList)+len(Map.set)] = 10
                        ff[-1] = np.ceil(self.count/100.0)

                try:
                        uq = cvxopt_solve_qp(H, ff, A, b)
                except ValueError:
                        uq = [0,0]
                        rospy.loginfo('Domain Error in cvx')

                if uq is None:
                        uq = [0,0]
                        rospy.loginfo('infeasible QP')

                if findBestCommandAnyway and len(uq[2:len(uq)-2*len(Map.set)-1:2])>0:   # If humans are around and findbestcommand active
                        if InUnsafe:
                                self.trajs.risk.append(1.0)
                        else:
                                r = np.zeros(len(uq[2:len(uq)-2*len(Map.set)-1:2]))
                                for k in range(len(uq[2:len(uq)-2*len(Map.set)-1:2])):
                                        r[k] = min(1, max(0,1-(1-Unsafe.CBF(x_r, x_o[UnsafeList[k]][0:2]))*exp(-uq[2*k+2]*T)))
                                        Dists[k] = Unsafe.set(x_r , x_o[UnsafeList[k]][0:2])
                                self.trajs.risk.append(max(r))
                elif not findBestCommandAnyway and len(uq[2:len(uq)-len(Map.set)-1])>0:
                        r = np.zeros(len(uq[2:len(uq)-len(Map.set)-1]))
                        for k in range(len(uq[2:len(uq)-len(Map.set)-1])):
                                r[k] = min(1, max(0,1-(1-Unsafe.CBF(x_r, x_o[UnsafeList[k]][0:2]))*exp(-uq[k+2]*T)))
                                Dists[k] = Unsafe.set(x_r , x_o[UnsafeList[k]][0:2])
                        self.trajs.risk.append(max(r))

                elif not findBestCommandAnyway and len(uq) == 2:  # feasible solution is not found
                        for k in range(len(UnsafeList)):
                                        Dists[k] = Unsafe.set(x_r , x_o[UnsafeList[k]][0:2])
                        self.trajs.risk.append(-risk)  # meaning that solution is not found
                else:  # No human is around
                        self.trajs.risk.append(0.0)
                self.trajs.minDist.append(minDist)

                return uq

if __name__ == '__main__':
        ## Parameters
        findBestCommandAnyway = 1  #make this zero if you don't want to do anything if it's riskier than intended
                                   #use 1 if you want to do the best even if there is risk
        plotanimation = 0
        # Goal info
        GoalCenter = np.array([0, 0])
        rGoal = np.power(0.5,2)
        # Unsafe
        UnsafeInclude = 12    # consider obstacle if in radius
        UnsafeRadius = 0.6    #radius of unsafe sets/distance from obstacles
        # Enviroment Bounds
        env_bounds = type('', (), {})()
        env_bounds.y_min = -1.2
        env_bounds.y_max = 1
        # env_bounds.x_max = 1.25
        # env_bounds.x_min = -1.35

        l = 0.01   #bicycle model approximation parameter
        U = np.array([[-0.33,0.33],[-0.3,0.3]])
        T = 1  #Lookahead horizon
        risk = 0.1    # max risk desired
        gamma = 2       # CBF coefficient
        u1d = 0  # desired input to save energy!
        # Plotting options
        plotit = 1
        plotlanes = 1

        robot = robot(l)
        GoalInfo = robot.GoalFuncs(GoalCenter,rGoal)
        UnsafeInfo = robot.UnsafeFuncs(gamma,UnsafeRadius)
        MapInfo = robot.MapFuncs(env_bounds)


        # Process arguments
        p = argparse.ArgumentParser(description='CBF controller')
        args = p.parse_args(rospy.myargv()[1:])

        try:
	        rospy.init_node('cbf_controller')
                cbf_controller = CBF_CONTROLLER(robot,GoalInfo,UnsafeInfo,MapInfo)
                control_priod = 0.05 #[sec] we can change controll priod with this parameter.
                rospy.Timer(rospy.Duration(control_priod), cbf_controller.controller_loop_callback)
                rospy.spin()

        except rospy.ROSInterruptException:
                pass
        plottrajs(cbf_controller.trajs)


