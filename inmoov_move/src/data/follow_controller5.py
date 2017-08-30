#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, actionlib, serial

from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory
from diagnostic_msgs.msg import *

pi=3.1415926

class FollowController:
    """ A controller for joint chains, exposing a FollowJointTrajectory action. """

    def __init__(self, name,device,joints_dict,ser):
        self.interpolating = 0
        self.serial = ser #串口通讯的句柄
        self.name = name
        self.joints_dict = joints_dict
        self.device=device
        self.trajectory_data = list()
        
        # parameters: rates and joints
        self.rate = rospy.get_param('~'+name+'/rate',50.0)
        self.joints = rospy.get_param('~elmo_driver/joints')
        self.fakejoints=["left_finger_one","left_finger_two","left_finger_three","left_finger_four","left_finger_five","left_hand","left_bicep",
        "left_bicep_rotate","left_shoulder_side","left_shoulder_up","head_leftright","head_updown","right_bicep","right_bicep_rotate","right_shoulder_side",
        "right_shoulder_up","mouth","eye","right_finger_one","right_finger_two","right_finger_three","right_finger_four","right_finger_five","right_hand",
        "waist_rotate","waist_lean","waist_front"];

        self.fake_min=list();
        self.fake_max=list();
        self.fake_min=[0,0,0,0,0,-20,5,-90,0,-70,-90,-20,5,-90,0,-70,0,0,0,0,0,0,0,255,-70,-30,255];
        self.fake_max=[0,0,0,0,0,20,85,90,60,70,90,40,85,90,60,70,0,0,0,0,0,0,0,256,70,30,256];
        for i in range(len(self.fake_min)):
            self.fake_min[i]=self.fake_min[i]*1.0/180*pi
            self.fake_max[i]=self.fake_max[i]*1.0/180*pi

        self.fakejoints_min=dict(zip(self.fakejoints,self.fake_min))
        self.fakejoints_max=dict(zip(self.fakejoints,self.fake_max))
        #rospy.loginfo(self.fake_min)
        #rospy.loginfo(self.fake_max)


        # action server
        actionname= rospy.get_param('~'+name+'/action_name','follow_joint_trajectory')
        self.server = actionlib.SimpleActionServer('/'+name+'/'+actionname , FollowJointTrajectoryAction, execute_cb=self.actionCb, auto_start=False)

        rospy.loginfo("Started FollowController ("+name +"). Joints: " + str(self.joints) + " on C" )

        self.server.start()

    def actionCb(self, goal): #回调函数
        rospy.loginfo(self.name + ": Action goal recieved.")
        traj = goal.trajectory

        if set(self.joints) != set(traj.joint_names):
            for j in self.joints:
                if j not in traj.joint_names:
                    msg = "Trajectory joint names does not match action controlled joints." + str(traj.joint_names)
                    rospy.logerr(msg)
                    self.server.set_aborted(text=msg)
                    return
            rospy.logwarn("Extra joints in trajectory")
            rospy.logwarn((traj.joint_names))

        if not traj.points:
            msg = "Trajectory empty."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joints]
        except ValueError as val:
            msg = "Trajectory invalid."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        if self.executeTrajectory(traj):   
            self.server.set_succeeded()
        else:
            self.server.set_aborted(text="Execution failed.")

        rospy.loginfo(self.name + ": Done.")
    
    def commandCb(self, msg):
        # don't execute if executing an action
        if self.server.is_active():
            rospy.loginfo(self.name+": Received trajectory, but action is active")
            return
        self.executing = True
        self.executeTrajectory(msg)
        self.executing = False    

    def data_process(self,point,traj,n):
        #rospy.loginfo(len(point.positions)) #输出joints数。
        joints_num = len(self.fakejoints)
        a=int(point.time_from_start.secs*1.0*1e3 + point.time_from_start.nsecs*1.0/1e6)
        h_a=a>>8 
        l_a=a&0x00FF
        self.lis = list()
        self.lis.append(n) #动作编号
        self.lis.append(h_a)
        self.lis.append(l_a) #该时间点距离开始时刻的时间间隔
        dic=dict(zip(traj.joint_names,point.positions))
        for i in range(joints_num):
        	#rospy.loginfo(self.fakejoints[i])
        	if dic.has_key(self.fakejoints[i]):
                #self.lis.append(dic[self.fakejoints[i]])
        	    #rospy.loginfo(self.fakejoints[i]);
        	    tem=(dic[self.fakejoints[i]]-self.fakejoints_min[self.fakejoints[i]])/(self.fakejoints_max[self.fakejoints[i]]-self.fakejoints_min[self.fakejoints[i]])
        	    tem=int(tem*253)+1
        	    self.lis.append(tem)
        	else:
        	    self.lis.append(0)
        length=joints_num+3

        crc16=self.calculateCRC(length)
        self.lis.append(crc16 >> 8) # checksum
        self.lis.append(crc16 & 0xFF)
        #rospy.loginfo(self.lis)
        #rospy.loginfo(len(self.lis))
        self.trajectory_data.append(self.lis)
        self.lis = []

    def calculateCRC(self,length):
    	temp=0xFFFF
    	for i in range(0,length):
    		#rospy.loginfo(self.lis[i])
    		temp=temp ^ self.lis[i]
    		for j in range(1,9):
    			flag=temp & 0x0001
    			temp >>= 1
    			if flag:
    				temp ^= 0xA001
    	temp2=temp >> 8
    	temp=(temp << 8) | temp2
    	temp &= 0xFFFF
    	return temp

    def serial_send(self):
        tim=0
        self.judge=0
        #rospy.loginfo(self.serial.isOpen())#确认串口已成功打开。
        time_num = len(self.trajectory_data) #数组长度，对应时间点的个数。
        joints_num = len(self.trajectory_data[0]) #带上编号和时间的数据长
        #rospy.loginfo(joints_num)'
        if type(self.serial)==type(1):
            try:
                port_name = rospy.get_param('~port','/dev/ttyUSB0')
                baud = int(rospy.get_param('~baud','115200'))
                tim=0.1
                self.serial = serial.Serial(port_name,baud,timeout=tim) #创建串口句柄。
   # rospy.loginfo('open serial successful.')
            except Exception, e:
                try:
                    port_name=rospy.get_param('~port','/dev/ttyUSB1')
                    self.serial=serial.Serial(port_name,baud,timeout=tim)
                except Exception,e:
                    rospy.loginfo('open serial failed.')
                    self.serial=1
        for i in range(time_num):
            for j in range(joints_num):
                try:
                    self.serial.write(chr(self.trajectory_data[i][j]))
                except AttributeError:
                    rospy.logwarn('ser has no attribute related to serial.')
                    break
            try:
                rospy.loginfo(self.trajectory_data[i])
                b=self.serial.readline()
                rospy.loginfo(b)
                rospy.sleep(0.1)
            except AttributeError:
                rospy.logwarn('ser has no attribute related to serial.')
                break




        #rospy.loginfo(self.trajectory_data[i])

            
        self.trajectory_data=list()

        #rospy.loginfo(self.trajectory_data[0]) 


    def executeTrajectory(self, traj): #主要的执行函数。
        rospy.loginfo("Executing trajectory")
        n=input("number of action:")

        rospy.logdebug(traj)
        # carry out trajectory
        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joints]
        except ValueError as val:
            rospy.logerr("Invalid joint in trajectory.")
            return False

        # get starting timestamp, MoveIt uses 0, need to fill in
        start = traj.header.stamp
        if start.secs == 0 and start.nsecs == 0:
            start = rospy.Time.now()

        r = rospy.Rate(self.rate)
        last = [ self.joints_dict[joint].position for joint in self.joints ]
        for point in traj.points: #按轨迹中的时间点输出
            #rospy.loginfo(point) ##able to print out the states of each joint 
            #rospy.loginfo(n)
            #n=n+1
            desired = [ point.positions[k] for k in indexes ]
            endtime = start + point.time_from_start
            while rospy.Time.now() + rospy.Duration(0.01) < start:
                rospy.sleep(0.01)
            while rospy.Time.now() + rospy.Duration(0.01) < endtime:
               # rospy.logwarn("456")
                err = [ (d-c) for d,c in zip(desired,last) ]
                velocity = [ abs(x / (self.rate * (endtime - rospy.Time.now()).to_sec())) for x in err ]
                rospy.logdebug(err)
                for i in range(len(self.joints)):
                    if err[i] > 0.001 or err[i] < -0.001:
                        cmd = err[i] 
                        top = velocity[i]
                        if cmd > top:
                            cmd = top
                        elif cmd < -top:
                            cmd = -top
                        last[i] += cmd

                    else:
                        velocity[i] = 0
                    #rospy.logwarn("123")
                    self.device.setJoint(self.joints[i],last[i])
                r.sleep()
            self.data_process(point,traj,n) #把point的数据传递给数据处理函数
        self.serial_send();
        return True

    def active(self):
        """ Is controller overriding servo internal control? """
        return self.server.is_active() or self.executing

    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        if self.active():
            msg.values.append(KeyValue("State", "Active"))
        else:
            msg.values.append(KeyValue("State", "Not Active"))
        return msg

