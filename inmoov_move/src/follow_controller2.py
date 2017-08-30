#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, actionlib

from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory
from diagnostic_msgs.msg import *


class FollowController:
    """ A controller for joint chains, exposing a FollowJointTrajectory action. """

    def __init__(self, name,joints_dict,ser):
        self.interpolating = 0
        self.serial = ser #串口通讯的句柄
        self.name = name
        self.joints_dict = joints_dict

        self.trajectory_data = list()
        
        # parameters: rates and joints
        self.rate = rospy.get_param('~'+name+'/rate',50.0)
        self.joints = rospy.get_param('~elmo_driver/joints')
       
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

    def data_process(self,point):
        #rospy.loginfo(len(point.positions)) #输出joints数。
        joints_num = len(point.positions)
        a=int(point.time_from_start.secs*1.0*1e3 + point.time_from_start.nsecs*1.0/1e6)
        h_a=a>>8 
        l_a=a&0x00FF
        self.lis = list()
        self.lis.append(2) #动作编号
        self.lis.append(h_a)
        self.lis.append(l_a) #该时间点距离开始时刻的时间间隔
        for i in range(joints_num):
            #rospy.loginfo(point.time_from_start.secs*1e3 + point.time_from_start.nsecs/1e6)   	
            self.lis.append(point.positions[i]) #每个电机的当前时间点的位置数据
        length=18

        #crc16=self.calculateCRC(length)
        #self.lis.append(crc16 >> 8) # checksum
        #self.lis.append(crc16 & 0xFF)
        rospy.loginfo(self.lis)
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
        #rospy.loginfo(self.serial.isOpen())#确认串口已成功打开。
        time_num = len(self.trajectory_data) #数组长度，对应时间点的个数。
        joints_num = len(self.trajectory_data[0]) #带上编号和时间的数据长
        #rospy.loginfo(joints_num)
        for i in range(time_num):
        	rospy.loginfo(self.trajectory_data[i])
        	#for j in range(joints_num):
        		#rospy.loginfo(self.trajectory_data[i][j])
        		#self.serial.write(chr(self.trajectory_data[i][j])) #
        	#rospy.loginfo(self.trajectory_data[i])
        		#serial.write(0x02) #
        	#b = self.serial.readline()
        	#rospy.loginfo(b)
        	
        	
        #rospy.sleep(0.1) #延迟一下。
        #rospy.loginfo(self.trajectory_data[0]) 


    def executeTrajectory(self, traj): #主要的执行函数。
        rospy.loginfo("Executing trajectory")
        rospy.logdebug(traj)
        n=0
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
            while rospy.Time.now() + rospy.Duration(0.01) < start:
                rospy.sleep(0.01)

            desired = [ point.positions[k] for k in indexes ]

            endtime = start + point.time_from_start
            while rospy.Time.now() + rospy.Duration(0.01) < endtime:
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




                        #self.device.setJoint(self.joints[i],last[i])

                        #rospy.logwarn("%s:%f"  %(self.joints[i],last[i]))
                    else:
                        velocity[i] = 0
                r.sleep()
            self.data_process(point) #把point的数据传递给数据处理函数
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

