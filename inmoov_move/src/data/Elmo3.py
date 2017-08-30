#!/usr/bin/env python
import rospy
from joint import *

pi=3.1415926

class Elmo():

    def __init__(self , name, joints_instance):
        self.name = name
        #self.fake = rospy.get_param('/sim', False)
        self.fake_min=[-70,-30,-20,-90,0,-70,-90,5,-20,0,-70,-90,5];
        self.fake_max=[70,30,40,90,60,70,90,85,20,60,70,90,85];
        for i in range(len(self.fake_min)):
            self.fake_min[i]=self.fake_min[i]*1.0/180*pi
            self.fake_max[i]=self.fake_max[i]*1.0/180*pi

        rospy.loginfo("Started Elmo C" )

  #      self.ctrjoints = rospy.get_param('~'+name+'/joints')
        self.joints_instance = joints_instance
        self.joint_names = list()
        self.joint_positions_cmd = list()
        self.joint_velocities_cmd = list()
        self.indexs = list()
        j=0
 #       for joint_name in rospy.get_param('~'+name+'/joints', dict()).keys():
        for joint_name in rospy.get_param('~elmo_driver/joints'):
            self.joint_names.append(joint_name)
            self.joint_positions_cmd.append((self.fake_min[j]+self.fake_max[j])/2)
            #self.joint_positions_cmd.append(0.0)
            j=j+1
            self.joint_velocities_cmd.append(0.0)
            for i in range(len(joints_instance)):
                if joints_instance[i].name == joint_name:
                    self.indexs.append(i)
                    break
        self.lasttime = rospy.Time.now().to_sec()

    def setJoint(self, joint_name,position):
        self.joint_positions_cmd[self.joint_names.index(joint_name)] = position

    def UpdateRead(self):
        #if not self.fake:
        #    pass
       # else:
        nowtime = rospy.Time.now().to_sec()
        deltatime = nowtime - self.lasttime 
        self.lasttime = nowtime
        for index in self.indexs:
            local_id = self.joint_names.index(self.joints_instance[index].name)
            self.joints_instance[index].velocity = (self.joints_instance[index].position - self.joint_positions_cmd[local_id])/deltatime
            self.joints_instance[index].position = self.joint_positions_cmd[local_id] 
        self.a=""
        for i in range(len(self.joints_instance)):
            self.a=self.a+"  "+str(self.joints_instance[i].position)

        #rospy.loginfo((self.joints_instance[0].name))

    def UpdateWrite(self):
        if not self.fake:
            pass
