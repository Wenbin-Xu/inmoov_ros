#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import serial # 引入rosserial做串口通讯。

from Elmo3 import *
from follow_controller5 import *
from joint import *
from publishers3 import *

if __name__ == "__main__":
    rospy.init_node("inmoov_controller")

    rospy.loginfo("ROS Serial init") #初始化串口节点。
 
    port_name = rospy.get_param('~port','/dev/ttyUSB0')
    baud = int(rospy.get_param('~baud','115200'))
    tim=0.1
    try:
        ser = serial.Serial(port_name,baud,timeout=tim) #创建串口句柄。
   # rospy.loginfo('open serial successful.')
    except Exception, e:
        try:
            port_name=rospy.get_param('~port','/dev/ttyUSB1')
            ser=serial.Serial(port_name,baud,timeout=tim)
        except Exception,e:
            rospy.loginfo('open serial failed.')
            ser=1
 
    robotjoints = rospy.get_param('~robotjoints')
    joints_instance = list()
    for jointname in robotjoints:
        joints_instance.append(Joint(jointname))
    elmo_controller=Elmo('elmo_controller',joints_instance)
    joints_dict = dict(zip(robotjoints, joints_instance))
    a = FollowController('body_controller', elmo_controller,joints_dict, ser) #初始化followcontroller，传递串口的句柄
    joint_state_publisher = JointStatePublisher()

    rospy.loginfo("Started FollowController ")
    #rospy.spin()
    rate = rospy.Rate(50) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
    	elmo_controller.UpdateRead()
    	joint_state_publisher.update(elmo_controller.joints_instance)
        rate.sleep()
