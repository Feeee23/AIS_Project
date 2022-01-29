#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import String

phi=0
phi_old=0
timestamp=0 #last time 

def talker(u):
    pub = rospy.Publisher('/sensor_data', String, queue_size=10)
    rospy.loginfo(str(u))
    pub.publish(str(u))

def callback(data):
    s=[data.linear_acceleration.x, data.angular_velocity.z] #sensordata
    u=[0,0]
    rospy.loginfo('\n----------------------\n'+str(data))
    global timestamp
    if timestamp==0: #at the first time set timestamp
        timestamp= data.header.stamp.secs
        pass
    t=data.header.stamp.secs-timestamp  #deltatime
    timestamp=data.header.stamp.secs
    global phi_old
    global phi
    phi_old=phi 
    phi=phi_old+s[1]*t
    rospy.loginfo('o =='+str(phi)+'winkelgeschw'+str(s[1]))
    u[0]=math.cos(phi_old)-math.sin(phi_old)*(phi-phi_old) #linearisation acceleration x global
    u[1]=math.sin(phi_old)+math.cos(phi_old)*(phi-phi_old) #linearisation acceleration y global
    #rospy.loginfo('x='+str(u[0])+' y='+str(u[1]))
    talker(u)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('sensor_converter', anonymous=True)

    rospy.Subscriber('/imu', Imu, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
