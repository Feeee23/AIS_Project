#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import String

phi=0 #orientation angle
phi_old=0 # old orientation angle
timestamp=0 #last time 

def talker(u): # publish the data into the topic /sensor_data 
    pub = rospy.Publisher('/sensor_data', String, queue_size=10)
    rospy.loginfo(str(u))
    pub.publish(str(u))

def callback(data): #converts Sensor data
    s=[data.linear_acceleration.x, data.angular_velocity.z] #needed sensordata is stored
    u=[0,0]
    rospy.loginfo('\n----------------------\n'+str(data))
    global timestamp
    if timestamp==0: #at the first time set timestamp to now
        timestamp= data.header.stamp.secs
        pass
    t=data.header.stamp.secs-timestamp  #calc deltatime
    timestamp=data.header.stamp.secs #remember timestamp
    global phi_old
    global phi
    phi_old=phi #remember the old angle
    phi=phi_old+s[1]*t #calc new angle (integration)
    rospy.loginfo('Angle =='+str(phi)+'angular velocity'+str(s[1]))
    u[0]=math.cos(phi_old)-math.sin(phi_old)*(phi-phi_old) #linearisation acceleration x global
    u[1]=math.sin(phi_old)+math.cos(phi_old)*(phi-phi_old) #linearisation acceleration y global
    talker(u)

def listener(): #main loop, starts subscriber
    rospy.init_node('sensor_converter', anonymous=True)

    rospy.Subscriber('/imu', Imu, callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    listener() 
