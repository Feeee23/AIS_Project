#!/usr/bin/env python
import rospy
import random
from gazebo_msgs.msg import ModelStates
from wifi_scan.msg import Fingerprint, AddressRSSI

Distance=[2.5, 3.5] #Distance between w1, w2 in meters
BSSID=['3C371254A3**','1CC63C149164','32CDA79A60F7'] #The name of our three Wifi points (FRITZBOX 7520 NN Ghz, Easybox-149130, Direct-DhC460-Series)


def talker(u): # publish the data into the topic /wifi_fp
    pub = rospy.Publisher('/wifi_fp', Fingerprint, queue_size=10)
    rospy.loginfo(u)
    pub.publish(u)

def callback(data): #creates wifi data
    ratio=[0,0]
    S=[100, 0, 0] #wifi strenght first one is set
    uncertainty=[random.uniform(-1,1),random.uniform(-1,1)] #adds an uncertanty to the sensor 
    rospy.loginfo('uncertainty'+str(uncertainty))
    ratio[0]=(Distance[0]/(data.pose[1].position.x+uncertainty[0]))-1 #calc ratio 1
    ratio[1]=(Distance[1]/(data.pose[1].position.y+uncertainty[1]))-1 #calc ratio 2
    rospy.loginfo('R= '+str(ratio))
    rospy.loginfo(' X= '+str(data.pose[1].position.x)+' Y= '+str(data.pose[1].position.y))
    rospy.loginfo(' X= '+str(data.pose[1].position.x+uncertainty[0])+' Y= '+str(data.pose[1].position.y+uncertainty[1]))
    S[1]=ratio[0]*S[0] #calc value 1
    S[2]=ratio[1]*S[0] #calc value 2
    rospy.loginfo(' S1= '+str(S[0])+' S2= '+str(S[1])+' S3= '+str(S[2]))
    send= Fingerprint() #create a msg type Fingerprint
    for i in range(3): #write all 3 wifi infos into the msg
        addr_rssi= AddressRSSI()
        addr_rssi.address=BSSID[i]
        addr_rssi.rssi=S[i]
        send.list.append(addr_rssi)
    talker(send)

def listener():  #main loop, starts subscriber
    rospy.init_node('sensor_converter', anonymous=True)

    rospy.Subscriber('/gazebo/model_states', ModelStates, callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    listener()
