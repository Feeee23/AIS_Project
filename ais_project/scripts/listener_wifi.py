#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from wifi_scan.msg import Fingerprint
from matplotlib import pyplot as plt
BSSID=['DEB0DA9B4D51','929BA9EF4B7D','3C371254A366'] #The name of our three Wifi points (Hier0023, LaptopAlt, Smartphone) #find with with $nmcli dev wifi
Distance=[2.5, 3.5] #Distance between w1, w2 in meters

def Draw(x, y): #outputs data into an pyplot
    plt.plot(x, y,'g^')
    plt.draw()
    plt.pause(0.00001)

def talker(pos): # publish the data into the topic /wifi_pos
    pub = rospy.Publisher('/wifi_pos', String, queue_size=10)
    rospy.loginfo(str(pos))
    pub.publish(str(pos))

def callback(data): #calc the position from the wifi data
    pos=[0,0]
    SigStrength=[0,0,0] #Strength of the messured Signal (S1, S2, S3) 
    for address_rssi in data.list:
        for i in range(len(BSSID)):
            if address_rssi.address==BSSID[i]: #search for the right BSSID and save the strengs
                SigStrength[i]=abs(address_rssi.rssi)

    if SigStrength[0]!=0 and SigStrength[1]!=0 and SigStrength[2]!=0: #if all 3 wifis are found
        pos[0]=Distance[0]/((SigStrength[1]/SigStrength[0])+1)     #calculate distance x
        pos[1]=Distance[1]/((SigStrength[2]/SigStrength[0])+1)     #calculate distance y
        talker(pos)
        rospy.loginfo('Pos x=%fm, Pos y=%fm', pos[0], pos[1])
        #Draw(pos[0], pos[1])
    else:
        rospy.loginfo('Not all neccesary Wifi networks found')

def listener(): #main loop, starts subscriber
    rospy.init_node('listener_wifi', anonymous=True)

    rospy.Subscriber('/wifi_fp', Fingerprint, callback, queue_size=1)

    rospy.spin()  # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    listener()
