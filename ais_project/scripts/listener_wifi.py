#!/usr/bin/env python
import imp
from turtle import distance
import rospy
from std_msgs.msg import String
from wifi_scan.msg import Fingerprint
#find the  with $nmcli dev wifi
BSSID=['3C371254A3**','1CC63C149164','32CDA79A60F7'] #The name of our three Wifi points (FRITZBOX 7520 NN Ghz, Easybox-149130, Direct-DhC460-Series)
#BSSID=['DEB0DA9B4D51','929BA9EF4B7D','3C371254A3**'] #The name of our three Wifi points (old Laptop, Smartphone, Router)
Distance=[2.8, 3.8] #Distance between w1, w2 in meters

def talker(pos):
    pub = rospy.Publisher('/wifi_pos', String, queue_size=10)
    rospy.loginfo(str(pos))
    pub.publish(str(pos))

def callback(data):
    pos=[0,0]
    SigStrength=[0,0,0] #Strength of the messured Signal (S1, S2, S3) 
    rospy.loginfo(rospy.get_caller_id()+ 'All I have found------------------------------')
    for address_rssi in data.list:
        rospy.loginfo(data)
        for i in range(len(BSSID)):
            if address_rssi.address==BSSID[i]:
                SigStrength[i]=abs(address_rssi.rssi)
                #rospy.loginfo('Test%s%i%i',BSSID[i], SigStrength[i], address_rssi.rssi)
        
        #rospy.loginfo('\nBSSID %s', address_rssi.address) # sagt wer sendet nicht netzwerk sonder phy gerät basic service set identifier https://www.juniper.net/documentation/en_US/junos-space-apps/network-director3.7/topics/concept/wireless-ssid-bssid-essid.html
        #rospy.loginfo('The Signal strength is: %s\n', str(address_rssi.rssi)) #je näher 0 desto besser https://www.speedcheck.org/de/wiki/rssi/
    #calculate distances
    if SigStrength[0]!=0 and SigStrength[1]!=0 and SigStrength[2]!=0:
        x=Distance[0]/((SigStrength[1]/SigStrength[0])+1)
        y=Distance[1]/((SigStrength[2]/SigStrength[0])+1)
        pos[0]=x
        pos[1]=y
        talker(pos)
        rospy.loginfo('Pos x=%fm, Pos y=%fm', x, y)
    else:
        rospy.loginfo('Not all neccesary Wifi networks found')

def listener():
    rospy.init_node('listener_wifi', anonymous=True)

    rospy.Subscriber('/wifi_fp', Fingerprint, callback)

    rospy.spin()  # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    listener()
