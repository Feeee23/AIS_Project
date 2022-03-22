#!/usr/bin/env python
from concurrent.futures import process
import numpy as np
import time
import rospy
from matplotlib import pyplot as plt
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates

TimeStamp=0
counter=0
LPos=np.array([0.0,0.0,0.0,0.0]) # last Position
WPos=np.array([0.0,0.0,0.0,0.0]) # Wifi Position
PPos=np.array([0.0,0.0,0.0,0.0]) # Predicted Position
EPos=np.array([0.0,0.0,0.0,0.0]) #Exact Position
PC=np.array([[100,0,0,0], #Process Covarience 
             [0,100,0,0],
             [0,0,0,0],
             [0,0,0,0]])
KGain=np.array([[0,0,0,0], #Kalman Gain
                [0,0,0,0],
                [0,0,0,0],
                [0,0,0,0]])
R=np.array([[4,0,0,0], #Observation Error
            [0,4,0,0],
            [0,0,0,0],
            [0,0,0,0]])
PN=np.array([[.3,0,0,0], #Process Noice
                       [0,.3,0,0],
                       [0,0,0,0],
                       [0,0,0,0]])

def Draw(): #outputs data into an pyplot
    plt.plot(LPos[0],LPos[1],'g^', WPos[0],WPos[1], 'ko', EPos[0], EPos[1], 'r+')
    plt.draw()
    plt.pause(0.00001)
    

def NewCorrection(): #creates the corrected position 
    global KGain, LPos, PC, counter
    KGain=PC/(PC+R)     #Calculate new Kalman Gain
    np.nan_to_num(KGain, False) #0/0 = nan -> set to 0
    LPos=PPos+np.matmul(KGain,(WPos-PPos)) #Calculate Corrected Position PPos+KGain(WPos-PPos)
    PC=np.matmul((np.identity(4)-KGain),PC) #Calc updated PC matrix
    rospy.loginfo('\nKalmanGain= \n'+str(KGain) + '\nCorrected Pos=\n' +str(LPos)+ '\nCorrected PC= \n'+str(PC)+'\n-------------------------\n')
    counter=counter+1
    if (counter%10)==0: #draw every 10. prediction
        Draw()


def NewPrediction(u): #makes a new prediction for Position and Process Covarience Matrix
    global TimeStamp
    if TimeStamp==0: #at the first time set timestamp to now
        TimeStamp=time.process_time()
        pass
    t=time.process_time()-TimeStamp #calc deltatime
    TimeStamp=time.process_time() #remember timestamp
    A=np.array([[1, 0, t, 0], #Integration matrix 1
                 [0, 1, 0, t],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])
    B=np.array([[0.5*t*t, 0], #Integration matrix 2
                [0, 0.5*t*t],
                [t, 0],
                [0, t]])
    global PPos, PC
    PPos= np.matmul(A,LPos)+np.matmul(B,u)          #New Prediction A*x+B*u
    PC= np.matmul(np.matmul(A, PC), A.transpose())+PN  #new Predicted Process Covariance matrix A*PC*A^T+PN
    rospy.loginfo('\nPredicted Pos=\n'+str(PPos)+'\nPC= \n'+str(PC)+'Time'+str(t))
    NewCorrection()

def WifiPos(data): #save the new Wifi Position
    global WPos
    rospy.loginfo('WiFi data:'+str(data))
    WPos[0]=float(str(data).split(',')[0].split('[')[1])
    WPos[1]=float(str(data).split(',')[1].split(']')[0])

def SensorData(data): #save the new Sensor Position
    u=[0,0] # Sensor accerleration
    u[0]=float(str(data).split(',')[0].split('[')[1])
    u[1]=float(str(data).split(',')[1].split(']')[0])
    rospy.loginfo('\nWifiData=\n'+str(WPos)+'\n SensorDATA=\n'+str(u))
    NewPrediction(u)

def ExactData(data): #saves the exact Position only for Simulation
    EPos[0]=data.pose[1].position.x
    EPos[1]=data.pose[1].position.y
    rospy.loginfo('\nEPos'+str(EPos))

def listener():#main loop, starts subscriber
    rospy.init_node('Kalmanfilter', anonymous=True)

    rospy.Subscriber('/wifi_pos', String, WifiPos,queue_size=1)
    #rospy.Subscriber('/gazebo/model_states', ModelStates, ExactData, queue_size=1) #Exact Position, only for Simulation
    rospy.Subscriber('/sensor_data', String, SensorData, queue_size=1)
    rospy.spin()  # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    listener()
