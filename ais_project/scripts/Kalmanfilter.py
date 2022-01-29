#!/usr/bin/env python
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
R=np.array([[25,0,0,0], #Observation Error
            [0,25,0,0],
            [0,0,0,0],
            [0,0,0,0]])

def Draw():
    plt.plot(LPos[0],LPos[1],'g^', WPos[0],WPos[1], 'ko', EPos[0], EPos[1], 'r+')
    plt.draw()
    plt.pause(0.00001)
    

def NewCorrection():
    global KGain, LPos, PC, counter
    KGain=PC/(PC+R)     #Calculate new kalman Gain
    np.nan_to_num(KGain, False) #o/o = nan -> set to 0
    rospy.loginfo('diff'+str(WPos-PPos)+'k*diff'+str(np.matmul(KGain,(WPos-PPos))))
    LPos=PPos+np.matmul(KGain,(WPos-PPos)) #Calculate Corrected Position
    PC=np.matmul((np.identity(4)-KGain),PC)
    rospy.loginfo('\nKalmanGain= \n'+str(KGain) + '\nCorrected Pos=\n' +str(LPos)+ '\nCorrected PC= \n'+str(PC)+'\n-------------------------\n')
    counter=counter+1
    if (counter%10)==0:
        Draw()


def NewPrediction(u):
    global TimeStamp
    if TimeStamp==0:
        TimeStamp=time.process_time()
        pass
    t=time.process_time()-TimeStamp
    TimeStamp=time.process_time()
    A=np.array([[1, 0, t, 0],
                 [0, 1, 0, t],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])
    B=np.array([[0.5*t*t, 0],
                [0, 0.5*t*t],
                [t, 0],
                [0, t]])
    global PPos, PC
    PPos= np.matmul(A,LPos)+np.matmul(B,u)          #New Prediction A*x+B*U
    PC= np.matmul(np.matmul(A, PC), A.transpose())+np.array([[10,0,0,0],[0,10,0,0],[0,0,0,0],[0,0,0,0]])  #new Predicted Process Covariance matrix
    rospy.loginfo('\nPredicted Pos=\n'+str(PPos)+'\nPC= \n'+str(PC)+'Time'+str(t))
    NewCorrection()

def WifiPos(data): #save the new Wifi Position
    global WPos
    rospy.loginfo('Test:'+str(data))
    WPos[0]=float(str(data).split(',')[0].split('[')[1])
    WPos[1]=float(str(data).split(',')[1].split(']')[0])

def SensorData(data): #save the new Sensor Position
    u=[0,0] # Sensor accerleration
    u[0]=float(str(data).split(',')[0].split('[')[1])
    u[1]=float(str(data).split(',')[1].split(']')[0])
    rospy.loginfo('\nWifiData=\n'+str(WPos)+'\n SensorDATA=\n'+str(u))
    NewPrediction(u)

def ExactData(data):
    EPos[0]=data.pose[1].position.x
    EPos[1]=data.pose[1].position.y
    rospy.loginfo('\nEPos'+str(EPos))

def listener():
    rospy.init_node('Kalmanfilter', anonymous=True)

    rospy.Subscriber('/wifi_pos', String, WifiPos)
    rospy.Subscriber('/gazebo/model_states', ModelStates, ExactData) #Exact Position, only for Simulation
    rospy.Subscriber('/sensor_data', String, SensorData)
    rospy.spin()  # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    listener()
