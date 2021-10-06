#!/usr/bin/env python

import roslib
roslib.load_manifest('joint_states_listener')
import rospy
from joint_states_listener.srv import *
from sensor_msgs.msg import JointState
import threading

import paho.mqtt.client as paho
broker="10.0.0.1"
port=1883

#holds the latest states obtained from joint_states messages
class LatestJointStates:

    def __init__(self):
        rospy.init_node('joint_states_listener')
        self.lock = threading.Lock()
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []
        self.thread = threading.Thread(target=self.joint_states_listener)
        self.thread.start()

    #thread function: listen for joint_states messages
    def joint_states_listener(self):
        rospy.Subscriber('joint_states', JointState, self.joint_states_callback)
        rospy.spin()


    #callback function: when a joint_states message arrives, save the values
    def joint_states_callback(self, msg):
        self.lock.acquire()
        self.name = msg.name
        self.position = msg.position
        self.velocity = msg.velocity
        self.effort = msg.effort
        self.lock.release()
        #store as string
        position_str = ", ".join(map(str, self.position))
        #publish to MQTT
        ret= client1.publish("servoarm/move", position_str)
        
#def on_publish(client,userdata,result):
#def on_connect(client,userdata,result):

#set & connect to MQTT
client1= paho.Client("control1")
#client1.on_publish = on_publish
#client1.on_connect = on_connect
client1.connect(broker,port, 60)

#run
if __name__ == "__main__":

    latestjointstates = LatestJointStates()
    print ("Ready - listening to ROS and connected to MQTT")
    rospy.spin()

