#! /usr/bin/env python3

import rospy

import actionlib

from energybot1_lift.msg import InterfaceAction, InterfaceResult, InterfaceFeedback

import serial

class energybot1_lift(object):
    # create messages that are used to publish feedback/result
    _feedback =InterfaceFeedback()
    _result = InterfaceResult()


    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer("energybot1_lift", InterfaceAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self.serial = serial.Serial("/dev/ttyACM1",115200, timeout = 1.0)
      
    def execute_cb(self, goal):
        # helper variables
        success = True
        
        # append the seeds for the fibonacci sequence
        self._feedback.Status = "In work"
        
        # publish info to the console for the user
        #rospy.loginfo('%s: Executing, der %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
        rospy.loginfo(f"Executing, Height: {goal.Height}, status of Gripper angle:{goal.isUp}, Gripper is opened: {goal.isClosed}")
        # start executing the action
        
        String = "d;g:" + str(goal.isUp) +','+ str(goal.isClosed)+ ','+ str(goal.isUp) + ','+str(goal.Height)+','+str(goal.isUnload)+'\n'
        rospy.loginfo(String)
        self.serial.write(String.encode())
        rospy.loginfo(String)
        self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        
          
        if success:
            self._result.Ok = True
            rospy.loginfo('%s: Succeeded energybot1_lift action' )
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('energybot1_lift')
    server = energybot1_lift(rospy.get_name())
    rospy.spin()