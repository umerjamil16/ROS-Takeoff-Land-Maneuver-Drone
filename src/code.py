#! /usr/bin/env python
import rospy

import actionlib

from actionlib_tutorials.msg import FibonacciFeedback, FibonacciResult, FibonacciAction
#from actionlib import TestAction.msg
#Test.action is file name
#from TestAction.msg import TestActionFeedback, TestActionResult, TestAction
from actionlib.msg import TestFeedback, TestResult, TestAction

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class FibonacciClass(object):

  # create messages that are used to publish feedback/result
  _feedback = TestFeedback()
  _result   = TestResult()

  def __init__(self):
    # creates the action server
    self._as = actionlib.SimpleActionServer("drone_sq_action_server", TestAction, self.goal_callback, False)
    self._as.start()

  def drone_takeoff(self):
    #Create a msg to takeoff the drone
    takeoff_pub = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
    #Drone taking off
    i = 0
    while not i == 3:
        takeoff_pub.publish(Empty())
        rospy.loginfo("Taking off the drone...")
        rospy.sleep(1)
        i += 1
  move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

  def drone_move_sq(self):
    #Drone Moving
    #Create a msg to move a drone
#    for i in range(0, side_time):
    move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    move_msg = Twist()
    move_msg.linear.x = 1
    move_msg.angular.z = 1
    move_pub.publish(move_msg)
    rospy.loginfo("Moving around...")


  def drone_land(self):
    #Drone Landing
    #Create a msg to land the drone
    land_pub = rospy.Publisher('/drone/land', Empty, queue_size=1)
    i = 0
    while not i == 3:
        #first stop the drone twist
        move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        move_msg = Twist()
        move_msg.linear.x = 1
        move_msg.angular.z = 1
        move_pub.publish(move_msg)
        land_pub.publish(Empty())
        rospy.loginfo("Landing the drone...")
        rospy.sleep(1)
        i += 1

  def goal_callback(self, goal):
    # this callback is called when the action server is called.
    # this is the function that computes the Fibonacci sequence
    # and returns the sequence to the node that called the action server

    # helper variables
    r = rospy.Rate(1)
    success = True

    #get the flight side time
    side_time = goal.goal

    # append the seeds for the fibonacci sequence
#    self._feedback.sequence = []

    # publish info to the console for the user
#    rospy.loginfo('"fibonacci_as": Executing, creating fibonacci sequence of order %i with seeds %i, %i' % ( goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))


    self.drone_takeoff()
    for i in range(0, side_time):
        self.drone_move_sq()
        rospy.sleep(1)
        # build and publish the feedback message
        self._feedback.feedback = i
        self._as.publish_feedback(self._feedback)
#    rospy.sleep()
    self.drone_land()

    # starts calculating the Fibonacci sequence
#    fibonacciOrder = goal.order
 #   for i in xrange(1, fibonacciOrder):

      # check that preempt (cancelation) has not been requested by the action client
    if self._as.is_preempt_requested():
        rospy.loginfo('The goal has been cancelled/preempted')
        # the following line, sets the client in preempted state (goal cancelled)
        self._as.set_preempted()
        success = False
        # we end the calculation of the Fibonacci sequence
      #  break

      # builds the next feedback msg to be sent
      # the sequence is computed at 1 Hz frequency
    r.sleep()

    # at this point, either the goal has been achieved (success==true)
    # or the client preempted the goal (success==false)
    # If success, then we publish the final result
    # If not success, we do not publish anything in the result
    if success:
        self._result.result = 1
        rospy.loginfo('Succeeded in drone man_ ' )
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
  rospy.init_node('dron_move_sq_node')
  FibonacciClass()
  rospy.spin()