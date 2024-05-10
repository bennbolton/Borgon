#!/usr/bin/env python
from __future__ import print_function
from re import X
   
import roslib
roslib.load_manifest('cube_spotting')
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from cube_spotting.msg import cubeData
from cube_spotting.msg import cubeArray
import numpy as np;

from open_manipulator_msgs.msg import OpenManipulatorState
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.msg import KinematicsPose
from open_manipulator_msgs.srv import SetKinematicsPose
from open_manipulator_msgs.srv import GetKinematicsPose, SetKinematicsPoseRequest
from open_manipulator_msgs.srv import SetJointPosition    #   if (data.cubes[c].cube_colour=='red'):
    #     area.append(data.cubes[c].area)
    #     coX.append(data.cubes[c].normalisedCoordinateX)
    #     coY.append(data.cubes[c].normalisedCoordinateY)
from open_manipulator_msgs.srv import GetJointPosition


class cubeFinder:

  def __init__(self):

    # Where the block is in the image (start at the centre)
    self.targetX=0.5
    self.targetY=0.5

    self.foundCubes = {'blue': [], 'red': [],'yellow':[]}

    self.scanning = False

    # Whether the robot is ready to move (assume it isn't)
    self.readyToMove=False

    # Home postion for the robot to move to
    self.jointPose=[0.058,0.238,-0.380,1.738]

    # Create the subscribers
    self.image_sub = rospy.Subscriber('states',OpenManipulatorState,self.getStates)
    self.joint_state_sub = rospy.Subscriber('joint_states',JointState,self.getJoints)
    self.moving_sub = rospy.Subscriber('cubes',cubeArray,self.getTarget)

    # Create the service caller to move the robot
    self.setPose = rospy.ServiceProxy('goal_joint_space_path', SetJointPosition)
    self.setGripper = rospy.ServiceProxy('goal_tool_control', SetJointPosition)

    self.setIKOrientation = rospy.ServiceProxy('goal_task_space_path_orientation_only', SetKinematicsPose)
    self.setIKPosition = rospy.ServiceProxy('goal_task_space_path_position_only', SetKinematicsPose)

    self.jointRequest=JointPosition()
    self.jointRequest.joint_name=["joint1","joint2","joint3","joint4"]  

    self.poseRequest = KinematicsPose()

    self.cubeList = []

    # hight of cubes is 35mm, 0.035 m 
    # blocks, 0.035, 0.07, 0.105

    self.locationStackCrt = (0.005,-0.202,0.035) 

    self.moveJoints([0,-1.05,0.374,0.706])



  def scan(self, steps=7, dur=10):
    twist_list = np.linspace(0.341,-2.1, steps)
    for i, th4 in enumerate([1.99,1.5]):
      for th1 in twist_list if not i else twist_list[::-1]:

        if len(self.cubeList) > 0:
          while not self.centreCube():

            pass

        self.moveJoints((th1,-0.841, 0.377,th4), dur=dur/steps)


    # 0.341, 

  def centreCube(self, colour):
    
    pass



  def getTarget(self, data):
    self.cubeList = data.cubes
    
      
 

  # Get the robot's joint positions
  def getJoints(self,data):
    self.jointPose=data.position

  # Get data on if the robot is currently moving
  def getStates(self,data):
    if (data.open_manipulator_moving_state=='"STOPPED"'):
      self.readyToMove=True
    else:
      self.readyToMove=False
    
  def moveJoints(self, jointPositions, dur=1):

    self.jointRequest.position=jointPositions
    self.setPose(str(),self.jointRequest,dur)
    rospy.sleep(dur)

  def moveTaskPos(self, pos, dur=2):
    request = KinematicsPose()
    request.pose.position.x = pos[0]
    request.pose.position.y = pos[1]
    request.pose.position.z = pos[2]
    response = self.setIKPosition(str(), "gripper", request, dur)
    rospy.sleep(dur)
    return response
   
  def moveTaskOrien(self, rot, dur=2):
    request = KinematicsPose()
    request.pose.orientation.x = rot[0]
    request.pose.orientation.y = rot[1]
    request.pose.orientation.z = rot[2]
    request.pose.orientation.w = rot[3]
    response = self.setIKOrientation(str(), "gripper", request, dur)
    rospy.sleep(dur)
    return response

def main(args):

  ic = cubeFinder()
  rospy.init_node('cube_finding', anonymous=True)
  goodPos1 = (0.16,-0.007,0.2)
  # try:
  #   response = ic.moveTaskPos(goodPos1)
  #   print(response)
  # except rospy.ServiceException as e:
  #   print(e)
  ic.scan(7)

  # try:
  #   #rospy.spin()
  #   while not rospy.is_shutdown():
  #     # ic.scan()
  #     pass
  # except KeyboardInterrupt:
  #   print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
