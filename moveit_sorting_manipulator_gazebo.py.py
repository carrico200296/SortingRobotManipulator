#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs
import tf_conversions
import math
from sensor_msgs.msg import JointState
 
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates

global bucket
global cubes
_waypoints = []
cubes =[[0]*3]*0

## gripper control
pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)

#Create callback. This is what happens when a new message is received
def sub_cal(msg):
  global cubes
  global bucket
  #rospy.loginfo("Recieved %s", msg)

  if not cubes:
    i = 2
    while i < len(msg.pose)-1:
      cubes.append([msg.pose[i].position.x, msg.pose[i].position.y, msg.pose[i].position.z]) 
      i += 1
    bucket = [msg.pose[i].position.x, msg.pose[i].position.y, msg.pose[i].position.z]
    print("cubes size: ", len(cubes))


def gripper_set(msg):
  currentJointState = rospy.wait_for_message("/joint_states",JointState)
  print 'Received!'
  currentJointState.header.stamp = rospy.get_rostime()
  if msg =="close":
    tmp = 0.7
  else:
    tmp = 0.005
  #tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
  currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
  rate = rospy.Rate(10) # 10hz
  for i in range(3):
    pub.publish(currentJointState)
    print 'Published!'
    rate.sleep()
  print 'end!'


#Initialize publisher
rospy.Subscriber('/gazebo/model_states', ModelStates, sub_cal, queue_size=1)

def move_group_python_interface_tutorial():

  ## BEGIN_TUTORIAL
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)
 
  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()
 
  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()
 
  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("Arm")
 
 
  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=1)
 

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Waiting for RVIZ..."
  rospy.sleep(2)
  print "============ Starting tutorial "

  scene = moveit_commander.PlanningSceneInterface()
  robot = moveit_commander.RobotCommander()
 
  rospy.sleep(1)
  '''
  scene.remove_world_object('groundplane')
  scene.remove_world_object('table')
  '''
 
  group.set_goal_orientation_tolerance(0.01)
  group.set_goal_tolerance(0.01)
  group.set_goal_joint_tolerance(0.01)
  group.set_num_planning_attempts(100)

  #waypoints
  wp = []
  if not _waypoints:
    ## make the eef looks down
    start_pose = group.get_current_pose().pose
    start_pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
    wp.append(start_pose)
    
    (plan1, fraction) = group.compute_cartesian_path(
                                      wp,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
    rospy.sleep(1)
    print("execute 1")
    group.execute(plan1,wait=True)
    #time to catch the right position of the eef this
    rospy.sleep(6.)
    print ("executed")

    ## iterating through all the cubes
    for cube in cubes:
      ## above the cube
      print "============ picking a cube"
      pose_goal2 = group.get_current_pose().pose
      wp2=[]
      pose_goal2.position.x = cube[0]
      pose_goal2.position.y = cube[1]
      pose_goal2.position.z = cube[2]+0.3
      print pose_goal2
      #Create waypoints
      wp2.append(copy.deepcopy(pose_goal2))
      
      #create cartesian  plan
      (plan2, fraction) = group.compute_cartesian_path(
                                      wp2,         # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
      rospy.sleep(2.)
      print("execute 2")
      group.execute(plan2,wait=True)
      rospy.sleep(4.)

      ## opening the gripper
      print("opening the gripper")
      gripper_set("open")

      ## going down
      pose_goal3 = group.get_current_pose().pose
      pose_goal3.position.z = pose_goal2.position.z - 0.5
      wp3=[]
      wp3.append(copy.deepcopy(pose_goal3))
      (plan3, fraction) = group.compute_cartesian_path(
                                      wp3,         # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
      rospy.sleep(2.)
      print("execute going down 1")
      group.execute(plan3,wait=True)
      rospy.sleep(2.)

      ## closing the gripper
      print("closing the gripper")
      gripper_set("close")
      rospy.sleep(3.)

      ## going up
      pose_goal4 = group.get_current_pose().pose
      pose_goal4.position.z = pose_goal3.position.z + 0.8
      wp4=[]
      wp4.append(copy.deepcopy(pose_goal4))
      (plan4, fraction) = group.compute_cartesian_path(
                                      wp4,         # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
      rospy.sleep(2.)
      print("execute going up 1")
      group.execute(plan4,wait=True)
      rospy.sleep(2.)


      ## going above the bucket
      pose_goal5 = group.get_current_pose().pose
      pose_goal5.position.x = bucket[0] + 0.1
      pose_goal5.position.y = bucket[1] - 0.1
      pose_goal5.position.z = bucket[2] + 0.8
      wp5=[]
      wp5.append(copy.deepcopy(pose_goal5))
      (plan5, fraction) = group.compute_cartesian_path(
                                      wp5,         # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
      rospy.sleep(2.)
      print("execute going above the bucket")
      group.execute(plan5,wait=True)
      rospy.sleep(2.)

       ##releasing the cube
      print("opening the gripper")
      gripper_set("open")
      rospy.sleep(3.)

 
  rospy.sleep(0.5)
 
  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
 
  ## END_TUTORIAL
 
  print "============ STOPPING"
 
  R = rospy.Rate(10)
  while not rospy.is_shutdown():
    R.sleep()
 
if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
