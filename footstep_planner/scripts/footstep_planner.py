#!/usr/bin/env python

import time
import rospy
import tf
import tf2_ros
import numpy

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage

LEFT = 0
RIGHT = 1

ROBOT_NAME = None
LEFT_FOOT_FRAME_NAME = None
RIGHT_FOOT_FRAME_NAME = None

goal = [3.0, - 0.35, 0.0]
buttonLocation = [0.0, 0.0, 0.0]
goalReached = False

LEG_LINK_1 = .8342
LEG_LINK_2 = .0609
LEG_TOTAL = LEG_LINK_1 + LEG_LINK_2
HIP_HEIGHT = .71736


LEG_BEND = numpy.arccos(HIP_HEIGHT/LEG_TOTAL)
FOOT_REACH = numpy.sin(LEG_BEND)*LEG_TOTAL - .11
SIDE_REACH = FOOT_REACH - .04

#constants for offsetting steps
X_OFFSET=.040204
Y_OFFSET=.1070573

def createFootStepDataTemplate(t_transfer, t_swing, mode, id):
    temp_msg = FootstepDataListRosMessage()
    temp_msg.transfer_time = t_transfer
    temp_msg.swing_time = t_swing
    temp_msg.execution_mode = mode
    temp_msg.unique_id = id

    return temp_msg
#side: string


def placeStep(goal, isSide, whichSide):
    tan_path = goal[1]/goal[0]
    theta_path = numpy.arctan(tan_path)
    #goal is a point
    if isSide:
        reach = SIDE_REACH
        y_offset_tmp = 2*Y_OFFSET
        d=.08
    else:
        reach = FOOT_REACH
        y_offset_tmp = Y_OFFSET
        d=.1

    if whichSide == RIGHT:
        d = -d
        y_offset_tmp = -y_offset_tmp

    x=numpy.cos(theta_path)*reach - X_OFFSET
    y=numpy.sin(theta_path)*reach - y_offset_tmp - d

    return [x,y]


def createNextFootstepMsg(side,point):
    x,y = goal
    msg = createFootStepDataTemplate(1.5, 1.5, 1, -1)
    msg.footstep_data_list.append(createFootStepOffset(side, [x, y, 0.0]))
    return msg


#feedback: bool
def updateGoal(dead_recon):
    global goal, buttonLocation
    if dead_recon:
        goal[0] -= dead_recon[2]
        goal[1] -= dead_recon[1]
        goal[2] -= 0
    else:
        goal[0] = buttonLocation.x
        goal[1] = buttonLocation.y

def isGoalReached():
    return goal[0]<0.1 and abs(goal[1])<0.2

def goToButton():
    side = RIGHT
    while not isGoalReached():
        print("Next step towards")
        print(goal)
        step = placeStep(goal, False, side)
        msg = createNextFootstepMsg(side, step)
        footStepListPublisher.publish(msg) # change that into FootStepwaitForFootsteps(1)
        waitForFootsteps(1)
        updateGoal(False)
        side ^= 1
        print(".")

# Creates footstep with the current position and orientation of the foot.
def createFootStepInPlace(stepSide):
    footstep = FootstepDataRosMessage()
    footstep.robot_side = stepSide

    if stepSide == LEFT:
        foot_frame = LEFT_FOOT_FRAME_NAME

    else:
        foot_frame = RIGHT_FOOT_FRAME_NAME

    footWorld = tfBuffer.lookup_transform('world', foot_frame, rospy.Time())
    footstep.orientation = footWorld.transform.rotation
    footstep.location = footWorld.transform.translation

    return footstep


# Creates footstep offset from the current foot position. The offset is in foot frame.
def createFootStepOffset(stepSide, offset):
    footstep = createFootStepInPlace(stepSide)

    # transform the offset to world frame
    quat = footstep.orientation
    rot = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    transformedOffset = numpy.dot(rot[0:3, 0:3], offset)

    footstep.location.x += transformedOffset[0]
    footstep.location.y += transformedOffset[1]
    footstep.location.z += transformedOffset[2]

    return footstep


#!!!!
def waitForFootsteps(numberOfSteps):
    global stepCounter
    stepCounter = 0
    while stepCounter < numberOfSteps:
        rate.sleep()
    rospy.loginfo('finished set of steps')


def receivedFootStepStatus(msg):
    global stepCounter
    if msg.status == 1:
        stepCounter += 1


def goalReceived(msg):
    global buttonLocation
    buttonLocation = msg.point

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_walk_test')

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run walk_test.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")

        else:
            ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

            right_foot_frame_parameter_name = "/ihmc_ros/{0}/right_foot_frame_name".format(ROBOT_NAME)
            left_foot_frame_parameter_name = "/ihmc_ros/{0}/left_foot_frame_name".format(ROBOT_NAME)

            if rospy.has_param(right_foot_frame_parameter_name) and rospy.has_param(left_foot_frame_parameter_name):
                RIGHT_FOOT_FRAME_NAME = rospy.get_param(right_foot_frame_parameter_name)
                LEFT_FOOT_FRAME_NAME = rospy.get_param(left_foot_frame_parameter_name)

                footStepStatusSubscriber = rospy.Subscriber("/ihmc_ros/{0}/output/footstep_status".format(ROBOT_NAME), FootstepStatusRosMessage, receivedFootStepStatus)
                footStepListPublisher = rospy.Publisher("/ihmc_ros/{0}/control/footstep_list".format(ROBOT_NAME), FootstepDataListRosMessage, queue_size=1)

                tfBuffer = tf2_ros.Buffer()
                tfListener = tf2_ros.TransformListener(tfBuffer)
                
                buttonSubscriber = rospy.Subscriber("/button_topic",goalReceived,queue_size=10)
                                
                rate = rospy.Rate(30) # 10hz
                time.sleep(1)

                # make sure the simulation is running otherwise wait
                if footStepListPublisher.get_num_connections() == 0:
                    rospy.loginfo('waiting for subsciber...')
                    while footStepListPublisher.get_num_connections() == 0:
                        rate.sleep()

                if not rospy.is_shutdown():                                                                                                
                    goToButton()
                
                
            else:
                if not rospy.has_param(left_foot_frame_parameter_name):
                    rospy.logerr("Missing parameter {0}".format(left_foot_frame_parameter_name))
                if not rospy.has_param(right_foot_frame_parameter_name):
                    rospy.logerr("Missing parameter {0}".format(right_foot_frame_parameter_name))

    except rospy.ROSInterruptException:
        pass
