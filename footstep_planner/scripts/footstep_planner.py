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




LEG_LINK_1 = .8342
LEG_LINK_2 = .0609
LEG_TOTAL = LEG_LINK_1 + LEG_LINK_2

HIP_HEIGHT = .71736

def planNextFootstep():
    straight = 1
    msg = FootstepDataListRosMessage()
    msg.transfer_time = 1.5
    msg.swing_time = 1.5
    msg.execution_mode = 1
    msg.unique_id = -1
    
    
    print('wolf')
    if straight == 1:
        START_X = 0
        START_Y = 0
        END_X = 3
        END_Y = -.35
		

        A = (END_X-START_X)/(END_Y-START_Y)
        B = START_Y-(A*START_X)
        GO = 1
        ARC = numpy.arctan((END_Y-START_Y)/(END_X-START_X))
	
        A1 = numpy.arccos(HIP_HEIGHT/LEG_TOTAL)
        R = numpy.sin(A1)*LEG_TOTAL - .11
        Lx = START_X
        Ly = START_Y
        Rx = START_X
        Ry = START_Y
        d = .1
        side = RIGHT
        while END_X-Rx > .6 and END_X-Lx > .6 and GO == 1:
            msg.footstep_data_list=[]
            if side == RIGHT: 
				print(RIGHT)
				Rx = (numpy.cos(ARC)*R)-.040204 + Lx 
				Ry = (numpy.sin(ARC)*R)+.1070573 + Ly - d
				print(Rx,Ry)				
				msg.footstep_data_list.append(createFootStepOffset(RIGHT, [Rx, Ry, 0.0]))
				side=LEFT

            elif side == LEFT:
                print(LEFT)
                Lx = (numpy.cos(ARC)*R)-.040204 + Rx
                Ly = (numpy.sin(ARC)*R)-.1070573 + Ry + d
                print(Lx,Ly)
                msg.footstep_data_list.append(createFootStepOffset(LEFT, [Lx, Ly, 0.0]))
                side=RIGHT
            footStepListPublisher.publish(msg)
            rospy.loginfo('taking a step...')
            waitForFootsteps(len(msg.footstep_data_list))
	sideways = 0
#WALK SIDEWAYS
	if sideways == 1:
		
		START_X = 3
		START_Y = -.35
		END_X = 3.01
		END_Y = 0

	#	Lx = START_X
	#	Ly = START_Y
	#	Rx = START_X
	#	Ry = START_Y


		d = .08

		A = (END_X-START_X)/(END_Y-START_Y)
		B = START_Y-(A*START_X)
		GO = 1
		ARC = numpy.arctan((END_Y-START_Y)/(END_X-START_X))
	
		A1 = numpy.arccos(HIP_HEIGHT/LEG_TOTAL)
		R = numpy.sin(A1)*LEG_TOTAL - .15 
		while END_Y-Ly > .03 and GO == 1:
		
			Lx = (numpy.cos(ARC)*R)-.040204 + Rx 
			Ly = (numpy.sin(ARC)*R)-(2*.1070573) + Ry 
			msg.footstep_data_list.append(createFootStepOffset(LEFT, [Lx, Ly, 0.0]))

			if END_Y-Ry > .03:
				Rx = (numpy.cos(ARC)*R)-.040204 + Lx 
				Ry = (numpy.sin(ARC)*R)+(2*.1070573) + Ly 

				if Ry > (Ly - .16):

					Ry = (Ly - .07)
				

				msg.footstep_data_list.append(createFootStepOffset(RIGHT, [Rx, Ry, 0.0]))
			else:
				GO = 0


#WALK Straight  
	st = 0
	if st == 1:
		
		START_X = 3
		START_Y = 0
		END_X = 5
		END_Y = 0.01

	#	Lx = START_X
	#	Ly = START_Y
	#	Rx = START_X
	#	Ry = START_Y


		d = .1

		A = (END_X-START_X)/(END_Y-START_Y)
		B = START_Y-(A*START_X)
		GO = 1
		ARC = numpy.arctan((END_Y-START_Y)/(END_X-START_X))
	
		A1 = numpy.arccos(HIP_HEIGHT/LEG_TOTAL)
		R = numpy.sin(A1)*LEG_TOTAL - .108 
		while END_X-Lx > .04 and GO == 1:
		    
			Lx = (numpy.cos(ARC)*R)-.040204 + Rx 
			Ly = (numpy.sin(ARC)*R)-.1070573 + Ry + d   
			msg.footstep_data_list.append(createFootStepOffset(LEFT, [Lx, Ly, 0.0]))

			if END_X-Rx > .04:
				Rx = (numpy.cos(ARC)*R)-.040204 + Lx 
				Ry = (numpy.sin(ARC)*R)+.1070573 + Ly - d
				msg.footstep_data_list.append(createFootStepOffset(RIGHT, [Rx, Ry, 0.0]))
			else:
				GO = 0


	footStepListPublisher.publish(msg)
	rospy.loginfo('taking a step...')
			#waitForFootsteps(len(msg.footstep_data_list))
			#updateGoal()	
			
	print("finished walking")

	#return 


def walkTest():
    msg = FootstepDataListRosMessage()
    msg.transfer_time = 1.5
    msg.swing_time = 1.5
    msg.execution_mode = 0
    msg.unique_id = -1

    # walk forward starting LEFT
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.5353, 0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.4, -0.1, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.6, -0.05, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.8, -0.15, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [1.0, -0.1, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [1.2, -0.2, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [1.4, -0.15, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [1.6, -0.25, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [1.8, -0.2, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [2.0, -0.3, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [2.2, -0.25, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [2.4, -0.35, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [2.6, -0.3, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [2.8, -0.4, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [3.0, -0.35, 0.0]))

    footStepListPublisher.publish(msg)
    rospy.loginfo('walk forward...')
    waitForFootsteps(len(msg.footstep_data_list))

# Creates footstep with the current position and orientation of the foot.
def createFootStepInPlace(stepSide):
    footstep = FootstepDataRosMessage()
    footstep.robot_side = stepSide

    global rightFootWorld, leftFootWorld
    if stepSide == LEFT:
        #foot_frame = LEFT_FOOT_FRAME_NAME
        world=leftFootWorld
	
    else:
        #foot_frame = RIGHT_FOOT_FRAME_NAME
	    world=rightFootWorld


    footstep.orientation = world.transform.rotation
    footstep.location = world.transform.translation
    footstep.location.x = 0
    footstep.location.y = 0
    #footstep.location.z = 0

    return footstep

# Creates footstep offset from the current foot position. The offset is in foot frame.
def createFootStepOffset(stepSide, offset):
    footstep = createFootStepInPlace(stepSide)

    # transform the offset to world frame
    quat = footstep.orientation
    rot = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    transformedOffset = numpy.dot(rot[0:3, 0:3], offset)

    print("offset_T:",transformedOffset)
    footstep.location.x += transformedOffset[0]
    footstep.location.y += transformedOffset[1]
    footstep.location.z += transformedOffset[2]
    
    print('created msg' , footstep.location)
    return footstep

def waitForFootsteps(numberOfSteps):
    global stepCounter
    stepCounter = 0
    while stepCounter < numberOfSteps:
        rate.sleep()
    rospy.loginfo('finished set of steps')

def recievedFootStepStatus(msg):
    global stepCounter
    if msg.status == 1:
        stepCounter += 1

if __name__ == '__main__':
    global leftFootWorld, rightFootWorld
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

                footStepStatusSubscriber = rospy.Subscriber("/ihmc_ros/{0}/output/footstep_status".format(ROBOT_NAME), FootstepStatusRosMessage, recievedFootStepStatus)
                footStepListPublisher = rospy.Publisher("/ihmc_ros/{0}/control/footstep_list".format(ROBOT_NAME), FootstepDataListRosMessage, queue_size=1)

                tfBuffer = tf2_ros.Buffer()
                tfListener = tf2_ros.TransformListener(tfBuffer)

                rate = rospy.Rate(10) # 10hz
                time.sleep(1)

                # make sure the simulation is running otherwise wait
                if footStepListPublisher.get_num_connections() == 0:
                    rospy.loginfo('waiting for subsciber...')
                    while footStepListPublisher.get_num_connections() == 0:
                        rate.sleep()

                if not rospy.is_shutdown():
					# walkTest()
                    rightFootWorld = tfBuffer.lookup_transform('world', RIGHT_FOOT_FRAME_NAME, rospy.Time())
                    leftFootWorld = tfBuffer.lookup_transform('world', LEFT_FOOT_FRAME_NAME, rospy.Time())
                    planNextFootstep()
            else:
                if not rospy.has_param(left_foot_frame_parameter_name):
                    rospy.logerr("Missing parameter {0}".format(left_foot_frame_parameter_name))
                if not rospy.has_param(right_foot_frame_parameter_name):
                    rospy.logerr("Missing parameter {0}".format(right_foot_frame_parameter_name))

    except rospy.ROSInterruptException:
        pass
