#!/usr/bin/env python
import rospy
import tf2_ros

from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from ihmc_msgs.msg import NeckTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
from ihmc_msgs.msg import NeckDesiredAccelerationsRosMessage

from vision.msg import IntTuple


headTrajTopic="/ihmc_ros/valkyrie/control/head_trajectory"
neckTrajTopic="/ihmc_ros/valkyrie/control/neck_trajectory"
neckAccelTopic="/ihmc_ros/valkyrie/control/neck_desired_acceleration"

neck_joints=[]
seqCtr=456
pos,vel,eff=[],[],[]

def waitForValkyrie():
    if headCommandPublisher.get_num_connections() == 0:
        rospy.loginfo('waiting for subsciber...')
        while footStepListPublisher.get_num_connections() == 0:
            rate.sleep()


def errorCallback(tpl):
    print "hi error"


def getNeckJoints(states):
    global neck_joints, states_sub
    global pos, vel, eff
    registerStates= True
    if not neck_joints:
        neck_joints.append(states.name.index('lowerNeckPitch'))
        neck_joints.append(states.name.index('neckYaw'))
        neck_joints.append(states.name.index('upperNeckPitch'))
        #print neck_joints

    if registerStates:
        pos=[states.position[t] for t in neck_joints ]
        vel=[states.velocity[w] for w in neck_joints ]
        eff=[states.effort[a] for a in neck_joints ]
       # print pos

def generateSeq():
    global seqCtr
    #a = Header()
    #print a.seq

    return  seqCtr

def createTrajPtMsg(t,w,tstep):
    #initalize 1DTrajPoint
    traj_pt = TrajectoryPoint1DRosMessage()
    traj_pt.time = tstep
    #fill in
    traj_pt.position=t
    traj_pt.velocity=w
    traj_pt.unique_id=generateSeq()
    return traj_pt

def createNeckTrajMsg(T,W,tstep):
        global seqCtr    
        #initialize NeckTraj
        command = NeckTrajectoryRosMessage()
        command.unique_id=generateSeq()
        #fill in JointTraj
        #command.joint_trajectory_messages[i].unique_id=[0 for i in xrange(len(W))]
        [command.joint_trajectory_messages.append(OneDoFJointTrajectoryRosMessage()) for w in W]
        [command.joint_trajectory_messages[i].trajectory_points.append(createTrajPtMsg(T[i],W[i],tstep)) for i in xrange(len(W))]
        
        for i in xrange(len(W)):
            command.joint_trajectory_messages[i].unique_id= generateSeq()
    
        seqCtr +=1
        return command


if __name__ == '__main__':
    #global headTrajTopic, neckTrajTopic, neckAccelTopic
    try:
        rospy.init_node('head_controller')

        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        states_sub=rospy.Subscriber("/hardware_joint_states", JointState, getNeckJoints)

        #commandPublisher=rospy.Publisher(headTrajTopic, HeadTrajectoryRosMessage, queue_size=10)
        commandPublisher=rospy.Publisher(neckTrajTopic, NeckTrajectoryRosMessage, queue_size=10)
        #commandPublisher=rospy.Publisher(neckAccelTopic, NeckDesiredAccelerationsRosMessage, queue_size=10)
        px_sub=rospy.Subscriber("/target_px_error", IntTuple, errorCallback)
        
        freq = 1
        rate = rospy.Rate(freq)
        
        #wait for the j_states
        while not  neck_joints:        
            rate.sleep()
        cmd=createNeckTrajMsg([0,0.4,0],[0,0.4,0],1)#pos[0],pos[1],pos[2]
        print cmd
        commandPublisher.publish(cmd)
        
        while not rospy.is_shutdown():
            
            print "pos: ", pos
            print "vel: ", vel
            #cmd=createNeckTrajMsg([pos[0],pos[1]-1,pos[2]],[0,0,0],1)
            #print cmd
            #commandPublisher.publish(cmd)
            rate.sleep()
            
    
    except rospy.ROSInterruptException:
        pass
