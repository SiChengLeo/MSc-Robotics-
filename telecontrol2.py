#!/usr/bin/env python

# Baxter_kinect visual based control system

# In the real robot, restricted by the workspace of baxter in lab, the right arm should be locked
# in a safety area to avoid crash. In the Live-Demo programm, the left arm is fixed in tuck condition.

import numpy
import argparse, sys, math
import rospy, tf2_ros

import baxter_interface
from baxter_interface import CHECK_VERSION
import FaceImage

BASE_FRAME = 'camera_depth_frame'
FRAMES = ['torso','left_shoulder','left_elbow','left_hand',
          'right_shoulder','right_elbow','right_hand']	
mirrored = rospy.get_param("~mirrored", False)
rate = rospy.get_param("~rate", 6)
test = rospy.get_param("~test", False)
# The main coordinate frame we used to control the Baxter robot in the OpenNI+NiTE
# Baxter has no leg so just use upper-bound body joint coordinate system.
# Base frame should be a static frame.In order to easy implement the model and transformation
# camera_depth_frame has been set as base frame. In the RVIZ, both set camera_depth_frame as base frame.		  

TEST_JOINT_ANGLES = dict()
TEST_JOINT_ANGLES['left'] = dict({'left_s0':0.32366,'left_s1':0.86056,'left_e0':-0.0456,
                                'left_e1':0.59326,'left_w0':1.02930,'left_w1':0.0272,'left_w2':-0.0276})
TEST_JOINT_ANGLES['right'] = dict({'right_s0':-0.4183,'right_s1':0.8862,'right_e0':0.2078,
                                'right_e1':0.5821,'right_w0':-1.35182,'right_w1':0.15684,'right_w2':0.29222})
TEST_JOINT_ANGLES['head'] = dict({'pitch':0, 'yaw':0, 'roll':0)

## 14 joint from Baxter Robot arm and RPY of the robot head.
## Redefine the initiat state of each joint when enable the robot
## in order to keep robot safety
## Coordinate transform is based on TF2 frames
def teleoperate(rate, test, mirrored):
    rate = rospy.Rate(rate)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    head = baxter_interface.Head()
    count = 0
    while not rospy.is_shutdown():
        rate.sleep()
        joint_angles = get_joint_angles(tfBuffer, test, mirrored)
        print(joint_angles)
        if joint_angles is not None:
            left.set_joint_positions(joint_angles['left'])
            right.set_joint_positions(joint_angles['right'])
            count = count + 1
            if count == 10:
                head_ang = (joint_angles['head']['yaw'])*10
            	if (head_ang > 0.8):
            		head_ang = 0.8
            	elif (head_ang < -0.8):
            		head_ang = -0.8
            	head.set_pan(head_ang)
               	count = 0
            print("updated positions")
    print("Rospy shutdown, exiting loop.")
    return True

## The sample of joint position is param rate
## The name of joint param name

def get_joint_angles(tfBuffer, test, mirrored):
    joint_angles = dict()
    joint_angles['left'] = dict()
    joint_angles['right'] = dict()
    joint_angles['head'] = dict()
    frames_all = []
    i=1
    while i < 6:
        frames = get_frame_positions(i, tfBuffer)
        i=i+1
        if frames is None:
            continue
        frames_all.append(frames)
    if not frames_all:
        # if there's a problem with tracking, don't move
        joint_angles = TEST_JOINT_ANGLES
        return joint_angles
    depth_all = []
    for frames in frames_all:
        depth_all.append(-frames['torso'][2])
    closest_user = depth_all.index(min(depth_all))
    frames = frames_all[closest_user]
    head_quat = get_frame_head(closest_user+1, tfBuffer)
    print("The number of user is:%d, The closest user is:%d\n" % (len(frames_all), closest_user))
    if test:
        # debugging test
        joint_angles = TEST_JOINT_ANGLES
    else:
        # Use normal vector method to calculate the angle between each link(direction vector)
        reh = frames['right_hand'] - frames['right_elbow']
        res = frames['right_shoulder'] - frames['right_elbow']
        rse = numpy.negative(res)
        leh = frames['left_hand'] - frames['left_elbow']
        les = frames['left_shoulder'] - frames['left_elbow']
        lse = numpy.negative(les)
        qx = head_quat[0]
        qy = head_quat[1]
        qz = head_quat[2]
        qw = head_quat[3]

        # find down vector and normals
        #print "pitch: %f,  yaw:  %f, roll:  %f"% (pitch, yaw, roll)
        nt = numpy.cross(frames['right_shoulder'] - frames['torso'], frames['left_shoulder'] - frames['torso'])
        d = numpy.cross(nt, frames['right_shoulder'] - frames['left_shoulder'])
        rns = numpy.cross(d, rse)
        lns = numpy.cross(d, lse)
        lne = numpy.cross(leh, les)
        rne = numpy.cross(reh, res)

        # normalize vectors
        reh = reh / numpy.linalg.norm(reh,ord=None, axis=None, keepdims=False)
        res = res / numpy.linalg.norm(res,ord=None, axis=None, keepdims=False)
        leh = leh / numpy.linalg.norm(leh,ord=None, axis=None, keepdims=False)
        les = les / numpy.linalg.norm(les,ord=None, axis=None, keepdims=False)
        rse = rse / numpy.linalg.norm(rse,ord=None, axis=None, keepdims=False)
        lse = lse / numpy.linalg.norm(lse,ord=None, axis=None, keepdims=False)
        nt = nt / numpy.linalg.norm(nt,ord=None, axis=None, keepdims=False)
        d = d / numpy.linalg.norm(d,ord=None, axis=None, keepdims=False)
        rns = rns / numpy.linalg.norm(rns,ord=None, axis=None, keepdims=False)
        lns = lns / numpy.linalg.norm(lns,ord=None, axis=None, keepdims=False)
        lne = lne / numpy.linalg.norm(lne,ord=None, axis=None, keepdims=False)
        rne = rne / numpy.linalg.norm(rne,ord=None, axis=None, keepdims=False)

        # solid geometry and normal vector to calculate the angle indirectly
        joint_angles['left']['left_s0'] = numpy.arccos(numpy.dot(nt,lns)) - math.pi/4.0 if mirrored else numpy.arccos(numpy.dot(nt,rns)) - math.pi*7.0/8.0# was + 0.0
        joint_angles['left']['left_s1'] = numpy.arccos(numpy.dot(d, lse)) - math.pi/2.0 if mirrored else numpy.arccos(numpy.dot(d, rse)) - math.pi/2.0
        joint_angles['left']['left_e0'] = numpy.arccos(numpy.dot(nt, lne)) if mirrored else numpy.arccos(numpy.dot(nt, rne)) - math.pi
        joint_angles['left']['left_e1'] = math.pi - numpy.arccos(numpy.dot(leh, les)) if mirrored else math.pi - numpy.arccos(numpy.dot(reh, res))
        joint_angles['left']['left_w0'] = 0.0
        joint_angles['left']['left_w1'] = 0.0
        joint_angles['left']['left_w2'] = 0.0
        joint_angles['right']['right_s0'] = numpy.arccos(numpy.dot(nt,rns)) - math.pi*7.0/8.0 if mirrored else numpy.arccos(numpy.dot(nt,lns)) - math.pi/4.0# was - pi
        joint_angles['right']['right_s1'] = numpy.arccos(numpy.dot(d, rse)) - math.pi/2.0 if mirrored else numpy.arccos(numpy.dot(d, lse)) - math.pi/2.0 if mirrored
        joint_angles['right']['right_e0'] = numpy.arccos(numpy.dot(nt, rne)) - math.pi if mirrored else numpy.arccos(numpy.dot(nt, lne))
        joint_angles['right']['right_e1'] = math.pi - numpy.arccos(numpy.dot(reh, res)) if mirrored else math.pi - numpy.arccos(numpy.dot(leh, les))
        joint_angles['right']['right_w0'] = 0.0
        joint_angles['right']['right_w1'] = 0.0
        joint_angles['right']['right_w2'] = 0.0
        joint_angles['head']['pitch'] = math.atan2(2*(qy*qz+qw*qx),qw*qw-qx*qx-qy*qy+qz*qz) if mirrored else - math.atan2(2*(qy*qz+qw*qx),qw*qw-qx*qx-qy*qy+qz*qz)
        joint_angles['head']['yaw'] = math.asin(-2*(qx*qz-qw*qy)) if mirrored else math.asin(-2*(qx*qz-qw*qy))
        joint_angles['head']['roll'] = math.atan2(2*(qx*qy+qw*qz),qw*qw+qx*qx-qy*qy-qz*qz) if mirrored else math.atan2(2*(qx*qy+qw*qz),qw*qw+qx*qx-qy*qy-qz*qz)
    return joint_angles

def get_frame_positions(user, tfBuffer):
    frame_positions = dict()
    try:
        for frame in FRAMES:
            transformation= tfBuffer.lookup_transform(BASE_FRAME, "%s_%d" % (frame, user), rospy.Time())
            translation = transformation.transform.translation
            pos = numpy.array([translation.x, translation.y, translation.z])
            frame_positions[frame] = pos
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print("Problem with kinect tracking.")
        print(e)
        return None
    return frame_positions

def get_frame_head(user, tfBuffer):
    transformation= tfBuffer.lookup_transform(BASE_FRAME, "%s_%d" % ('head', user), rospy.Time())
    rotation = transformation.transform.rotation
    quat = numpy.array([rotation.x, rotation.y, rotation.z, rotation.w])
    return quat
## Advanced functions about baxter interface showing
def main():
    print("Initializing node... ")
    rospy.init_node("teleop_head")

    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    face = FaceImage.FaceImage()
    #head_init = baxter_interface.Head()
    #head_init.set_pan(0)
    def clean_shutdown():
        print("\nExiting...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
        print("Enabling robot... ")
        rs.enable()
    rospy.on_shutdown(clean_shutdown)

    face.set_image('waiting')
    rospy.sleep(5)
    face.set_image('intro')
    rospy.sleep(5)
    face.set_image('eyes')

    teleoperate(rate, test, mirrored)


if __name__ == '__main__':
    main()



