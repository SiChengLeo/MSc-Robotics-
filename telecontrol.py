#!/usr/bin/env python

# Baxter_kinect visual based control system

# In the real robot, restricted by the workspace of baxter in lab, the right arm should be locked
# in a safety area to avoid crash. In the Live-Demo programm, the left arm is fixed in tuck condition.

import numpy 
import argparse, sys, math
import rospy, tf2_ros
import baxter_interface
from baxter_interface import CHECK_VERSION

BASE_FRAME = 'camera_depth_frame'
FRAMES = ['torso','left_shoulder','left_elbow','left_hand',
          'right_shoulder','right_elbow','right_hand']

arg_fmt = argparse.RawDescriptionHelpFormatter
parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                 description=main.__doc__)
parser.add_argument(
    '-r', '--rate', type=int, default=10,
    help='rate to sample the joint positions'
    )

parser.add_argument(
    '-t', '--test', type=bool, default=False,
    help='use hardcoded test joint angles'
    )
parser.add_argument(
    '-u', '--user', type=int, default=1,
    help='kinect user number to use'
)

parser.add_argument(
    '-m', '--mirrored', type=bool, default=False,
    help='mirror the teleoperators movements'
)

TEST_JOINT_ANGLES = dict()
TEST_JOINT_ANGLES['left'] = dict({'left_s0':0.0,'left_s1':0.0,'left_e0':0.0,
                                'left_e1':0.0,'left_w0':0.0,'left_w1':0.0,'left_w2':0.0})
TEST_JOINT_ANGLES['right'] = dict({'right_s0':0.0,'right_s1':0.0,'right_e0':0.0,
                                'right_e1':0.0,'right_w0':0.0,'right_w1':0.0,'right_w2':0.0})


## 14 joint from Baxter Robot and initiat state of joint




def teleoperate(rate, user, test, mirrored):

    rate = rospy.Rate(rate)  #param rate at which to sample joint positions in ms
    #  make these attributes of a class
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    right = baxter_interface.Limb('right')
    left = baxter_interface.Limb('left')

    while not rospy.is_shutdown():
        rate.sleep()
        joint_angles = get_joint_angles(user, tfBuffer, test, mirrored)
        print(joint_angles)
        if joint_angles is not None:
            left.set_joint_positions(joint_angles['left'])
            right.set_joint_positions(joint_angles['right'])
            print("updated positions")
    print("Rospy shutdown, exiting loop.")
    return True

def get_joint_angles(user, tfBuffer, test, mirrored):
    """
    """
    joint_angles = dict()
    joint_angles['left'] = dict()
    joint_angles['right'] = dict()

    frames = get_frame_positions(user, tfBuffer)

    if frames is None and not test:
        return None

    if test:
        joint_angles = TEST_JOINT_ANGLES
    else:
        # Calculate the angle of each joint by vector. Pose of each joint is the pose of coordinate.

        reh = frames['right_hand'] - frames['right_elbow']
        res = frames['right_shoulder'] - frames['right_elbow']
        rse = numpy.negative(res)
        leh = frames['left_hand'] - frames['left_elbow']
        les = frames['left_shoulder'] - frames['left_elbow']
        lse = numpy.negative(les)
        
        # find down vector and normals
        nt = numpy.cross(frames['right_shoulder'] - frames['torso'], frames['left_shoulder'] - frames['torso'])
        d = numpy.cross(nt, frames['right_shoulder'] - frames['left_shoulder'])
        rns = numpy.cross(d, rse) 
        lns = numpy.cross(d, lse)
        lne = numpy.cross(leh, les)
        rne = numpy.cross(reh, res)
        
        # normalize the vectors
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

        # To control the robot, we should to calculate the angle of each joint
        # when we get each joint angle we can control the robot motion by kinemics
        # calculate the angular of each joint, the angular is equals to the angular between 
        # each related vector constructed by the key point of human body. 
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
	
def main():

    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("teleoperation")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
        print("Enabling robot... ")
        rs.enable()
    rospy.on_shutdown(clean_shutdown)

    teleoperate(args.rate, args.user, args.test, args.mirrored)

if __name__ == '__main__':
    main()
