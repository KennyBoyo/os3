#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState


"""
Script used to mock an OS3ProblemInput    
"""
class ManualJointAngles():
    def __init__(self, listening_topic, publish_topic):
        pass
    def get_upper_limb_angles(self, shoulder: np.array, elbow: np.array, wrist:np.array):
        # Assume all vectors are defined in the shoulder coordinate frame. All vectors are [px, py, pz]
        shoulder_rel_elbow = shoulder - elbow # Note v_shoulder = (px, py, pz) = (0,0,0) always
        wrist_rel_elbow = wrist - elbow
        mag_shoulder_rel_elbow = np.linalg.norm(shoulder_rel_elbow) # L1
        mag_wrist_rel_elbow =  np.linalg.norm(wrist_rel_elbow) # L2

        # Calculate elbow angle, theta_4 (Dot product method)
        rhs = np.dot(wrist_rel_elbow, shoulder_rel_elbow)/(mag_shoulder_rel_elbow * mag_wrist_rel_elbow)
        theta_4 = np.arccos(rhs)

        # Calculate medial rotation, theta_1
        theta_1 = np.arctan2(elbow[0], -elbow[1])
        
        # Calculate flexion/extension, theta_2
        if np.cos(theta_1) != 0:
            s2 = -elbow[1] / (np.cos(theta_1) * mag_shoulder_rel_elbow)
        else: 
            s2 = elbow[0] / (np.sin(theta_1) * mag_shoulder_rel_elbow)

        theta_2 = np.arctan2(s2, elbow[2]/mag_shoulder_rel_elbow)

        # Calculate abduction/adduction, theta_3 
        x1 = -np.sin(theta_1) * np.cos(theta_2) * wrist[0] + \
            np.cos(theta_1) * np.cos(theta_2) * wrist[1] + \
            np.sin(theta_2) * wrist[2]
        x2 = np.cos(theta_1) * wrist[0] + np.sin(theta_1) * wrist[1]

        theta_3 = np.arctan2(x1, x2)

        return np.rad2deg(np.array([theta_1, theta_2, theta_3, theta_4]))


def get_angle_to_axis(axis, b, a):
# https://math.stackexchange.com/questions/2548811/find-an-angle-to-rotate-a-vector-around-a-ray-so-that-the-vector-gets-as-close-a
    u = axis/np.linalg.norm(axis)
    
    c = a - np.dot(a, u)*u
    mag_c = np.linalg.norm(c)
    e = c
    if mag_c != 1 and np.any(c):
        e /= mag_c

    f = np.cross(u,e)

    theta = np.arctan2(np.dot(b, f), np.dot(b, e))
    return theta

if __name__ == "__main__":
    rospy.init_node("manual_joint_angles")
    rate = rospy.Rate(30) # ROS Rate at 5Hz
    pub = rospy.Publisher("/os3/synchronised_step_problem", JointState, queue_size=10)
    counter = 0

    default_msg = JointState()
    default_msg.header.frame_id = "panda_K"
    default_msg.name = ["Flexion", "Rotation", "Adduction", "Elbow"]
    default_msg.velocity = [0]*4
    default_msg.effort = [1]*4

    angles = ManualJointAngles("", "").get_upper_limb_angles(np.array([0,0,0]),
                                                       np.array([0,0,1]),
                                                       np.array([0,0,2]))
    print(angles)
    rot = 0 # y axis
    elbow = 0
    addu = 0 # x axis
    elev = np.pi/2 #np.pi/4 # z axis

    while not rospy.is_shutdown():

        counter = 0 
        rot = 0 # y axis
        elbow = 0
        addu = 0 # x axis
        elev = np.pi/2 #np.pi/4 # z axis
        while counter <30:
            counter += 1 
            default_msg.header.stamp = rospy.Time.now()
            default_msg.position = [elev,rot,addu,elbow]
            # print(default_msg.position)

            elev -= 1/30
            elev = elev%(2*np.pi)

            addu += 1/30
            addu = addu%(2*np.pi)

            pub.publish(default_msg)
            rate.sleep()

        counter = 0 
        rot = 0 # y axis
        elbow = 0
        addu = 0 # x axis
        elev = np.pi/2 #np.pi/4 # z axis
        while counter < 5 * 30:
            counter += 1 
            default_msg.header.stamp = rospy.Time.now()
            default_msg.position = [elev,rot,addu,elbow]
            # print(default_msg.position)

            addu += 1/30
            addu = addu%(2*np.pi)

            pub.publish(default_msg)
            rate.sleep()

        counter = 0 
        rot = 0 # y axis
        elbow = 0
        addu = 0 # x axis
        elev = np.pi/2 #np.pi/4 # z axis
        while counter < 5 * 30:
            counter += 1 
            default_msg.header.stamp = rospy.Time.now()
            default_msg.position = [elev,rot,addu,elbow]
            # print(default_msg.position)

            elev -= 1/30
            elev = elev%(2*np.pi)

            pub.publish(default_msg)
            rate.sleep()


        counter = 0 
        rot = 0 # y axis
        elbow = 0
        addu = 0 # x axis
        elev = np.pi/2 #np.pi/4 # z axis
        while counter < 5 * 30:
            counter += 1 
            default_msg.header.stamp = rospy.Time.now()
            default_msg.position = [elev,rot,addu,elbow]
            # print(default_msg.position)

            elev -= 1/30
            elev = elev%(2*np.pi)

            addu += 1/30
            addu = addu%(2*np.pi)

            pub.publish(default_msg)
            rate.sleep()

        counter = 0 
        rot = 0 # y axis
        elbow = 0
        addu = 0 # x axis
        elev = -np.pi/2 #np.pi/4 # z axis
        while counter < 5 * 30:
            counter += 1 
            default_msg.header.stamp = rospy.Time.now()
            default_msg.position = [elev,rot,addu,elbow]
            # print(default_msg.position)

            elev -= 1/30
            elev = elev%(2*np.pi)

            addu += 1/30
            addu = addu%(2*np.pi)

            pub.publish(default_msg)
            rate.sleep()

        pub.publish(default_msg)
        rate.sleep()