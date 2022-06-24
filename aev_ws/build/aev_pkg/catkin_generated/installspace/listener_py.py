#!/usr/bin/env python2
import rospy
from aev_pkg.msg import ecu_feedback_msg

def callback(data):
    rospy.loginfo("msg_counter: %d" % (data.msg_counter))
    rospy.loginfo("feedbackSpeed: %d" % (data.feedbackSpeed))
    rospy.loginfo("acceleratorLevel: %d" % (data.acceleratorLevel))
    rospy.loginfo("acceleratorSwitch: %d" % (data.acceleratorSwitch))
    rospy.loginfo("brakeSwitch: %d" % (data.brakeSwitch))
    rospy.loginfo("movingDirection: %d" % (data.movingDirection))
    rospy.loginfo("turnSignal: %d" % (data.turnSignal))
    rospy.loginfo("horn: %d" % (data.horn))
    rospy.loginfo("--------------------------------------")


def listener():
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber("ECU_Feedback_Data", ecu_feedback_msg, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
