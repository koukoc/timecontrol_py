#!/usr/bin/env python3
import rospy
from FlySequence import FlightSequence 


def printTime(event):
    print(rospy.get_time())

if __name__ == '__main__':
    rospy.init_node('main',anonymous=True)
    Seq=FlightSequence()
    rospy.loginfo('sequence start')
    rospy.sleep(0.2)
    ModePassed = True
    if ModePassed:
        ModePassed = Seq.LiftOffMode()

    rospy.spin()
