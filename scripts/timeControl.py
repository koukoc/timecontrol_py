#!/usr/bin/env python3
import rospy
from FlySequence import FlightSequence 


def printTime(event):
    print(rospy.get_time())

if __name__ == '__main__':
    rospy.init_node('main',anonymous=True)
    # rospy.Timer(rospy.Duration(0.1),printTime)

    Seq=FlightSequence()
    # ModePassed = Seq.Hold()
    ModePassed = True
    if ModePassed:
        ModePassed = Seq.LiftOffMode()
    if ModePassed:
        ModePassed = Seq.Separate()

    if  ModePassed:
        ModePassed = Seq.Autopilot()

    rospy.spin()
