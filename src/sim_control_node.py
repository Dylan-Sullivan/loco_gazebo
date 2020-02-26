#!/usr/bin/env python

import rospy
from loco_pilot.msg import Command
from geometry_msgs.msg import Wrench

cmdPub = None

def commands_callback(data):
    global cmdPub
    (yaw, throttle, _, pitch, _, _) = data.axes
    msg = Command()

    msg.pitch = pitch
    msg.yaw = yaw*-1
    msg.throttle = throttle

    cmdPub.publish(msg)


def thrust_apply():
    rospy.init_node('sim_control_node', anonymous=True)
    rate=rospy.Rate(10) # 10 Hz

    apply_left=rospy.Publisher("/left_thrust",geometry_msgs/Wrench, queue_size=5)
    apply_vertical=rospy.Publsiher("/vertical_thrust",geometry_msgs/Wrench, queue_size=5)
    apply_right=rospy.Publisher("/right_thrust",geometry_msgs/Wrench, queue_size=5)
    apply_drag=rospy.Publsiher("/drag_force",geometry_msgs/Wrench, queue_size=5)

    rospy.Subscriber("/loco/command", Command, commands_callback)


    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        spinner()
    except rospy.ROSInterruptException:
        pass
