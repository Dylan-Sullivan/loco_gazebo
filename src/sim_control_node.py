#!/usr/bin/env python

import rospy
from loco_pilot.msg import Command
from geometry_msgs.msg import Wrench

apply_left=None
apply_vertical=None
apply_right=None
apply_drag=None

def commands_callback(data):
    global apply_left
    global apply_vertical
    global apply_right
    global apply_drag

    pitch=data.pitch
    yaw=data.yaw
    throttle=data.throttle

    left=Wrench()
    right=Wrench()

    left.force.x=1.0*throttle
    right.force.x=1.0*throttle

    apply_left.publish(left)
    apply_right.publish(right)


def thrust_apply():
    global apply_left
    global apply_vertical
    global apply_right
    global apply_drag

    rospy.init_node('sim_control_node', anonymous=True)
    rate=rospy.Rate(10) # 10 Hz

    apply_left=rospy.Publisher("/left_thrust",Wrench, queue_size=5)
    apply_vertical=rospy.Publisher("/vertical_thrust",Wrench, queue_size=5)
    apply_right=rospy.Publisher("/right_thrust",Wrench, queue_size=5)
    apply_drag=rospy.Publisher("/drag_force",Wrench, queue_size=5)

    rospy.Subscriber("/loco/command", Command, commands_callback)


    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        thrust_apply()
    except rospy.ROSInterruptException:
        pass
