#!/usr/bin/env python

import rospy
from loco_pilot.msg import Command
from geometry_msgs.msg import Wrench
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Quaternion, Transform
from tf.transformations import *

apply_left=None
apply_vertical=None
apply_right=None
apply_drag=None

vertical_raw=0
right_raw=0
left_raw=0
maxvel=0

def commands_callback1(data):

    global vertical_raw
    global right_raw
    global left_raw

    max_force=23.15 # Newtons

    pitch=data.pitch
    yaw=data.yaw
    throttle=data.throttle

    vertical_raw=max_force*pitch
    left_raw=max_force*throttle+max_force*yaw
    right_raw=max_force*throttle-max_force*yaw

def commands_callback2(data):
    global apply_left
    global apply_vertical
    global apply_right
    global apply_drag

    global maxvel

    lumped_drag_param=20.578

    vertical=Wrench()
    left=Wrench()
    right=Wrench()
    drag=Wrench()

    for i in range (0,len(data.name)):
        if data.name[i]=="robot::right_thruster":
            rtindex=i
        if data.name[i]=="robot::left_thruster":
            ltindex=i
        if data.name[i]=="robot::vertical_thruster":
            vtindex=i
        if data.name[i]=="robot::loco_base_frame":
            drindex=i


    vertical_vec= [0,0,vertical_raw,0]
    right_vec= [0,0,right_raw,0]
    left_vec=[0,0,left_raw,0]

    # Unknown issue with using Quaternion constructor, this is a workaround.
    quat_rt=[data.pose[rtindex].orientation.x,data.pose[rtindex].orientation.y,data.pose[rtindex].orientation.z,data.pose[rtindex].orientation.w]
    quat_lt=[data.pose[ltindex].orientation.x,data.pose[ltindex].orientation.y,data.pose[ltindex].orientation.z,data.pose[ltindex].orientation.w]
    quat_vt=[data.pose[vtindex].orientation.x,data.pose[vtindex].orientation.y,data.pose[vtindex].orientation.z,data.pose[vtindex].orientation.w]
    quat_dr=[data.pose[drindex].orientation.x,data.pose[drindex].orientation.y,data.pose[drindex].orientation.z,data.pose[drindex].orientation.w]

    new_vertical=quaternion_multiply(quaternion_multiply(quat_vt,vertical_vec),quaternion_conjugate(quat_vt))
    new_right=quaternion_multiply(quaternion_multiply(quat_rt,right_vec),quaternion_conjugate(quat_rt))
    new_left=quaternion_multiply(quaternion_multiply(quat_lt,left_vec),quaternion_conjugate(quat_lt))


    #Drag quaternion operations have yet to be completed
    #new_dragx=quaternion_multiply(quaternion_multiply(quat_dr,[1,0,0,0]),quaternion_conjugate(quat_dr))
    #new_dragy=quaternion_multiply(quaternion_multiply(quat_dr,[0,1,0,0]),quaternion_conjugate(quat_dr))
    #new_dragz=quaternion_multiply(quaternion_multiply(quat_dr,[0,0,1,0]),quaternion_conjugate(quat_dr))
    vertical.force.x=new_vertical[0]
    vertical.force.y=new_vertical[1]
    vertical.force.z=new_vertical[2]
    left.force.x=new_left[0]
    left.force.y=new_left[1]
    left.force.z=new_left[2]
    right.force.x=new_right[0]
    right.force.y=new_right[1]
    right.force.z=new_right[2]


    drag.force.x=-lumped_drag_param*data.twist[drindex].linear.x*abs(data.twist[drindex].linear.x)
    drag.force.y=-lumped_drag_param*data.twist[drindex].linear.y*abs(data.twist[drindex].linear.y)
    drag.force.z=-lumped_drag_param*data.twist[drindex].linear.z*abs(data.twist[drindex].linear.z)
    drag.torque.x=-lumped_drag_param*(.133*data.twist[drindex].angular.x*abs(data.twist[drindex].angular.x))
    drag.torque.y=-lumped_drag_param*(.133*data.twist[drindex].angular.y*abs(data.twist[drindex].angular.y))
    drag.torque.z=-lumped_drag_param*(.133*data.twist[drindex].angular.z*abs(data.twist[drindex].angular.z))

    vel=(data.twist[drindex].linear.x**2+data.twist[drindex].linear.y**2+data.twist[drindex].linear.z**2)**0.5
    if vel>maxvel:
        maxvel=vel
        print(maxvel)


    apply_vertical.publish(vertical)
    apply_left.publish(left)
    apply_right.publish(right)
    apply_drag.publish(drag)


def thrust_apply():
    global apply_left
    global apply_vertical
    global apply_right
    global apply_drag

    rospy.init_node('sim_control_node', anonymous=True)
    rate=rospy.Rate(10) # 10 Hz

    apply_left=rospy.Publisher("/left_thrust",Wrench, queue_size=1)
    apply_vertical=rospy.Publisher("/vertical_thrust",Wrench, queue_size=1)
    apply_right=rospy.Publisher("/right_thrust",Wrench, queue_size=1)
    apply_drag=rospy.Publisher("/drag_force",Wrench, queue_size=1)

    rospy.Subscriber("/loco/command", Command, commands_callback1)
    rospy.Subscriber("/gazebo/link_states",LinkStates,commands_callback2)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        thrust_apply()
    except rospy.ROSInterruptException:
        pass
