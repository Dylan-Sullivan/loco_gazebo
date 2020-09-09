#!/usr/bin/env python3

#This file is based on teleop_twist_keyboard.py

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState


import sys, select, termios, tty


msg = """
Reading from the keyboard
---------------------------
Planar Movement:
        w
   a         d
        x

i : up (+z)
k : down (-z)

Anything Else : stop

r/v : increase/decrease thruster power by 10% of full
---------------------------
CTRL-C to quit
"""

#pitch-yaw-throttle
moveBindings = {
        'w':( 1, 0, 0, 0),
        'a':( 0, 1, 0, 0),
        'd':( 0,-1, 0, 0),
        'x':(-1, 0, 0, 0),

        'i':( 0, 0, 1, 0),
        'k':( 0, 0,-1, 0),

    }

speedBindings={
        'r':(0.1,0),
        'v':(-0.1,0),
    }


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)
    rospy.init_node('vel_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    #print(speed)
    x_vel = 0
    y_vel = 0
    z_vel = 0
    status = 0

    try:
        print(msg)
        print( "Thruster input power is currently at: " + str(speed*100) + " %")

        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x_vel = moveBindings[key][0]
                y_vel = moveBindings[key][1]
                z_vel = moveBindings[key][2]

            elif key in speedBindings.keys():
                status=status+1
                if (status == 14):
                    print(msg)
                    status = 0

                if speed+speedBindings[key][0]>1:

                    print("Thruster power at maximum.")

                elif speed+speedBindings[key][0]<0:

                    print("Thruster power at minimum.")

                else:
                    if speed + speedBindings[key][0] < 0.1:
                        speed=0

                    else:
                        speed = speed + speedBindings[key][0]

                    print( "Thruster input power is currently at: " + str(speed*100) + " %")

            else:
                x_vel = 0
                y_vel = 0
                z_vel = 0
                if (key == '\x03'):
                    break

            # Get actual state of LoCO
            rospy.wait_for_service('/gazebo/get_model_state')
            try:
                model_state = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState )
                loco_base=model_state("robot","")
                #actual_pos=Vector3(loco_base.link_state.pose.position.x,loco_base.link_state.pose.position.y,loco_base.link_state.pose.position.z)
            except rospy.ServiceException as e:
                print ("Service call failed: %s"%e)

            vel = ModelState()
            vel.model_name='robot'
            vel.pose.position.x=loco_base.pose.position.x
            vel.pose.position.y=loco_base.pose.position.y
            vel.pose.position.z=loco_base.pose.position.z
            vel.twist.linear.x=x_vel*speed
            vel.twist.linear.y=y_vel*speed
            vel.twist.linear.z=z_vel*speed
            pub.publish(vel)

    except Exception as e:
        print(e)

    finally:
        vel = ModelState()
        vel = ModelState()
        vel.model_name='robot'
        vel.twist.linear.x=0
        vel.twist.linear.y=0
        vel.twist.linear.z=0
        pub.publish(vel)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
