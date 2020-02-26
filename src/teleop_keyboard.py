#!/usr/bin/env python

#This file is based on teleop_twist_keyboard.property

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

#from geometry_msgs.msg import Twist
from loco_pilot.msg import Command

import sys, select, termios, tty

msg = """
Reading from the keyboard
---------------------------
Moving around:
        w
   a    s    d

i : up (+z)
k : down (-z)

anything else : stop

q/z : increase/decrease max thruster input by 10% of full

CTRL-C to quit
"""

#pitch-yaw-throttle
CommandBindings = {
        'w':(0,0,1),
        'a':(0,-1,0),
        's':(0,0,-1),
        'd':(0,1,0),
        'i':(1,0,0),
        'k':(-1,0,0),
    }

inputBindings={
        'q':(0.1),
        'z':(-0.1),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


#def vels(speed,turn):
    #return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('/loco/command', Command, queue_size = 1)
    rospy.init_node('teleop_keyboard')

    #speed = rospy.get_param("~speed", 0.5)
    #turn = rospy.get_param("~turn", 1.0)
    f_left = 0
    f_vert = 0
    f_right = 0
    throttle = 0.5
    #status = 0

    try:
        print(msg)
        #print(vels(speed,turn))
        print("Thruster input throttle is currently: %s %" %(throttle*100)
        while(1):
            key = getKey()
            if key in forceBindings.keys():
                f_left = forceBindings[key][0]
                f_vert = forceBindings[key][1]
                f_right = forceBindings[key][2]
################
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
