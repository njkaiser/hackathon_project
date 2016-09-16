import serial
import time
import rospy
import sys
import tty
import termios
from geometry_msgs.msg import Point

def keyboard_input():
    rospy.init_node('keyboard_input', anonymous=True)
    pub = rospy.Publisher('point_data', Point, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    pt = Point()
    while not rospy.is_shutdown():
        # pt.x controls panning movement
        # pt.y controls tilting movement
        # pt.z controls servo number (1 for pan, 0 for tilt)
        key = ord(getch())
        if key == 17:
            break
        elif key == 65: # up arrow
            pt.x = 0
            pt.y = 1
            pt.z = 0
        elif key == 66: # down
            pt.x = 0
            pt.y = -1
            pt.z = 0
        elif key == 67: #right
            pt.x = -1
            pt.y = 0
            pt.z = 1
        elif key == 68: #left
            pt.x = 1
            pt.y = 0
            pt.z = 1
        rospy.loginfo(pt)
        pub.publish(pt)
        rate.sleep()

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN,old)

if __name__ == '__main__':
    try:
        keyboard_input()
    except rospy.ROSInterruptException:
        pass
