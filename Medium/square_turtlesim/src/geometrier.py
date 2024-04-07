#!/usr/bin/env python
# license removed for brevity
import sys
import rospy
import termios


from std_msgs.msg import String
from geometry_msgs.msg import Twist

def lineafrente():
    twist.linear.x = 4
    pub.publish(twist)

def rotacionCuadrada():
    twist.angular.z = 1.57 # Radianes
    pub.publish(twist)

def rotacionTriangulo():
    twist.angular.z = 2.094
    pub.publish(twist)

def reset():
    twist.linear.x = 0
    twist.linear.y = 0
    twist.angular.z = 0
    pub.publish(twist)

def cuadrada():
    print('Haciendo cuadrado... ')
    for j in range (4):
        
        reset()
        rospy.sleep(1)
        lineafrente()
        rospy.sleep(1)
        reset()
        rospy.sleep(1)
        rotacionCuadrada()
        rospy.sleep(1)
        reset()

def triangulo():
    print('Haciendo triangulo... ')
    for j in range (3):
        
        reset()
        rospy.sleep(1)
        lineafrente()
        rospy.sleep(1)
        reset()
        rospy.sleep(1)
        rotacionTriangulo()
        rospy.sleep(1)
        reset()
        

# MAIN
        

if __name__ == "__main__":

    twist = Twist()
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)

    rospy.init_node('geometrier', anonymous= False)

    sel = input("¿Quieres que haga un triángulo o un cuadrado? ")

    if sel == "cuadrado":

        cuadrada()

    if sel == "triangulo":
        triangulo()