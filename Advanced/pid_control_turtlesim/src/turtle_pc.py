#!/usr/bin/env python

# rosrun turtlesim turtlesim_node
# rosservice call reset

# Importar librerías.
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import radians

class MoveTurtleProportionalControl:

    def __init__(self):
        rospy.init_node('control_tortuga_x_y_theta')
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.current_x = 0
        self.current_y = 0
        self.current_th = 0

        # Proportional constants for x, y, and theta
        self.Kp_x = 0.5
        self.Kp_y = 0.5
        self.Kp_theta = 0.5

    def pose_callback(self, pose):
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_th = pose.theta

    def move_turtle_to_desired_P(self, desired_xyTH):
        while not rospy.is_shutdown():
            error_x = desired_xyTH[0] - self.current_x
            error_y = desired_xyTH[1] - self.current_y
            error_th = desired_xyTH[2] - self.current_th

            vel_x = self.Kp_x * error_x
            vel_y = self.Kp_y * error_y
            vel_th = self.Kp_theta * error_th

            twist_msg = Twist()
            twist_msg.linear.x = vel_x
            twist_msg.linear.y = vel_y
            twist_msg.angular.z = vel_th

            self.velocity_publisher.publish(twist_msg)

            rospy.loginfo("Posición actual: (%f, %f, %f), Error: (%f, %f, %f), Velocidades: (%f, %f, %f)",
                          self.current_x, self.current_y, self.current_th, error_x, error_y, error_th, vel_x, vel_y, vel_th)

            if abs(error_x) < 0.1 and abs(error_y) < 0.1 and abs(error_th) < radians(5):
                rospy.loginfo("Posición deseada alcanzada")
                break

            self.rate.sleep()

    def get_desired_coords_from_user(self):
        print("Ingrese las posiciones deseadas (x, y, theta):")

        dPos = [0, 0, 0]

        dPos[0] = float(input("Coordenada en x: "))
        dPos[1] = float(input("Coordenada en y: "))
        dPos[2] = radians(float(input("Coordenada en theta (en grados): ")))

        return dPos

    def move_turtle_interactively(self):

        while not rospy.is_shutdown():
            desired_xyTH = self.get_desired_coords_from_user()
            desired_xyTH[2] = radians(desired_xyTH[2])
            self.move_turtle_to_desired_P(desired_xyTH)

if __name__ == '__main__':
    try:
        move_turtle_proportional = MoveTurtleProportionalControl()
        move_turtle_proportional.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
