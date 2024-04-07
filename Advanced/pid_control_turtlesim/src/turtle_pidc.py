#!/usr/bin/env python

# rosrun turtlesim turtlesim_node
# rosservice call reset

# Importar librerías.

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, radians

class MoveTurtlePIDControl:
    def __init__(self):
        rospy.init_node('control_tortuga_pose')
        
        # Suscribirse al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el topic de comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        self.current_pose = Pose()
        self.last_error = Pose()
        self.error_accumulation = Pose()

        # Constantes de proporcionalidad, integral y derivativa del controlador para x
        self.Kp_x = 9
        self.Ki_x = 0.02
        self.Kd_x = 0.2

        # Constantes de proporcionalidad, integral y derivativa del controlador para y
        self.Kp_y = 9
        self.Ki_y = 0.02
        self.Kd_y = 0.2

        # Constantes de proporcionalidad, integral y derivativa del controlador para theta
        self.Kp_theta = 0.1
        self.Ki_theta = 0.001
        self.Kd_theta = 0.01

    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.current_pose = pose

    def move_turtle_to_desired_pose(self, desired_pose):
        while not rospy.is_shutdown():
            # Calcular el error de posición
            error = Pose()
            error.x = desired_pose.x - self.current_pose.x
            error.y = desired_pose.y - self.current_pose.y
            error.theta = desired_pose.theta - self.current_pose.theta
            
            # Sumar el error a la acumulación de errores
            self.error_accumulation.x += error.x
            self.error_accumulation.y += error.y
            self.error_accumulation.theta += error.theta
            
            # Calcular las velocidades lineales y de giro
            vel_linear_x = self.Kp_x * error.x + self.Ki_x * self.error_accumulation.x + self.Kd_x * (error.x - self.last_error.x)
            vel_linear_y = self.Kp_y * error.y + self.Ki_y * self.error_accumulation.y + self.Kd_y * (error.y - self.last_error.y)
            vel_angular = self.Kp_theta * error.theta + self.Ki_theta * self.error_accumulation.theta + self.Kd_theta * (error.theta - self.last_error.theta)
            
            # Guardar el error actual para usarlo en la próxima iteración
            self.last_error = error
            
            # Crear un mensaje de Twist para enviar el comando de movimiento
            twist_msg = Twist()
            twist_msg.linear.x = vel_linear_x
            twist_msg.linear.y = vel_linear_y
            twist_msg.angular.z = vel_angular
            
            # Publicar el mensaje
            self.velocity_publisher.publish(twist_msg)
            
            # Imprimir la posición actual, el error y las velocidades en la terminal
            rospy.loginfo("Posición actual: (%f, %f, %f), Error: (%f, %f, %f), Velocidades lineales: (%f, %f), Velocidad angular: %f",
                          self.current_pose.x, self.current_pose.y, self.current_pose.theta,
                          error.x, error.y, error.theta,
                          vel_linear_x, vel_linear_y, vel_angular)
            
            # Verificar si se alcanza la posición deseada
            if abs(error.x) < 0.1 and abs(error.y) < 0.1 and abs(error.theta) < radians(1):
                rospy.loginfo("Posición deseada alcanzada")
                break
            
            # Esperar hasta la siguiente iteración
            self.rate.sleep()

    def get_desired_pose_from_user(self):
        print("Ingrese la posición deseada en la forma (x, y, theta) separada por comas:")
        input_str = input("Coordenadas (x, y, theta): ")
        desired_pose_values = list(map(float, input_str.split(',')))
        desired_pose = Pose()
        desired_pose.x = desired_pose_values[0]
        desired_pose.y = desired_pose_values[1]
        desired_pose.theta = radians(desired_pose_values[2])
        return desired_pose

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Obtener la posición deseada del usuario
            desired_pose = self.get_desired_pose_from_user()

            # Mover la tortuga a la posición deseada
            self.move_turtle_to_desired_pose(desired_pose)

if __name__ == '__main__':
    try:
        move_turtle_pid = MoveTurtlePIDControl()
        move_turtle_pid.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
