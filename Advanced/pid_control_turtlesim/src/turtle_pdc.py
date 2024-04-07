#!/usr/bin/env python

# rosrun turtlesim turtlesim_node
# rosservice call reset

# Importar librerías.

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, radians

class MoveTurtlePDControl:
    def __init__(self):
        rospy.init_node('control_tortuga')
        
        # Suscribirse al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el topic de comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Variables para controlar x, y y theta
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0
        self.last_error_x = 0
        self.last_error_y = 0
        self.last_error_theta = 0

    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_desired_position(self, desired_x, desired_y, desired_theta):
        # Constantes de proporcionalidad y derivativa del controlador (ajustables)
        Kp_x = 1
        Kd_x = 0.1
        Kp_y = 1
        Kd_y = 0.1
        Kp_theta = 0.25
        Kd_theta = 0.04

        while not rospy.is_shutdown():
            # Calcular los errores de posición y orientación
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y
            error_theta = desired_theta - self.current_theta
            
            # Calcular las velocidades lineales y angulares del movimiento
            vel_x = Kp_x * error_x + Kd_x * (error_x - self.last_error_x)
            vel_y = Kp_y * error_y + Kd_y * (error_y - self.last_error_y)
            vel_theta = Kp_theta * error_theta + Kd_theta * (error_theta - self.last_error_theta)
            
            # Guardar los errores actuales para usarlos en la próxima iteración
            self.last_error_x = error_x
            self.last_error_y = error_y
            self.last_error_theta = error_theta
            
            # Crear un mensaje de Twist para enviar el comando de movimiento
            twist_msg = Twist()
            twist_msg.linear.x = vel_x
            twist_msg.linear.y = vel_y
            twist_msg.angular.z = vel_theta
            
            # Publicar el mensaje
            self.velocity_publisher.publish(twist_msg)
            
            # Imprimir la posición actual, los errores y las velocidades en la terminal
            rospy.loginfo("Posición actual: (%f, %f), Errores: (%f, %f, %f), Velocidades: (%f, %f, %f)",
                          self.current_x, self.current_y, error_x, error_y, error_theta, vel_x, vel_y, vel_theta)
            
            # Verificar si se alcanzan las posiciones deseadas
            if abs(error_x) < 0.1 and abs(error_y) < 0.1 and abs(error_theta) < radians(10):
                rospy.loginfo("Posición deseada alcanzada")
                break
            
            # Esperar hasta la siguiente iteración
            self.rate.sleep()

    def get_desired_position_from_user(self):
        print("Ingrese la posición deseada en el espacio (x, y, theta):")
        x = float(input("Coordenada x: "))
        y = float(input("Coordenada y: "))
        theta = radians(float(input("Ángulo theta (en grados): ")))
        return x, y, theta

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Obtener la posición deseada del usuario
            desired_x, desired_y, desired_theta = self.get_desired_position_from_user()

            # Mover la tortuga a la posición deseada
            self.move_turtle_to_desired_position(desired_x, desired_y, desired_theta)

if __name__ == '__main__':
    try:
        move_turtle_pd = MoveTurtlePDControl()
        move_turtle_pd.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
