#!/usr/bin/env python
# license removed for brevity
import sys
import rospy
import termios
import tty
import select

from std_msgs.msg import String
from geometry_msgs.msg import Twist


#MENSAJE INICIAL
msg = """
Mapa de teclas:
---------------------------
     q   w  e 
   a          d
         s   
---------------------------

CTRL-C para salir.
"""
# MAPEO DE TECLAS

moveBindings = {
        # X, Y, Theta

    'w': (1, 0, 0),         # Frente
    'a': (0, 1, 0),        # Izquierda
    'd': (0, -1, 0),         # Derecha
    's': (-1, 0, 0),        # Atrás
    'e': (0,0,-0.5),          # Giro derecha
    'q': (0,0,0.5),           # Giro izquierda
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# MAIN

if __name__ == "__main__":

    settings = termios.tcgetattr(sys.stdin)

    # PUBLISHER EN turtle/cmd_vel UTILIZANDO LOS VALORES DE TWIST

    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    
    # INICIALIZA EL NODO: teleop_twist_keyboard
    rospy.init_node('teleoper', anonymous= False)


    twist = Twist()

    try:
        # Imprime el mensaje
        print (msg)
        
        while(1):
            # Asigna a key los valores del vector de la tecla presionada
            key = getKey()

            # Obtiene los valores de x, y, z y th del vector key
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][2]
                        
            # Si no se presiona ninguno, x, y, z y theta serán defaulteados a 0.
            else:
                x = 0
                y = 0
                th = 0
                if (key == '\x03'):
                    break
           
           # Se actualizan los valores de x, y y z para el twist.

            twist.linear.x = x 
            twist.linear.y = y 
            twist.angular.z = th
            pub.publish(twist)


    finally:
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)