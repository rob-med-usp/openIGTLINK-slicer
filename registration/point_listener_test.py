#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from ros_igtl_bridge.msg import igtlpoint
import os



class Listener():

    def __init__(self):
        self.txt_nome = "slicer_points.txt"

    def callback(self, data):
        name = data.name
        x = data.pointdata.x
        y = data.pointdata.y
        z = data.pointdata.z

        id = name[-1:]
        # rospy.loginfo("\x1b[H")
        rospy.loginfo('PONTO RECEBIDO: Nome: %s; x: %f; y: %f; z: %f;', name, x, y, z)
        msg = f"{x:.2f} {y:.2f} {z:.2f} {id}" # <-- Ajustar precisão do registro com essas casas decimais
        print(msg + "\n")

        # a fazer:
        # log com historico de pontos recebidos com horario e data (em local especifico)

        if id == "1":
            open(self.txt_nome, "w").close() # apaga arquivo a cada recebimento dos pontos 

        with open(self.txt_nome, "a") as file:
            file.write(msg + "\n")

    

    def set_node_pub(self):
        rospy.init_node('point_in', anonymous=True)

        # file.write()

        rospy.Subscriber('IGTL_POINT_IN', igtlpoint, self.callback)
    
        rospy.spin()
    
    def flush(self):
        with open(self.txt_nome, "w") as file:
            file.flush()

def main():

    listener = Listener()
    listener.flush()
    listener.set_node_sub()

if __name__ == "__main__":
    main()
