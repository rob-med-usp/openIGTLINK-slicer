import rospy
from geometry_msgs.msg import Point
from ros_igtl_bridge.msg import igtlpoint
import os

txt_nome = "slicer_points.txt"

def callback(data):
    name = data.name
    x = data.pointdata.x
    y = data.pointdata.y
    z = data.pointdata.z

    id = name[-1:]
    rospy.loginfo('PONTO RECEBIDO: Nome: %s; x: %f; y: %f; x: %f;', name, x, y, z)
    msg = f"{x:.2f} {y:.2f} {z:.2f} {id}" # <-- Ajustar precisão do registro com essas casas decimais
    print(msg + "\n")

    # a fazer:
    # log com historico de pontos recebidos com horario e data (em local especifico)

    if id == "1":
        open(txt_nome, "w").close() # apaga arquivo a cada recebimento dos pontos 

    with open(txt_nome, "a") as file:
        file.write(msg + "\n")

   

def sub():
    rospy.init_node('point_in', anonymous=True)

    
    # file.write()

    rospy.Subscriber('IGTL_POINT_IN', igtlpoint, callback)
    

    rospy.spin()

if __name__ == "__main__":
    with open(txt_nome, "w") as file:
        file.flush()
    sub()