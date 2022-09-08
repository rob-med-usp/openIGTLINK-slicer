import rospy
from geometry_msgs.msg import Point
from ros_igtl_bridge.msg import igtlpoint


def callback(data):
    name = data.name
    x = data.pointdata.x
    y = data.pointdata.y
    z = data.pointdata.z

    rospy.loginfo('PONTO RECEBIDO: Nome: %s; x: %f; y: %f; x: %f;', name, x, y, z)

def sub():
    rospy.init_node('point_in', anonymous=True)

    rospy.Subscriber('IGTL_POINT_IN', igtlpoint, callback)

    rospy.spin()

if __name__ == "__main__":
    sub()