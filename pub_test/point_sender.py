import rospy
from geometry_msgs.msg import Point
import random
#from ros_igtl_bridge import igtlpoint
from ros_igtl_bridge.msg import igtlpoint
   

def pointPub():
    pub = rospy.Publisher('IGTL_POINT_OUT', igtlpoint, queue_size=10)
    rospy.init_node('point_sender', anonymous=True)

    point = igtlpoint()
    point.pointdata.x = 260.0
    point.pointdata.y = 260.0
    point.pointdata.z = 260.0
    point.name = "ROS_IGTL_Test_Pointttt"

    rate = rospy.Rate(1)

    
    while not rospy.is_shutdown():
        pub.publish(point)
        rospy.loginfo('Ponto publicado em /IGTL_POINT_OUT')
        rate.sleep()
        
        point.pointdata.x = random.randint(-260,260)
        point.pointdata.y = random.randint(-260,260)
        point.pointdata.z = random.randint(-260,260)


   

if __name__ == "__main__":
    try:
        pointPub()
    except rospy.ROSInterruptException:
        pass

    # TOPICOS A SEREM TRABALHADOS
    # /IGTL_IMAGE_IN
    # /IGTL_IMAGE_OUT
    # /IGTL_POINTCLOUD_IN
    # /IGTL_POINTCLOUD_OUT
    # /IGTL_POINT_IN
    # /IGTL_POINT_OUT
    # /IGTL_POLYDATA_IN
    # /IGTL_POLYDATA_OUT
    # /IGTL_STRING_IN
    # /IGTL_STRING_OUT
    # /IGTL_TRANSFORM_IN
    # /IGTL_TRANSFORM_OUT

