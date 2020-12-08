#!/usr/bin/env python
# license removed for brevity
""" importar librerias necesarias """
import rospy
import RPi.GPIO as GPIO # libreria para comunicacion de puestos GPIO de la raspberry
import time             # libreria para obtener el tiempo 

from geometry_msgs.msg import Pose # importamos el tipo de dato pose
Enc=Pose()
def talker():
    pub = rospy.Publisher('Encoder', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        
        
        rospy.loginfo(Enc)
        pub.publish(Enc)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
