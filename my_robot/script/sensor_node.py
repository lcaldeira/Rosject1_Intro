#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

class Sensor():

    def __init__(self):
        rospy.init_node("sensor_node", anonymous=False)                     # informações gerais do dó       
        self.pub = rospy.Publisher("sensor/value", Int32, queue_size=10)    # tópico a ser publicado
        self.rate = rospy.Rate(1)                                           # taxa de publicação (Hz)
        self.value = Int32()                                                # objeto a ser publicado
        self.value.data = 0

    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo(f"Publishing data value = {self.value.data}")     # log para acompanhar a pub
            self.pub.publish(self.value)                                    # publica o valor
            self.value.data = (self.value.data + 1)*(self.value.data < 99)  # atualização do valor pq sim
            self.rate.sleep()                                               # garante o ritmo de publicação

if __name__ == "__main__":
    try:
        sens = Sensor()
        sens.run()
    except rospy.ROSInterruptException:
        pass