#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

class App():

    def __init__(self):
        rospy.init_node("app_node", anonymous=False)                # informações gerais do dó
        rospy.Subscriber("sensor/value", Int32, self.update)        # subscrito com callback
        self.value = Int32()
    
    def run(self):
        rospy.spin()                                                # deixa o programa em espera constante
    
    def update(self, msg):
        self.value = msg
        print(f'Receiving message: value={self.value.data}')

if __name__ == "__main__":
    try:
        app = App()
        app.run()
    except rospy.ROSInterruptException:
        pass