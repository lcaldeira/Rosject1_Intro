#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float64, String

MAX_VEL = 70 # velocidade máxima das rodas, em RPM

class Motor():

    def __init__(self):
        # inicializa o módulo de controle do motor
        rospy.init_node("motor_control_node", anonymous=False)
        # se inscreve para receber mensagens de controle e colisão
        rospy.Subscriber("control_msg", String, self.controller)
        rospy.Subscriber("collision/status", Bool, self.collisor)
        # cria os tópicos de velocidade das rodas direita e esquerda
        self.pub_r = rospy.Publisher("motor/velocity/right", Float64, queue_size=10)
        self.pub_l = rospy.Publisher("motor/velocity/left", Float64, queue_size=10)
        
        self.command = String(data='stop') # comando a ser seguido
        self.vel_r = Float64(data=0) # velocidade da rota da direita
        self.vel_l = Float64(data=0) # velocidade da rota da esquerda
        self.colliding = Bool(data=False) # status de colisão
        self.postman = rospy.Rate(1) # frequência de envio das mensagens, em Hz

    def controller(self, msg):
        # recebe o comando do controle remoto
        print(f'Comando do controle remoto: {msg.data}')
        self.command.data = msg.data

    def collisor(self, msg):
        # recebe o status de colisão
        print(f'Status do sensor de colisão: {msg.data}')
        self.colliding.data = msg.data
    
    def run(self):
        while not rospy.is_shutdown():

            # comando 'pare' ou estar prestes a colidir
            if self.command.data == 'stop' or self.colliding.data:
                self.vel_r.data = 0
                self.vel_l.data = 0
            # comando 'ir para frente'
            elif self.command.data == 'forward':
                self.vel_r.data = +MAX_VEL
                self.vel_l.data = +MAX_VEL
            # comando 'ir para trás'
            elif self.command.data == 'backward':
                self.vel_r.data = -MAX_VEL
                self.vel_l.data = -MAX_VEL
            # comando 'virar p/ direita'
            elif self.command.data == 'right':
                self.vel_r.data = -MAX_VEL
                self.vel_l.data = +MAX_VEL
            # comando 'virar p/ esquerda'
            elif self.command.data == 'left':
                self.vel_r.data = +MAX_VEL
                self.vel_l.data = -MAX_VEL
            # notifica o erro
            else:
                print('Comando errado')

            # entrega os valores para as rodas e vai dormir
            self.pub_r.publish(self.vel_r)
            self.pub_l.publish(self.vel_l)
            self.postman.sleep()

if __name__ == "__main__":
    try:
        motor_ctrl = Motor()
        motor_ctrl.run()
    except rospy.ROSInterruptException:
        pass