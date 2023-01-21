#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on

@author:

Description
"""
# Import numpy to process data
import numpy as np

# ROS Python API
import rospy
import message_filters

# Cluster controllers
from gpsic.controladores.adaptativo import asmc_asv as asmc
from gpsic.toolkit import ros

# from heron_msgs.msg import Drive
from std_msgs.msg import Float64

from geometry_msgs.msg import TwistStamped, Pose2D, Vector3

# Constants that may change in multiple places
MSG_QUEUE_MAXLEN = 50

class ASMCControllerNode(object):
    """
    
    """
    def __init__(self):
        self.rate = float(rospy.get_param('~rate', '100'))    # 100 Hz
        self.slop = float(rospy.get_param('~slop', 0.02))

        self.lambda_u = 0.001
        self.lambda_r = 0.001
        self.k_beta_u = 0.1         # k2
        self.k_beta_r = 0.01         # k2
        self.k_min_u = 0.1
        self.k_min_r = 0.1
        self.mu_u = 0.01
        self.mu_r = 0.05
        self.k_u = 2
        self.k_r = 0.5

        self.u_d = 0.0
        self.psi_d = 0.0
        self.theta = 0.0
        self.u = 0.0
        self.v = 0.0
        self.r = 0.0


        t0 = rospy.get_time()  #position.header.stamp.to_sec()
        # Construyo el controlador que se va a utilizar
        self.ctrl = asmc.ASMC(t0, 
                self.k_beta_u, self.k_beta_r, 
                self.lambda_u, self.lambda_r,
                self.k_min_u * np.ones((1, 1)), self.k_u * np.ones((1, 1)), self.mu_u * np.ones((1, 1)), 
                self.k_min_r * np.ones((1, 1)), self.k_r * np.ones((1, 1)), self.mu_r * np.ones((1, 1)))

        # Me suscribo al tópico donde se envian los datos de posición
        self.NED_subs = rospy.Subscriber(
            '/vectornav/ins_2d/NED_pose',
            Pose2D,
            self.NEDpose_cb)
        # rospy.loginfo('[ASMC] Subscribing to Cluster position topic: %s', position_topic)

        # Me suscribo al tópico donde se envian los datos de velocidad
        self.vel = rospy.Subscriber(
            '/vectornav/ins_2d/local_vel',
            Vector3,
            self.vel_cb)
        # rospy.loginfo('[ASMC] Subscribing to Cluster velocity topic: %s', velocity_topic)

        # Me suscribo al tópico donde se envía la referencia
        self.ref_vel_subs = rospy.Subscriber(
            '/guidance/desired_speed',
            Float64,
            self.ref_vel_cb)
        self.ref_vel_subs = rospy.Subscriber(
            '/guidance/desired_heading',
            Float64,
            self.ref_heading_cb)
        # rospy.loginfo('[ASMC] Subscribing to Cluster reference topic: %s', reference_topic)

        # Creo el publicador que despacha los mensajes del control
        self.rt_cmd = rospy.Publisher(
            '/usv_control/controller/right_thruster',
            Float64,
            queue_size=MSG_QUEUE_MAXLEN)
        self.lt_cmd = rospy.Publisher(
            '/usv_control/controller/left_thruster',
            Float64,
            queue_size=MSG_QUEUE_MAXLEN)

        self.ctrl_input = rospy.Publisher(
            '/usv_control/controller/control_input',
            Pose2D,
            queue_size=MSG_QUEUE_MAXLEN)
        # rospy.loginfo('[ASMC] Will publish Cluster commands to topic: %s', command_topic)

        # Creo el timer con el que lo voy a invocar
        self.publish_timer = rospy.Timer(
            rospy.Duration(1./self.rate), 
            self.update, 
            oneshot=False)
        rospy.loginfo('[REF] Update rate: %f Hz', self.rate)

        rospy.loginfo("[ASMC] Starting 2 Vehicle Cluster PID Controller...")

    def ref_heading_cb(self, msg):
        self.psi_d = msg.data

    def ref_vel_cb(self, msg):
        self.u_d = msg.data

    def vel_cb(self, msg):
        self.u = msg.x
        self.v = msg.y
        self.r = msg.z

    def NEDpose_cb(self, msg):
        self.theta = msg.theta

    def update(self, event):
        """Compute control action based on references and positions messages"""

        t = rospy.get_time()

        vel = np.array([self.u, self.v, self.r, 0.0, 0.0, 0.0])
        vel_d = np.array([self.u_d, 0.0, 0.0, 0.0, 0.0, 0.0])
        psi = self.theta
        psi_d = self.psi_d
        # Genero la acción de control
        T_u, T_r, [Tport, Tstbd] =  self.ctrl.control(t, vel_d, vel, psi_d, psi)

        mes = Pose2D()
        mes.x = T_u
        mes.theta = T_r
        self.ctrl_input.publish(mes)

        # Creo el mensaje que en algún momento se va a enviar
        message = Float64()
        # message.header.stamp = rospy.Time.now()
        message.data = Tport
        self.lt_cmd.publish(message)
        message.data = Tstbd
        self.rt_cmd.publish(message)

    def shutdown(self):
        """Unregisters publishers and subscribers and shutdowns timers"""
        self.position_subs.unregister()
        self.velocity_subs.unregister()
        self.reference_subs.unregister()
        self.command_publisher.unregister()
        rospy.loginfo("[ASMC] Sayonara ASMC controller. Nos vemo' en Disney.")

def main():
    """¿Entrypoint? del nodo"""
    rospy.init_node('asmc_controller', anonymous=True, log_level=rospy.INFO)

    try:
        node = ASMCControllerNode()
    except KeyboardInterrupt:
        rospy.loginfo("[ASMC] Received Keyboard Interrupt (^C) while initialization. Shutting down.")
        return

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[ASMC] Received Keyboard Interrupt (^C). Shutting down.")

    node.shutdown()

if __name__ == '__main__':
    main()
