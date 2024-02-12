#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
from pynput import keyboard


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sympy as sp
import numpy as np
import math
from sympy import Matrix , N
from sympy import *
from sympy import sin,cos, symbols, pi, diff, zeros, eye , simplify
sp.init_printing(use_unicode=True, wrap_line =False)

import time

# Define key codes
LIN_VEL_STEP_SIZE = 4.0
ANG_VEL_STEP_SIZE = 0.5

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.settings = termios.tcgetattr(sys.stdin)


    def run_keyboard_control(self):
        self.msg = """
        Circular Path
        ---------------------------
        """

        # self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()





        self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
   
        PI = sp.pi

        # Define symbolic variables
        A_i, a_i, alpha_i, d_i, theta_i, T_0_6 = symbols('A_i, a_i alpha_i d_i theta_i. T_0_6')
        theta1, theta2, theta3, theta4, theta5, theta6, A1, A2, A3, A4, A5, A6 = symbols('theta1, theta2, theta3, theta4, theta5, theta6, A1, A2, A3, A4, A5, A6')
        phi, r = symbols('phi r')
        
        # dhtable=[['a', 'alpha', 'd', 'theta']]       #New Robot: DH table of home configuration
        D_h_1 = [0, pi/2, 8.96, (theta1)]
        D_h_2 = [42.9, 0 , 0, (theta2 +pi/2)]
        D_h_3 = [40,0, 0, (theta3)]
        D_h_4 = [0, -pi/2, 8.1,(theta4-pi/2)]
        D_h_5 = [0, pi/2, 8.1, theta5]
        D_h_6 = [0, 0, 7.154, theta6]


        # Define the A_i matrix template
        A_i = Matrix([
            [cos(theta_i), -(cos(alpha_i)*sin(theta_i)), sin(alpha_i)*sin(theta_i), a_i*cos(theta_i)],
            [sin(theta_i), cos(theta_i)*cos(alpha_i) , -(sin(alpha_i)*cos(theta_i)) , a_i*sin(theta_i)],
            [0, sin(alpha_i), cos(alpha_i), d_i],
            [0, 0, 0, 1]
        ])

        # Substitute values from D_h parameters table into A_i to obtain the n to n-1 matrices in terms of theta_i
        A1 = A_i.subs({a_i: D_h_1[0], alpha_i: D_h_1[1], d_i: D_h_1[2], theta_i: D_h_1[3]})
        A2 = A_i.subs({a_i: D_h_2[0], alpha_i: D_h_2[1], d_i: D_h_2[2], theta_i: D_h_2[3]})
        A3 = A_i.subs({a_i: D_h_3[0], alpha_i: D_h_3[1], d_i: D_h_3[2], theta_i: D_h_3[3]})
        A4 = A_i.subs({a_i: D_h_4[0], alpha_i: D_h_4[1], d_i: D_h_4[2], theta_i: D_h_4[3]})
        A5 = A_i.subs({a_i: D_h_5[0], alpha_i: D_h_5[1], d_i: D_h_5[2], theta_i: D_h_5[3]})
        A6 = A_i.subs({a_i: D_h_6[0], alpha_i: D_h_6[1], d_i: D_h_6[2], theta_i: D_h_6[3]})
    
        
        T_0_1 = A1
        T_0_2 = T_0_1*A2
        T_0_3 = T_0_2*A3
        T_0_4 = T_0_3*A4
        T_0_5 = T_0_4*A5
        T_0_6 = simplify(T_0_5*A6)
        


        print("next position")
        time.sleep(1)  # Pauses for 4 seconds
        # position 1:
        q_current = Matrix([0,0,0,0,0,0])
        # q_current = Matrix([1.5,0.8,-0.6,-1,0.9,0])
        p1_T= T_0_6.subs({theta1: q_current[0], theta2: q_current[1], theta3: q_current[2], theta4: q_current[3], theta5: q_current[4], theta6: q_current[5]})
        P_endeffector = N(T_0_6[0:3 , -1])
        P_endeffector= N(P_endeffector.subs({theta1: q_current[0], theta2: q_current[1], theta3: q_current[2], theta4: q_current[3], theta5: q_current[4], theta6: q_current[5]}))

                # print(joint_positions.data)
        joint_positions.data = [0.0,0.0,0.0,0.0,float(q_current [0]),float(q_current [1]),float(q_current [2]),float(q_current [3]),float(q_current [4]),float(q_current [5])]
        

        # self.joint_position_pub.publish(q)
        # self.wheel_velocities_pub.publish(wheel_velocities)

        wheel_velocities.data = [0.0,0.0,0.0,0.0]
        print(joint_positions.data)
        # print(wheel_velocities.data)
        print(P_endeffector)
        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)

        

        print("Circular Trajactory following")
        time.sleep(1)  # Pauses for 4 seconds
 

        Q = [[0.0,0.0,0.0,0.0,0.0,0.0],
            [-23.7,-0.8,-1.09,1.3,-1.5,0.0],
            [-24.1,-0.8,-1.09,1.3,-1.5,0.0],
            [-24.6,-0.8,-1.09,1.3,-1.5,0.0],
            [-24.9,-0.8,-1.09,1.3,-1.5,0.0],
            [-25.1,-0.8,-1.09,1.3,-1.5,0.0],
            [-25.6,-0.8,-1.09,1.3,-1.5,0.0],
            [-26.0,-0.8,-1.09,1.3,-1.5,0.0],
            [-26.2,-0.8,-1.09,1.3,-1.5,0.0],
            [-26.7,-0.8,-1.09,1.3,-1.5,0.0],
            [-27.1,-0.8,-1.09,1.3,-1.5,0.0],
            [-27.6,-0.8,-1.09,1.3,-1.5,0.0],
            [-27.9,-0.8,-1.09,1.3,-1.5,0.0],
            [-28.4,-0.8,-1.09,1.3,-1.5,0.0],
            [-28.9,-0.8,-1.09,1.3,-1.5,0.0],
            [-29.3,-0.8,-1.09,1.3,-1.5,0.0],
            [-29.8,-0.8,-1.09,1.3,-1.5,0.0],
            [-30.2,-0.8,-1.09,1.3,-1.5,0.0],
            [-30.6,-0.8,-1.09,1.3,-1.5,0.0],
            [-31.1,-0.8,-1.09,1.3,-1.5,0.0],
            [-31.6,-0.8,-1.09,1.3,-1.5,0.0]]
        # pprint(Q)

        # position final:

        for i in range (0,20):
            q_current = Q[i]
            p1_T= T_0_6.subs({theta1: q_current[0], theta2: q_current[1], theta3: q_current[2], theta4: q_current[3], theta5: q_current[4], theta6: q_current[5]})
            P_endeffector = T_0_6[0:3 , -1]
            P_endeffector= P_endeffector.subs({theta1: q_current[0], theta2: q_current[1], theta3: q_current[2], theta4: q_current[3], theta5: q_current[4], theta6: q_current[5]})

            # print(joint_positions.data)
            joint_positions.data = [0.0,0.0,0.0,0.0,float(q_current [0]),float(q_current [1]),float(q_current [2]),float(q_current [3]),float(q_current [4]),float(q_current [5])]

            wheel_velocities.data = [0.0,0.0,0.0,0.0]
            print(joint_positions.data)
            # print(wheel_velocities.data)
            print(P_endeffector)
            self.joint_position_pub.publish(joint_positions)
            self.wheel_velocities_pub.publish(wheel_velocities)
            time.sleep(1)
        
           



def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()