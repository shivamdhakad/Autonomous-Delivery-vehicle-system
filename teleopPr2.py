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

# Define key codes
LIN_VEL_STEP_SIZE = 2.0
ANG_VEL_STEP_SIZE = 0.1

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        self.msg = """
        Control Your Car!
        ---------------------------
        Moving around:
            w
        a    s    d

        q : force stop

        Esc to quit
        """

        self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        linear_vel=0.0
        steer_angle=0.0
        l2_steer_angle=0.0
        l3_steer_angle=0.0
        l4_steer_angle=0.0
        l5_steer_angle=0.0
        l6_steer_angle=0.0
        le_steer_angle=0.0


        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'q':  # Quit
                    linear_vel=0.0
                    steer_angle=0.0
                elif key == 'w':  # Forward
                    linear_vel += LIN_VEL_STEP_SIZE
                elif key == 's':  # Reverse
                    linear_vel -= LIN_VEL_STEP_SIZE
                elif key == 'd':  # Right
                    steer_angle -= ANG_VEL_STEP_SIZE
                elif key == 'a':  # Left
                    steer_angle += ANG_VEL_STEP_SIZE


                # For movement in arm joints
                # move link2
                elif key == 'r':  # clockwise
                    l2_steer_angle -= ANG_VEL_STEP_SIZE
                elif key == 'f':  # anticlockwise
                    l2_steer_angle += ANG_VEL_STEP_SIZE

                # move link3

                elif key == 't':   # clockwise
                    l3_steer_angle -= ANG_VEL_STEP_SIZE
                elif key == 'g':  # anticlockwise
                    l3_steer_angle += ANG_VEL_STEP_SIZE

                # move link4
                elif key == 'y':   # clockwise
                    l4_steer_angle -= ANG_VEL_STEP_SIZE
                elif key == 'h':  # anticlockwise
                    l4_steer_angle += ANG_VEL_STEP_SIZE
                
                # move link5
                elif key == 'u':   # clockwise
                    l5_steer_angle -= ANG_VEL_STEP_SIZE
                elif key == 'j':  # anticlockwise
                    l5_steer_angle += ANG_VEL_STEP_SIZE                 
                
                # move link6
                elif key == 'i':   # clockwise
                    l6_steer_angle -= ANG_VEL_STEP_SIZE
                elif key == 'k':  # anticlockwise
                    l6_steer_angle += ANG_VEL_STEP_SIZE
                
                # move link endeffector
                elif key == 'o':   # clockwise
                    le_steer_angle -= ANG_VEL_STEP_SIZE
                elif key == 'l':  # anticlockwise
                    le_steer_angle += ANG_VEL_STEP_SIZE  


                if steer_angle>1.0:
                        steer_angle=1.0
                if steer_angle<-1.0:
                    steer_angle=-1.0

                print("Steer Angle",steer_angle)
                print("Linear Velocity",linear_vel)
                # Publish the twist message
                wheel_velocities.data = [linear_vel,-linear_vel,linear_vel,-linear_vel]
                # joint_positions.data = [steer_angle,steer_angle,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
                joint_positions.data = [steer_angle,steer_angle,-steer_angle,-steer_angle,l2_steer_angle,l3_steer_angle,
                                            l4_steer_angle,l5_steer_angle,l6_steer_angle,le_steer_angle]
                print(joint_positions.data)

                self.joint_position_pub.publish(joint_positions)
                self.wheel_velocities_pub.publish(wheel_velocities)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()