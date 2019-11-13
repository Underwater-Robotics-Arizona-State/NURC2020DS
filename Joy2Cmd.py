#!/usr/bin/env/python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import control_scheme_id
from enum import enum


class Joy2Cmd:
    def __init__(self):
        self.drive_pub = rospy.Publisher('raw_drive_setpoint', Twist, queue_size=10)
        self.raw_drive_setpoint_msg = Twist()
        self.raw_drive_setpoint_msg.linear.x = self.raw_drive_setpoint_msg.linear.y = self.raw_drive_setpoint.linear.z = 0.0
        self.raw_drive_setpoint_msg.angular.x = self.raw_drive_setpoint.angular.y = self.raw_drive_setpoint.angular.z = 0.0
        self.target_drive_linear = [0.0, 0.0, 0.0]
        self.target_drive_rot = [0.0, 0.0, 0.0]

        self.cam_pitch_vel.pub = rospy.Publisher('cam_pitch_vel', Float, queue_size=10)
        self.cam_pitch_vel_msg = Float32()
        self.cam_pitch_vel_msg.linear.x = self.target_drive.linear.y = self.target_drive_linear.z = 0.0
        self.cam_pitch_vel_msg.angular.x = self.target_drive.angular.y = self.target_drive.angular.z = 0.0
        self.cam_pitch_vel_linear = [0.0, 0.0, 0.0]
        self.cam_pitch_vel_angular = [0.0, 0.0, 0.0]

        self.e_stop_pub = rospy.Publisher('e_stop', Bool, queue_size=10)
        self.e_stop_msg = Bool()
        self.e_stop_msg.data = False
        self._e_stop = False

        self.light_pub = rospy.Publisher('light', Bool, queue_size=10)
        self.light_msg = Bool()
        self.light_msg.data = False
        self._light_button_prev = False
        self._light_state = False

        self.reference_frame_id_pub = rospy.Publisher('reference_frame_id', Int32, queue_size=10) 
        self.reference_frame_id_msg = Int32()
        self.reference_frame_id_msg.data = 0
        self._reference_frame_prev = False
        self._reference_frame_state = False

        self.control_scheme_id = 0

    def _number_verf(number, boundary):
        if number < boundary[0]:
             number = boundary[0]
        elif number > boundary[1]:
             number = boundary[1]

        return number
    
    def control_scheme_callback(self, data):
        self.control_scheme_id = data


    def joy_callback(self, data):
        control_scheme1 = ControlScheme(control_scheme_id)
        # Button implementation for e_stop
        self.e_stop = data.axes[control_scheme1.getBehavior("e_stop")]
        if self.e_stop:    # If emergency stop state
            self.target_drive_linear = self.target_drive_rot = [0.0, 0.0, 0.0]
            self.target_drive_msg.linear.x = self.target_drive.linear.y = self.target_drive.linear.z = 0.0
            self.target_drive_mgs.angular.x = self.target_drive.angular.y = self.target_drive.angular.z = 0.0

            self.e_stop_msg.data = True

            self.light_msg.data = self._light_state = self._light_button_prev = False

        else:     # Not in emergency stop state
            self.e_stop_msg.data = False

            #  Light
            self._update_light_state(data.button[control_scheme1.getBehavior("light_toggle")])
            self.light-msg.data = self._light_state

            # Camera
            self.cam_pitch_vel_linear = data.axes[control_scheme1.getBehavior("camera_pitching")]
            self.cam_pitch_vel_linear.msg = self.cam_pitch_vel_linear

            # trans x, y and z
            self._trans_drive[0] = self._number_verf(data.axes[control_scheme1.getBehavior("trans_x")], [-1.0, 1.0])
            self._trans_drive[1] = self._number_verf(data.axes[control_scheme1.getBehavior("trans_y")], [-1.0, 1.0])
            self._trans_drive[2] = self._number_verf(data.axes[control_scheme1.getBehavior("trans_z")], [-1.0, 1.0])

            self.target_drive_msg.linear.x = self._trans_drive[0]
            self.target_drive_msg.linear.y = self._trans_drive[1]
            self.target_drive_msg.linear.z = self._trans_drive[2]

            # rot x, y and z
            self._trans_drive_rot[0] = self._number_verf(data.axes[control_scheme1.getBehavior("rot_x")], [-1.0, 1.0])
            self._trans_drive_rot[1] = self._number_verf(data.axes[control_scheme1.getBehavior("rot_y")], [-1.0, 1.0])
            self._trans_drive_rot[2] = self._number_verf(data.axes[control_scheme1.getBehavior("rot_z")], [-1.0, 1.0])


            self.target_drive_msg.angular.x = self._trans_drive_rot[0]
            self.target_drive_msg.angular.y = self._trans_drive_rot[1]
            self.target_drive_msg.angular.z = self._trans_drive_rot[2]

            # reference frame toggle
            self._update_reference_state(data.button[control_scheme1.getBehavior("reference_frame_toggle")])
            self.refernece_frame_id_msg = self._reference_frame_state

        self.drive_pub.publish(self.target_drive_msg)
        self.e_stop_pub.publish(self.e_stop_msg)
        self.light_pub.publish(self.light_msg)
        self.reference_frame_id_pub.publish(self.reference_frame_id_msg)
        self.cam_pitch_vel_pub.publish(self.cam_pitch_vel_msg)

    def start(self):
        rospy.Subscriber('joystick', Joy, self.joy_callback)
        rospy.Subscriber('control_scheme_id', Int32, self.control_scheme_callback)

        rospy.init_node('Joy2Cmd')
        rospy.spin()

    @property
    def e_stop(self):
        return self._e_stop

    @e_stop.setter
    def e_stop(self, value):
        if self._e_stop:
            pass
        else:
            self._e_stop = value
            
    @property
    def reference_frame_state(self):
        return self._reference_frame_state

    @reference_frame_state.setter
    def reference_frame_state(self, value):
        pass

    def _update_reference_state(self, current_ref_input)
        if (self._reference_frame_prev == False) and (current_ref_input == True):
            self._reference_frame_state = not self._reference_frame_state)
        self._reference_frame_prev = current_ref_input)

    @property
    def light_state(self):
        return self._light_state

    @light_state.setter
    def light_state(self, value):
        pass

    def _update_light_state(self, current_light_input):
        if (self._light_button_prev == False) and (current_light_input == True):
            self._light_state = not self._light_state
        self._light_button_prev = current_light_input    

if __name__ == "__main__":
    try:
        Joy2Command = Joy2Cmd()
        Joy2Command.start()
    except rospy.ROSInterruptException:
        pass



