#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


class ThrustScaler:
    def __init__(self):
        self.thrust_bounds = ((-1000, 1000), (-1000, 1000), (-1000, 1000))
        self.setpoint_msg = Twist()
        self.setpoint_msg.linear.x = 0
        self.setpoint_msg.linear.y = 0
        self.setpoint_msg.linear.z = 0
        self.setpoint_pub = None

    def scale_setpoint(self, data):
        trans = [data.linear.x, data.linear.y, data.linear.z]
        angular = [data.angular.x, data.angular.y, data.angular.z]

        if trans[0] <= 0:
            x_scaled = trans[0] * abs(self.thrust_bounds[0][0])
        elif trans[0] > 0:
            x_scaled = trans[0] * abs(self.thrust_bounds[0][1])

        if trans[1] <= 0:
            y_scaled = trans[1] * abs(self.thrust_bounds[1][0])
        elif trans[1] > 0:
            y_scaled = trans[1] * abs(self.thrust_bounds[1][1])

        if trans[2] <= 0:
            z_scaled = trans[2] * abs(self.thrust_bounds[2][0])
        elif trans[2] > 0:
            z_scaled = trans[2] * abs(self.thrust_bounds[2][1])

        if rot[0] <= 0:
            x_scaled = trans[0] * abs(self.thrust_bounds[0][0])
        elif trans[0] > 0:
            x_scaled = trans[0] * abs(self.thrust_bounds[0][1])

        if trans[1] <= 0:
            y_scaled = trans[1] * abs(self.thrust_bounds[1][0])
        elif trans[1] > 0:
            y_scaled = trans[1] * abs(self.thrust_bounds[1][1])

        if trans[2] <= 0:
            z_scaled = trans[2] * abs(self.thrust_bounds[2][0])
        elif trans[2] > 0:
            z_scaled = trans[2] * abs(self.thrust_bounds[2][1])

        rot_x_scaled, rot_y_scaled, rot_z_scaled = (angular[0], angular[1], angular[2])

        scaled_setpoint_trans = [trans_x_scaled, trans_y_scaled, trans_z_scaled]
        scaled_setpoint_rot = [rot_x_scaled, rot_y_scaled, rot_z_scaled]

        self.setpoint_msg.linear.x = scaled_setpoint_trans[0]
        self.setpoint_msg.limear.y = scaled_setpoint_trans[1]
        self.setpoint_msg.linear.z = scaled_setpoint_trans[2]

        self.setpoint_msg.angular.x = scaled_setpoint_rot[0]
        self.setpoint_msg.angular.y = scaled_setpoint_rot[1]
        self.setpoint_msg.angular.z = scaled_setpoint_rot[2]


        self.setpoint_pub.publish(self.setpoint_msg)


    def start(self):
        pass


if __name__ == "__main__":
    try:
        input_proc = InputProcessor()
        input_proc.start()
    except rospy.ROSInterruptException:
        pass