#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

This python file implements the dbw_node publishers and subscribers. 
You will need to write ROS subscribers for the /current_velocity, /twist_cmd, and /vehicle/dbw_enabled topics. 
This file also imports the Controller class from twist_controller.py which will be used for implementing the necessary controllers. 
The function used to publish throttle, brake, and steering is publish.

Note that throttle values passed to publish should be in the range 0 to 1, 
although a throttle of 1 means the vehicle throttle will be fully engaged. 
Brake values passed to publish should be in units of torque (N*m). 
The correct values for brake can be computed using the desired acceleration, weight of the vehicle, and wheel radius.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')
        
        #create a ditionary with param data
        param_data = {}
        param_data['vehicle_mass'] = rospy.get_param('~vehicle_mass', 1736.35)
        param_data['fuel_capacity'] = rospy.get_param('~fuel_capacity', 13.5)
        param_data['brake_deadband'] = rospy.get_param('~brake_deadband', .1)
        param_data['decel_limit'] = rospy.get_param('~decel_limit', -5)
        param_data['accel_limit'] = rospy.get_param('~accel_limit', 1.)
        param_data['wheel_radius'] = rospy.get_param('~wheel_radius', 0.2413)
        param_data['wheel_base'] = rospy.get_param('~wheel_base', 2.8498)
        param_data['steer_ratio'] = rospy.get_param('~steer_ratio', 14.8)
        param_data['max_lat_accel'] = rospy.get_param('~max_lat_accel', 3.)
        param_data['max_steer_angle'] = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Subscribe to all the topics you need to
        # dbw_enabled
        self.dbw_enabled = None
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        # twist
        self.twist_cmd = None
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)

        # current velocity
        self.current_velocity = None
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        
        # TODO: Create `Controller` object
        self.controller = Controller(param_data)       
        
        # TODO: Call publish
        self.publish_cb()



    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg

    def twist_cmd_cb(self, msg):
        self.twist_cmd = msg

    def current_velocity_cb(self, msg):
        self.current_velocity = msg

    def publish_cb(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            if not None in [self.controller, self.twist_cmd, self.current_velocity, self.dbw_enabled]:
                throttle, brake, steer = self.controller.control(self.twist_cmd.twist.linear.x, 
                                                                self.twist_cmd.twist.angular.z,
                                                                self.current_velocity.twist.linear.x,
                                                                self.dbw_enabled)
                # FIXME: remove
                rospy.loginfo('proposed linear velocity: %f', self.twist_cmd.twist.linear.x)
                rospy.loginfo('proposed angular velocity: %f', self.twist_cmd.twist.angular.z)
                rospy.loginfo('current linear velocity: %f', self.current_velocity.twist.linear.x)
                rospy.loginfo('steer: %f', steer)
                rospy.loginfo('throttle: %f', throttle)
                rospy.loginfo('brake: %f', brake)

                if self.dbw_enabled:
                    self.publish(throttle, brake, steer)
            
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    try:
        DBWNode()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

