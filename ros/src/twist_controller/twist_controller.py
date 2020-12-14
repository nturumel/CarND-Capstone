from pid import PID
from yaw_controller import YawController 
from lowpass import LowPassFilter  
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_BRAKE = 400.0

class Controller(object):
    def __init__(self, param_data):
        
        # TODO: Implement
        # initialise global parameters field
        self.param_data = param_data
        
        # initialise throttle controller (kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM)
        kp = 0.3 
        ki = 0.1
        kd = 0.
        mn = 0. # minimum throttle value
        mx = 0.4 # maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        
        # initialise yaw controller (wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.yaw_controller = YawController(self.param_data['wheel_base'], self.param_data['steer_ratio'], 0.1 , self.param_data['max_lat_accel'], self.param_data['max_steer_angle'])

        # initialise the low pass filter
        # FIXME: figure out why a low pass filter is used
        tau = 0.5 # cutoff frequency, 1/(2*pi*tau)
        ts = 0.02 # interval time, 1/50 Hz
        self.vel_lpf = LowPassFilter(tau, ts)
        self.time = rospy.get_time()
        
        pass

    def control(self, proposed_velocity, proposed_angular_velocity, current_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0, 0, 0
        #steering
        steering = self.yaw_controller.get_steering(proposed_velocity, proposed_angular_velocity, current_velocity)
        error = proposed_velocity - current_velocity 
        
        # brake and throttle
        current_time = rospy.get_time()
        delta_t = current_time - self.time
        self.time = current_time
        throttle = self.throttle_controller.step(error, delta_t)
        brake = 0

        if proposed_velocity == 0 and current_velocity < 0.1: # need to stop
            # set throttle to zero and brake to max
            throttle = 0
            brake = MAX_BRAKE 

        elif throttle < 0.1 and error < 0: # need to slow down
            # set throttle to zero and calculate the break
            throttle = 0
            # FIXME: needs to replaced by fuel left
            vehicle_mass  = self.param_data['vehicle_mass'] + GAS_DENSITY * self.param_data['fuel_capacity']  
            decel = min(error, self.param_data['decel_limit'])
            brake = abs((vehicle_mass) * self.param_data['wheel_radius'] * (error / delta_t))


        return throttle, brake, steering
