#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np

# Lateral control

# TODO: write the PID controller using what you've learned in the previous activities

# Note: y_hat will be calculated based on your DeltaPhi() and poseEstimate() functions written previously 

def PIDController(
    v_0, # assume given (by the scenario)
    y_ref, # assume given (by the scenario)
    y_hat, # assume given (by the odometry)
    prev_e_y, # assume given (by the previous iteration of this function)
    prev_int_y, # assume given (by the previous iteration of this function)
    delta_t): # assume given (by the simulator)
    """
    Args:
        v_0 (:double:) linear Duckiebot speed.
        y_ref (:double:) reference lateral pose
        y_hat (:double:) the current estiamted pose along y.
        prev_e_y (:double:) tracking error at previous iteration.
        prev_int_y (:double:) previous integral error term.
        delta_t (:double:) time interval since last call.
    returns:
        v_0 (:double:) linear velocity of the Duckiebot 
        omega (:double:) angular velocity of the Duckiebot
        e_y (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
        e_int_y (:double:) current integral error (automatically becomes prev_int_y at next iteration).
    """
    
    # TODO: these are random values, you have to implement your own PID controller in here    
    
    # Error:
    e_y = y_ref - y_hat
    # Integral if the error:
    e_int = prev_int_y + e_y * delta_t
    # Anti-windup for the integral
    e_int = max(min(e_int, 10), -10)
    # Derivative of the error
    e_der = (e_y - prev_e_y) / delta_t
    
    k_e = 10
    k_i = 0
    k_d = 100
    
    omega = k_e * e_y + k_i * e_int + k_d * e_der
    # print(f"\ny_hat:{y_hat}")
    # print(f"e:{e} \n e_int:{e_int} \n e_der:{e_der} \n omega: {omega}")
        
    return [v_0, omega], e_y, e_int

