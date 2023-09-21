import numpy as np

"""
This script provides some functions for controlling purposes:
- Different versions of Thrust_to_PWM function:
    + Thrust_to_PWM(thrust, alpha)
    + Thrust_to_PWM_v1(thrust)
    + Thrust_to_PWM_modified(thrust, mass)
- Compute the control input for Crazyflie: 

- Feedback controller function: compute_control(v_ref, x0, xref, Kf)

To control the Crazyflie drone, we need to send a packet of control input over the simulation time.

The approach that we used is based on flatness control.This will allow us to linearize the non-linear system in closed loop
by applying a method called feedback linearization.
The result of this method is a double integrator dynamics.


"""
def Thrust_to_PWM(Thrust,alpha):
    """
    Reference:
    https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/pwm-to-thrust/
    """
    pwm_signal = 65535 * (-140.5e-3*alpha + np.sqrt(140.5e-3 ** 2 - 4 * 0.409e-3 * (-0.099 - Thrust))) / (2 * 0.409e-3 * 256)
    return pwm_signal

def Thrust_to_PWM_v1(Thrust):
    """
    Reference:
    https://www.research-collection.ethz.ch/handle/20.500.11850/214143

    """
    a1 = 2.130295e-11
    a2 = 1.032633e-6
    a3 = 5.484560e-4
    m = 33.0 # quadcopter mass : from 28 to 33 gram
    g = 9.81 # gravitational acceleration
    kc = m*g/4000
    m_ratio = 1.9
    PWM_theo = (-a2 + np.sqrt(a2**2 + 4*a1*(Thrust * kc - a3)))/(2*a1*m_ratio)
    # Mapping 0-65535 to 10000-60000 
    alpha = 0.7630
    beta = 10000
    pwm_signal = alpha * PWM_theo + beta
    return pwm_signal

def Thrust_to_PWM_modified(Thrust,m=33.0):
    """
    Reference:
    https://www.research-collection.ethz.ch/handle/20.500.11850/214143

    """
    a1 = 2.130295e-11
    a2 = 1.032633e-6
    a3 = 5.484560e-4
    # a2 = 1.00133e-6
    # a3 = -9.51544e-4

    g = 9.81 # gravitational acceleration
    kc = m*g/4000
    PWM_signal = (-a2 + np.sqrt(a2**2 + 4*a1*(Thrust * kc - a3)))/(2*a1)
    return PWM_signal

def compute_control(v_ref, x0, xref, Kf):
    """ Feedback controller for Crazyflie """
    v = v_ref + np.matmul(Kf, x0 - xref)
    return v

def get_real_input(v_controls,yaw):
    """ Compute the control input for Crazyflie [Thrust, Roll, Pitch, Yaw(default=0)] """

    g = 9.81
    T = np.round(np.sqrt(v_controls[0] ** 2 + v_controls[1] ** 2 + (v_controls[2] + g) ** 2), 5)
    # print(T)
    phi = np.round(np.arcsin((v_controls[0] * np.sin(yaw) - v_controls[1] * np.cos(yaw)) / T), 5)
    theta = np.round(np.arctan((v_controls[0] * np.cos(yaw) + v_controls[1] * np.sin(yaw)) / (
                v_controls[2] + g)), 5)
    controls = [T, phi, theta]
    return controls

def get_cf_input(v_controls, yaw, T_coeff=23.5, desired_yaw = 0, alpha=1.000, mass = 33.0, bias=[0,0]):
    g = 9.81
    T = np.round(np.sqrt(v_controls[0] ** 2 + v_controls[1] ** 2 + (v_controls[2] + g) ** 2), 5)
    phi = np.round(np.arcsin((v_controls[0] * np.sin(yaw) - v_controls[1] * np.cos(yaw)) / T), 5)
    theta = np.round(np.arctan((v_controls[0] * np.cos(yaw) + v_controls[1] * np.sin(yaw)) / (
                v_controls[2] + g)), 5)
    controls = [T, phi, theta]
    # Thrust_pwm = int(T_coeff*Thrust_to_PWM(controls[0] / g,alpha))
    Thrust_pwm = int(T_coeff*Thrust_to_PWM_modified(controls[0]/g,mass)*alpha)
    print('Thrust_pwm = ',Thrust_pwm)
    Roll = (controls[1] * 180) / np.pi  
    Pitch = (controls[2] * 180) / np.pi
    Yawrate = 0.000*(yaw - desired_yaw) * 180 / np.pi
    controls_cf = [Roll+bias[0], Pitch+bias[1], Yawrate, Thrust_pwm]
    return controls_cf
