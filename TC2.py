import math
#Use the formula A=F/M for finding the acceleration and update velocity based on that
#Then according to the slip ratio you will find the acceleration which is based on wheel velocities
from numpy import linspace, arctan, sin
import array as arr

mass = 177.5 #kg
gravity = 9.81 #m/s^2
weight = mass * gravity #kg * m/s^2


Pk = 2 #Porportional gain 0-255
Ik = .3 #intergral gain 0-255
Dk = 20 #Derivative gain 0-255


#Variables that is important for wheel slippage
wheelAngularVelocity = 0 #radians/s (wheel speed)
wheelAngularAccel = 28.2744 #radians/s^2 (wheel torque basically)
maxAngularAccel = 1 #radians/s^2
throttlePercent = 0 #same as wheelAngularAccel but as percentage
wheelRadius = 0.432 #m
wheelHubLVelocity = 0.1 #m/s (car speed)
tireTreadLVelocity = wheelRadius * wheelAngularVelocity #radius * radians/sec
wheelSlipVelocity = tireTreadLVelocity - wheelHubLVelocity #difference between car speed and actual wheel speed
wheelSlip = wheelSlipVelocity / wheelHubLVelocity #Percentage of wheel slippage
slippage = 1.0 #Used for starting wheel slippage

    
def rotation_to_radians(rotation):
    radians = rotation * 6.2832
    return radians

def ms_to_radians(ms):
    radians = ms / wheelRadius
    return radians

def radians_to_ms(radians):
    ms = radians * wheelRadius
    return ms

def generator(vehicle_velocity, speedTarget, wVelocity, torqueWheel):
    target = 1.08 #Target wheel slippage for PID controller to get to
    time = arr.array('i', [0]) #Time in seconds
    velocity = arr.array('f', [vehicle_velocity]) #Velocity of car
    wheel_velocity = arr.array('f', [wVelocity]) #Velocity of wheels
    wheelAngularAccel = (2*torqueWheel) / (8.6 * wheelRadius)
    wheelAngularAccel = radians_to_ms(wheelAngularAccel)
    i = 0
    wheelTorqueConstant = torqueWheel
    throttlePercent = 1.0
    slippage = 1.0
    while i < 1000000:
        if (vehicle_velocity >= speedTarget):
            return velocity, wheel_velocity
        wheelAngularAccel = (2*torqueWheel) / (8.6 * wheelRadius)
        wheelAngularAccel = radians_to_ms(wheelAngularAccel)
        prevError = target - slippage
        slippage = (vehicle_velocity + wheelAngularAccel) / vehicle_velocity
        if slippage < 1:
            slippage = 1.0
        F = magic_formula(weight, slippage)
        LongAccel = (F / mass) / 50
        vehicle_velocity = vehicle_velocity + LongAccel
        wheelAngularVelocity = slippage * vehicle_velocity
        velocity.append(velocity[i] + LongAccel)
        wheel_velocity.append(wheelAngularVelocity)
        error = target - slippage
        throttlePercent = throttlePercent + porportional(error) + integral(error, i) + derivative(error, prevError, i)
        if throttlePercent > 1.0:
            throttlePercent = 1.0
        if throttlePercent < 0.0:
            throttlePercent = 0.0
        torqueWheel = throttlePercent * (wheelTorqueConstant / 100)
        if torqueWheel < 0:
            torqueWheel = 0
        if torqueWheel > 100:
            torqueWheel = 100
        #if i == 100:
            #target = 1.08
        i = i + 1
        

    return velocity, wheel_velocity


def porportional(error):
    P = Pk * error
    return P

def integral(error, time):
    I = Ik * error * time
    return I

def derivative(error, prevError, time):
    if time == 0:
        return 0
    D = Dk * (error * prevError) / time
    return D


#Magic forumla is used for simulation of certain surfaces to tire contact and the dynamics of them interacting with each other.

def magic_formula(w, p, b=10, c=1.9, d=1, e=.97):
    """

    The magic formula [1] computes the longitudinal force between wheel and tarmac
    in function of the longitudinal tire slip.

    [1] Pacejka, H. B. Tire and Vehicle Dynamics. Elsevier Science, 2005

    Parameters
    ----------
    num : int, optional
        Number of samples to generate. Default is 50. Must be non-negative.
    p : float
        Weight applied on the tire [N].
    b : float
        Stiffness factor. Default value corresponds to dry tarmac.
    c : float
        Shape factor. Default value corresponds to dry tarmac.
    d : float
        Peak factor. Default value corresponds to dry tarmac.
    e : float
        Curvature factor. Default value corresponds to dry tarmac.

    Returns
    -------
    F : float or array
        If x is a slip ratio, Func is the longitudinal force between tire and tarmac [N], if x is a wheel slip angle, Func
        is the lateral force.
    alpha : float, array_like
        Tire slip ratio or slip angle.
    """
    p = p - 1
    F = w * d * sin(c * arctan(b * p * (1 - e) + e * b * p - arctan(b * p))) #Made a change in order for python to recognize the formula better
    #F = w * d * sin(c * arctan(b * p - e(b * p - arctan(b * p))))
    return F
        
