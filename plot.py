import matplotlib.pyplot as plt
import array as arr
from TC2 import magic_formula, generator

#-- Parameters --#
wheelTorque = 200 #Newtons/s
carStartingVelocity = 1 #m/s
wheelStartingVelocity = 1 #m/s
speedTarget = 40 #m/s

#vehicle_speed, wheel_speed, wheel_slippage_percent = 

if __name__ == '__main__':

    car_velocity, wheel_velocity = generator(carStartingVelocity, speedTarget, wheelStartingVelocity, wheelTorque)

    i = 0

    while car_velocity[i] < speedTarget:
        i = i + 1
    time = arr.array('f', [0])

    k = 1

    while k <= i:
        time.append(k / 50)
        k = k + 1

    k = 1

    startSlip = ((wheel_velocity[k] / car_velocity[k]) - 1) * 100
    slip = arr.array('f', [startSlip])

    while k <= i:
        slip.append(((wheel_velocity[k] / car_velocity[k]) - 1) * 100)
        k = k + 1

   
    #Graphing
    fig, ax1 = plt.subplots()
    ax1.plot(time, wheel_velocity, label = 'Wheel Velocity')
    ax1.plot(time, car_velocity, label = 'Car Velocity')
    ax2 = ax1.twinx()
    ax1.set_xlim([0,20])
    ax2.set_ylim([0, 100])
    ax2.plot(time, slip, label = 'Slippage', color = 'Red')
    ax1.legend()
    ax2.legend()
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Velocity (M/s)')
    ax2.set_ylabel('Slippage (%)')

    plt.show()
