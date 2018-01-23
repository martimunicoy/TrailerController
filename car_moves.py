import sys
from matplotlib import pyplot as plt
import numpy as np
from numpy import sin, tan, cos, arctan
from matplotlib.animation import FuncAnimation
from animation import Car, Trailer, Animation


# Initial variables
initial_phi = 0


def abs_1st_period_angle(angle):
    summand = np.pi * 2

    if angle > 0:
        while angle >= summand:
            angle -= summand
    else:
        while angle < 0:
            angle += summand

    return angle


def turning_radius(car, delta):
    if tan(delta) == 0:
        return None
    else:
        return np.abs(car.L1 / tan(delta))


def turning_center(car, radius, theta1, delta):
    if delta > 0:
        sign = -1
    else:
        sign = +1
    #theta1 = abs_1st_period_angle(theta1)

    dx = car.L2 * cos(theta1) + radius * sin(theta1) * sign
    dy = car.L2 * sin(theta1) - radius * cos(theta1) * sign
    return np.array((dx, dy))


def turn_around(car, radius, theta1, center, V, delta, dt):
    if delta > 0:
        sign = -1
    else:
        sign = +1

    alpha = arctan(car.L2 / radius)
    hitch_radius = np.hypot(radius, car.L2)
    angularV = sign * V / radius
    arc = angularV * dt
    theta1 -= arc

    dx = hitch_radius * cos(theta1 + (np.pi / 2 + alpha) * sign)
    dy = hitch_radius * sin(theta1 + (np.pi / 2 + alpha) * sign)

    shift = (np.array((dx, dy)) + center)

    #theta1 = abs_1st_period_angle(theta1)

    return shift, theta1


if __name__ == "__main__":
    # Model parameters
    L1 = 2.2
    L2 = 1.2
    L3 = 2
    V = -10
    delta = 0.2

    # Simulation parameters
    dt = 0.1
    steps = 1000

    fig, axis = plt.subplots(1)
    plt.gca().set_aspect('equal', adjustable='box')
    axis.set_xlim(0, 30)
    axis.set_ylim(0, 30)
    my_car = Car(L1, L2, axis, fig)
    my_trailer = Trailer(L3, axis, fig)

    my_car.initiate()
    my_trailer.initiate()

    theta1 = 0.1
    theta2 = 3
    position = np.array((15, 15))
    my_car._update(position, theta1, delta)
    my_trailer._update(position, theta2)

    radius = turning_radius(my_car, delta)
    center = position + turning_center(my_car, radius, theta1, delta)


    for i in xrange(10):
        if delta > 0:
            sign = -1
        else:
            sign = +1
        axis.plot(center[0], center[1], color='red', marker=".", markersize=3)
        radius = turning_radius(my_car, delta)
        center = position + turning_center(my_car, radius, theta1, delta)
        print radius, center
        shift, theta1 = turn_around(my_car, radius, theta1, center, V, delta, 0.1)
        axis.plot(shift[0], shift[1], color='blue', marker=".", markersize=3)
        print center
        position = shift

    plt.show()
