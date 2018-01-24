import sys
from matplotlib import pyplot as plt
import numpy as np
from numpy import sin, tan, cos, arctan
from matplotlib.animation import FuncAnimation
from animation import Car, Trailer, Animation

# Constants
CONTROLLER_MAX_ANGLE = 0.15
CONTROLLER_DELTA_STEP = 0.05
CONTROLLER_MAX_DELTA = 0.7

# Initial variables
initial_phi = 0


def phi_predictor(car, trailer, state, V, dt):
    new_phi = state["phi"] + (V / trailer.L3 * sin(state["phi"]) + V / car.L1 *
                              tan(state["delta"]) * (1 + car.L2 *
                                                     cos(state["phi"]) /
                                                     trailer.L3)) * dt
    state["phi"] = new_phi


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


def turning_center(car, radius, state):
    if state["delta"] > 0:
        sign = -1
    else:
        sign = +1

    dx = car.L2 * cos(state["theta1"]) + radius * sin(state["theta1"]) * sign
    dy = car.L2 * sin(state["theta1"]) - radius * cos(state["theta1"]) * sign

    return np.array((dx, dy))


def turn_around(car, radius, center, state, V, dt):
    if state["delta"] > 0:
        sign = -1
    else:
        sign = +1

    alpha = arctan(car.L2 / radius)
    hitch_radius = np.hypot(radius, car.L2)
    angularV = sign * V / radius
    arc = angularV * dt
    state["theta1"] -= arc
    state["theta1"] = abs_1st_period_angle(state["theta1"])

    dx = hitch_radius * cos(state["theta1"] + (np.pi / 2 + alpha) * sign)
    dy = hitch_radius * sin(state["theta1"] + (np.pi / 2 + alpha) * sign)

    state["current_position"] = (np.array((dx, dy)) + center)


def staight_ahead(state, V, dt):
    state["current_position"] = state["current_position"] + \
        np.array(V * cos(state["theta1"]) * dt, V * sin(state["theta1"]) * dt)


def get_delta_from_phi(delta_phi, phi, V, L1, L2, L3, dt):
    delta = np.arctan((-L1 * (delta_phi * L3 - sin(phi) * dt * V)) / (
                      dt * V * (L3 + L2 * cos(phi))))

    if (delta > np.pi / 2.) and (delta < 3 * np.pi / 2):
        delta -= np.pi

    return delta


def move_wheels(old_delta, delta):
    gain = min(np.abs(delta), CONTROLLER_DELTA_STEP)

    if delta > old_delta:
        delta = old_delta + gain
    elif delta < old_delta:
        delta = old_delta - gain
    else:
        return old_delta

    if np.abs(delta) < 1e-5:
        return 0

    if delta > 0:
        return min(delta, CONTROLLER_MAX_DELTA)
    else:
        return max(delta, -CONTROLLER_MAX_DELTA)


def fix_large_phi(car, trailer, points, state, dt, V):
    while (np.abs(state["phi"] - np.pi) > CONTROLLER_MAX_ANGLE / 2):
        # Get new delta
        if state["phi"] < np.pi:
            delta_phi = -1
        else:
            delta_phi = +1
        state["delta"] = move_wheels(state["delta"],
                                     get_delta_from_phi(delta_phi,
                                                        state["phi"], V,
                                                        car.L1, car.L2,
                                                        trailer.L3, dt))

        # Run simulation step
        simulation_step(car, trailer, state, V, dt)

        # Add new state point
        add_state_to_point(state, points)


def simulation_step(car, trailer, state, V, dt):
    radius = turning_radius(car, state["delta"])

    if radius is not None:
        center = state["current_position"] + turning_center(car, radius,
                                                            state)
        turn_around(car, radius, center, state, V, dt)

    else:
        state["delta"] = 0
        staight_ahead(state, V, dt)

    phi_predictor(car, trailer, state, V, dt)
    state["theta2"] = state["theta1"] + state["phi"]


def add_state_to_point(state, points):
    points.append((state["current_position"], state["theta1"],
                  state["theta2"], state["delta"]))


def test_turn_around():
    # Model parameters
    L1 = 2.2
    L2 = 1.2
    L3 = 2
    V = -10
    delta = 0.2

    # Simulation parameters
    dt = 0.1
    steps = 100

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

    for i in xrange(steps):
        axis.plot(center[0], center[1], color='red', marker=".", markersize=3)
        radius = turning_radius(my_car, delta)
        center = position + turning_center(my_car, radius, theta1, delta)
        print radius, center
        shift, theta1 = turn_around(my_car, radius, theta1, center, V, delta,
                                    dt)
        axis.plot(shift[0], shift[1], color='blue', marker=".", markersize=3)
        print center
        position = shift

    plt.show()


def test_fix_large_phi():
    # Model parameters
    L1 = 2.2
    L2 = 1.2
    L3 = 2
    V = -2

    # Simulation parameters
    dt = 0.05
    steps = 1000

    # Set the figure and axis for the animation
    fig, axis = plt.subplots(1)
    plt.gca().set_aspect('equal', adjustable='box')
    axis.set_xlim(0, 30)
    axis.set_ylim(0, 30)
    my_car = Car(L1, L2, axis, fig)
    my_trailer = Trailer(L3, axis, fig)

    # Define initial state of the system
    state = {}
    state["theta1"] = 0.1
    state["phi"] = np.pi - 0.1
    state["theta2"] = state["theta1"] + state["phi"]
    state["current_position"] = np.array((15, 15))
    state["delta"] = 0.2

    # Initiate list to save all state points
    points = []
    points.append((state["current_position"], state["theta1"], state["theta2"],
                   state["delta"]))

    # Start simulation
    for i in xrange(steps):
        # In case of a large phi angle, fix it
        if np.abs(state["phi"] - np.pi) > CONTROLLER_MAX_ANGLE:
            fix_large_phi(my_car, my_trailer, points, state, dt, V)

        # Restrain again delta value, only for testing purposes
        state["delta"] = move_wheels(state["delta"], 0.2)

        # Run simulation step
        simulation_step(my_car, my_trailer, state, V, dt)

        # Add new state point
        add_state_to_point(state, points)

    # Display simulation
    print(len(points))
    animation = Animation(my_car, my_trailer, points)
    simulation = FuncAnimation(fig, animation.update, interval=10,
                               frames=len(points),
                               repeat=True, blit=True)
    animation.show()


if __name__ == "__main__":
    #test_turn_around()
    test_fix_large_phi()
