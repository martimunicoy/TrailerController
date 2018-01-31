#!/usr/bin/env python
# -*- coding: utf-8 -*-


# General imports
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

# Custom imports
from animation import Car, Trailer, Animation, save
from controller import Controller
from util import cl_parser, file_parser, print_license_header

import constants as co


# Main behavior
if __name__ == "__main__":
    # Print license header
    print_license_header()

    # Parse command-line arguments
    input_file = cl_parser()
    parsed_args = file_parser(input_file)

    # Set display configuration
    fig, axis = plt.subplots(1)
    plt.gca().set_aspect('equal', adjustable='box')
    if "X_AXIS" in parsed_args:
        axis.set_xlim(float(parsed_args["X_AXIS"][0]),
                      float(parsed_args["X_AXIS"][1]))
    else:
        axis.set_xlim(0, 40)

    if "Y_AXIS" in parsed_args:
        axis.set_ylim(float(parsed_args["Y_AXIS"][0]),
                      float(parsed_args["Y_AXIS"][1]))
    else:
        axis.set_ylim(0, 40)

    # Set variables according to either input values or default values
    # Set L1
    if "L1" in parsed_args:
        L1 = float(parsed_args["L1"])
    else:
        L1 = co.L1

    # Set L2
    if "L2" in parsed_args:
        L2 = float(parsed_args["L2"])
    else:
        L2 = co.L2

    # Set L3
    if "L3" in parsed_args:
        L3 = float(parsed_args["L3"])
    else:
        L3 = co.L3

    # Define car
    my_car = Car(L1, L2, axis, fig)

    # Define trailer
    my_trailer = Trailer(L3, axis, fig)

    # Initiate controller
    my_controller = Controller(my_car, my_trailer, dt=co.DT,
                               default_steps=co.STEPS, default_V=co.V)

    # Define trajectory points according to either input values or some default
    # ones. Then, add them to controller
    if "POINTS" in parsed_args:
        for point in parsed_args["POINTS"]:
            my_controller.add_trajectory_point(point)
    else:
        my_controller.add_trajectory_point((25, 5))
        my_controller.add_trajectory_point((10, 10))
        my_controller.add_trajectory_point((5, 25))

    # Stablish initial state of the system according to either input values or
    # default values
    # Theta1
    if "THETA1" in parsed_args:
        theta1 = float(parsed_args["THETA1"])
    else:
        theta1 = co.THETA1

    # Phi
    if "PHI" in parsed_args:
        phi = float(parsed_args["PHI"])
    else:
        phi = co.PHI

    my_controller.initiate(theta1, phi)

    # Display trajectory points
    my_controller.display_points(axis)

    # Initiate simulation
    my_controller.simulate()

    # Set animation parameters according to either input values or default
    # values
    # Repetitions
    if "REPEAT" in parsed_args:
        repeat = \
            parsed_args["REPEAT"].upper() == "TRUE" or \
            parsed_args["REPEAT"].upper() == "T" or \
            parsed_args["REPEAT"].upper() == "1"
    else:
        repeat = co.ANIMATION_REPEAT

    # Interval
    if "INTERVAL" in parsed_args:
        interval = int(parsed_args["INTERVAL"])
    else:
        interval = co.ANIMATION_INTERVAL

    # Run simulation animation
    animation = Animation(my_car, my_trailer, my_controller.state_points)
    simulation = FuncAnimation(fig, animation.update, interval=interval,
                               frames=len(my_controller.state_points),
                               repeat=repeat, blit=True)

    # Save simulation to a file
    if "SAVE" in parsed_args:
        save(simulation, parsed_args["SAVE"])

    # Show animation on screen
    animation.show()
