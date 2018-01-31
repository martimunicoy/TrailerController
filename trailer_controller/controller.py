#!/usr/bin/env python
# -*- coding: utf-8 -*-


# General imports
import numpy as np
from random import uniform
from numpy import sin, tan, cos, arctan

# Custom imports
import constants as co
from util import first_period_angle


# This script mainly depends on the following class:
class Controller():

    def __init__(self, car, trailer, dt=0.01, default_steps=1000,
                 default_V=-10):
        self.car = car
        self.trailer = trailer
        self.dt = dt
        self.steps = default_steps
        self.V = default_V
        self.state_points = []
        self.trajectory_points = []
        self.state = None

    # Save the current state of the simulation to be displayed in the animation
    def add_simulation_state(self):
        self.state_points.append((self.state["current_position"],
                                 self.state["theta1"],
                                 self.state["theta2"],
                                 self.state["delta"]))

    # Add one point to the trajectory
    def add_trajectory_point(self, point):
        # If point is not a numpy array, convert it
        if type(point) is list or type(point) is tuple:
            point = np.array(point)

        # Add point to trajectory list
        self.trajectory_points.append(point)

    # Display the trajectory points in the animation plot
    def display_points(self, axis):
        for point in self.trajectory_points:
            axis.plot(point[0], point[1], color='red', marker=".",
                      markersize=3)

    # Define initial state of the system
    def initiate(self, theta1=0, phi=np.pi):
        self.state = {}
        self.state["theta1"] = theta1
        self.state["phi"] = phi
        self.state["theta2"] = self.state["theta1"] + self.state["phi"]
        self.state["delta"] = 0

    # Perform a single simulation step, after having chosen a corresponding
    # delta
    def simulation_step(self):
        # Get turning radius
        radius = turning_radius(self)

        # If radius is None, it means that the car has a linear trajectory
        if radius is not None:
            # Get turning center
            center = self.state["current_position"] + turning_center(self,
                                                                     radius)
            # Make it turn
            turn_around(self, radius, center)

        else:
            # Make it go straight
            straight_ahead(self)

        # Predit the new phi angle according to the movement of the car
        phi_predictor(self)
        # Update theta2 angle according to the predicted phi
        # Do not touch this! Phi is predicted by the model in a way that needs
        # to be substracted from theta1 in order to get theta2
        self.state["theta2"] = self.state["theta1"] - self.state["phi"]

    # Execute a simulation using the controller
    def simulate(self, V=None, max_steps=None):
        # Check trajectory points
        if ((len(self.trajectory_points) < 2) or
                (self.trajectory_points is None)):
            print "Error: add a minimum of two trajectory trajectory points" \
                  " with add_trajectory_point() before calling simulate()" \
                  " function"
            return

        # Check initial state
        if self.state is None:
            print "Error: initial state of the system is not defined. Define" \
                  " it previously by calling initiate() function"
            return

        # Add initial position to initial state
        self.state["current_position"] = self.trajectory_points.pop(0)

        # Set velocity V
        if V is not None:
            self.V = V

        # Set maximum number of steps max_steps
        if max_steps is None:
            max_steps = self.steps

        # Print out information about the simulation
        print "Starting simulation of the trailer-controller..."

        # Add first state to state points list
        self.add_simulation_state()

        # In case of an initial large phi angle, fix it
        if np.abs(self.state["phi"] - np.pi) > co.CONTROLLER_MAX_ANGLE:
            fix_large_phi(self)

        # Initialize variables for the loop
        checked_points = 0
        found = False
        goal_point = self.trajectory_points.pop(0)

        # Notify which is the current goal point
        print "- Attempting to approach point ", goal_point

        # Start simulation
        for i in xrange(max_steps):
            # Calculate orientation vectors according to current position
            goal_dir = goal_point - self.state["current_position"]
            theta1_vec = np.array((cos(self.state["theta1"]),
                                   sin(self.state["theta1"])))

            # Face trailer to goal direction
            face_trailer_to_dir(self, goal_dir, theta1_vec)

            # Check if we have arrived to the current goal point
            for point in self.state_points[checked_points:]:
                dist = np.linalg.norm(goal_point - point[0])
                if dist < co.CONTROLLER_THRESHOLD:
                    # If this is the case and no points are left, finish loop
                    if len(self.trajectory_points) == 0:
                        print "Trajectory completed!"
                        goal_point = None
                        found = True
                    else:
                        # If there are remaining points, go to the next one
                        goal_point = self.trajectory_points.pop(0)
                        print "- Attempting to approach point ", goal_point
                    break
                checked_points += 1

            # Remove leftover points
            if found:
                for i in range(checked_points, len(self.state_points)):
                    del(self.state_points[-1])
                if goal_point is None:
                    break
                else:
                    found = False


# Functions used by the controller class are found below, sorted alphabetically
def add_noise(delta):
    noise = uniform(-1, 1)
    return delta + co.CONTROLLER_MAX_NOISE * noise


def face_trailer_to_dir(ctrl, goal_dir, theta1_vec):
    # Get the rotation direction performing the cross product between both
    # vectors. Its sign will indicate the suitable rotation direction
    rotation_direction = np.sign(np.cross(goal_dir, theta1_vec))

    # Get the distance from current position to goal node
    dist = np.linalg.norm(goal_dir)

    # @TODO: constrain effects still need more testing!
    # Select a constrain level depending on the distance left to achieve the
    # goal point
    if dist > 12:
        constrain_lvl = 1
    elif dist > 6:
        constrain_lvl = 2
    else:
        constrain_lvl = 3

    # Face trailer
    while (np.abs(ctrl.state["phi"] - np.pi) < co.CONTROLLER_MAX_ANGLE /
           float(constrain_lvl * 2)):
        # Add another condition to turn around until delta_theta1 is 0.5,
        # for instance (or pi/4)
        # Get new delta
        if rotation_direction < 0:
            delta_phi = -1
        else:
            delta_phi = +1
        ctrl.state["delta"] = move_wheels(ctrl.state["delta"],
                                          get_delta_from_phi(delta_phi, ctrl),
                                          5)
        # Run simulation step
        ctrl.simulation_step()
        # Add new state point
        ctrl.add_simulation_state()

    # Fix phi
    while (np.abs(ctrl.state["phi"] - np.pi) > co.CONTROLLER_MAX_ANGLE / 6.):
        # Get new delta
        if ctrl.state["phi"] < np.pi:
            delta_phi = -1
        else:
            delta_phi = +1
        ctrl.state["delta"] = move_wheels(ctrl.state["delta"],
                                          get_delta_from_phi(delta_phi, ctrl),
                                          2)
        # Run simulation step
        ctrl.simulation_step()
        # Add new state point
        ctrl.add_simulation_state()


def fix_large_phi(ctrl):
    i = 0
    while (np.abs(ctrl.state["phi"] - np.pi) > co.CONTROLLER_MAX_ANGLE / 3):
        # Get new delta
        if ctrl.state["phi"] < np.pi:
            ctrl.delta_phi = -1
        else:
            delta_phi = +1
        ctrl.state["delta"] = \
            move_wheels(ctrl.state["delta"],
                        get_delta_from_phi(delta_phi, ctrl), 10)

        # Run simulation step
        ctrl.simulation_step()

        # Add new state point
        ctrl.add_simulation_state()

        i += 1
        if i > 100:
            break


def get_delta_from_phi(delta_phi, ctrl):
    delta = arctan((-ctrl.car.L1 * (delta_phi * ctrl.trailer.L3 -
                    sin(ctrl.state["phi"]) * ctrl.dt * ctrl.V)) /
                   (ctrl.dt * ctrl.V * (ctrl.trailer.L3 + ctrl.car.L2 *
                    cos(ctrl.state["phi"]))))

    if ((delta > np.pi / 2.) and (delta < 3 * np.pi / 2)):
        delta -= np.pi

    return delta


def move_wheels(old_delta, delta, speed):
    gain = min(np.abs(delta - old_delta), co.CONTROLLER_DELTA_STEP)

    if delta > old_delta:
        delta = old_delta + gain * speed
    elif delta < old_delta:
        delta = old_delta - gain * speed
    else:
        return old_delta

    if np.abs(delta) < 1e-5:
        return 0

    if delta > 0:
        return min(delta, co.CONTROLLER_MAX_DELTA)
    else:
        return max(delta, -co.CONTROLLER_MAX_DELTA)


def phi_predictor(ctrl):
    new_phi = ctrl.state["phi"] + \
        (ctrl.V / ctrl.trailer.L3 * sin(ctrl.state["phi"]) +
         ctrl.V / ctrl.car.L1 * tan(ctrl.state["delta"]) *
         (1 + ctrl.car.L2 * cos(ctrl.state["phi"]) /
         ctrl.trailer.L3)) * ctrl.dt

    ctrl.state["phi"] = add_noise(new_phi)


def straight_ahead(ctrl):
    current_position = ctrl.state["current_position"]
    shift = np.array((ctrl.V * cos(ctrl.state["theta1"]) * ctrl.dt,
                      ctrl.V * sin(ctrl.state["theta1"]) * ctrl.dt))
    new_position = current_position + shift

    ctrl.state["current_position"] = new_position


def turn_around(ctrl, radius, center):
    # Determine sign according to delta
    if ctrl.state["delta"] > 0:
        sign = -1
    else:
        sign = +1

    # Get angle variables
    alpha = arctan(ctrl.car.L2 / radius)
    hitch_radius = np.hypot(radius, ctrl.car.L2)
    angularV = sign * ctrl.V / radius
    arc = angularV * ctrl.dt

    # Get new angle theta1
    ctrl.state["theta1"] -= arc
    ctrl.state["theta1"] = first_period_angle(ctrl.state["theta1"])

    # Upload current position according to theta1
    dx = hitch_radius * cos(ctrl.state["theta1"] + (np.pi / 2 + alpha) * sign)
    dy = hitch_radius * sin(ctrl.state["theta1"] + (np.pi / 2 + alpha) * sign)

    ctrl.state["current_position"] = (np.array((dx, dy)) + center)


def turning_center(ctrl, radius):
    if ctrl.state["delta"] > 0:
        sign = -1
    else:
        sign = +1

    dx = ctrl.car.L2 * cos(ctrl.state["theta1"]) + radius * \
        sin(ctrl.state["theta1"]) * sign
    dy = ctrl.car.L2 * sin(ctrl.state["theta1"]) - radius * \
        cos(ctrl.state["theta1"]) * sign

    return np.array((dx, dy))


def turning_radius(ctrl):
    if tan(ctrl.state["delta"]) == 0:
        return None
    else:
        return np.abs(ctrl.car.L1 / tan(ctrl.state["delta"]))
