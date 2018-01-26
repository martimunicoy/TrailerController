import sys
from matplotlib import pyplot as plt
from numpy import sin, tan, cos, arctan
from matplotlib.animation import FuncAnimation
from animation import Car, Trailer, Animation

import numpy as np

# Constants
CONTROLLER_THRESHOLD = 0.5
CONTROLLER_MAX_ANGLE = 0.15
CONTROLLER_DELTA_STEP = 0.08
CONTROLLER_MAX_DELTA = 0.7

# Initial variables
initial_phi = 0


class Trajectory():

    def __init__(self, type='linear', in_position=np.array((15, 15)),
                 in_theta1=0, in_phi=np.pi, fi_position=np.array((5, 20)),
                 fi_theta2=np.pi / 2, num_points=1000):
        # Defining initial state
        self.in_position = in_position
        self.in_theta1 = in_theta1
        self.in_phi = in_phi

        # Defining final state
        self.fi_position = fi_position
        self.fi_theta2 = fi_theta2

        # Calculating trajectory
        self.type = type
        self.num_points = num_points
        if self.type == 'linear':
            self.points = self.linear()
        elif self.type == 'circular':
            self.points = self.circular()
        else:
            sys.exit("Error: invalid trajectory type")

    def linear(self):
        # Initiate points list
        points_list = []

        # Increment of the position per point
        delta_position = (self.fi_position - self.in_position) / \
            float(self.num_points)

        # Add initial position
        points_list.append(self.in_position)

        # Calculate iteratively next trajectory points
        for point in xrange(self.num_points - 1):
            previous_point = points_list[-1]
            new_point = previous_point + delta_position
            points_list.append(new_point)

        # Return final points list
        return points_list

    def circular(self):
        # Initiate points list
        points_list = []

        # Increment of the position per point
        delta_position = (self.fi_position - self.in_position) / \
            float(self.num_points)

        # Calculate iteratively next trajectory points
        new_point = self.in_position

        # Linear part
        if (delta_position[0] != delta_position[1]):
            max_index = np.abs(delta_position[0]) < np.abs(delta_position[1])
            mx = int(max_index)
            mn = int(not max_index)
            delta_position[mn] = 0

            while (np.abs(new_point[mx] - self.fi_position[mx]) >
                   np.abs(new_point[mn] - self.fi_position[mn])):
                points_list.append(new_point)
                previous_point = points_list[-1]
                new_point = previous_point + delta_position
        else:
            mx = 0
            mn = 1
            delta_position[mn] = 0
            points_list.append(new_point)

        # Circular part
        diff = self.fi_position - points_list[-1]
        print "diff: ", diff
        radius2 = pow(np.abs(diff[0]), 2)
        center = points_list[-1] + diff[mn] * np.array((mx, mn))
        if diff[mn] > 0:
            mn_sign = -1
        else:
            mn_sign = +1
        for point in xrange(self.num_points - len(points_list)):
            previous_point = points_list[-1]
            c1 = previous_point[mx] * np.array((mn, mx)) + delta_position
            c2 = (mn_sign * np.sqrt(radius2 - pow(c1[mx] - center[mx], 2)) +
                  center[mn]) * np.array((mx, mn))
            new_point = c1 + c2
            points_list.append(new_point)

        # Return final points list
        return points_list

    def draw(self, axis):
        for point in self.points:
            axis.plot(point[0], point[1], alpha=0.5, color='blue',
                      marker=".", markersize=2)


class Controller():

    def __init__(self, car, trailer, dt=0.01, def_steps=1000, def_V=-10):
        self.car = car
        self.trailer = trailer
        self.dt = dt
        self.def_steps = def_steps
        self.def_V = def_V
        self.points = []

        # Trajectory attributes
        self.trajectory = None
        self.trajectory_points = []

    def add_traj(self, trajectory, reduce_traj_points_to=10):
        # Add as many trajectory points as required
        for i in xrange(trajectory.num_points):
            if (i % int(trajectory.num_points /
               (reduce_traj_points_to + 1)) == 0):
                self.trajectory_points.append(trajectory.points[i])

        # Always last trajectory point must be added
        self.trajectory_points.append(trajectory.points[-1])

        # Save current trajectory as a path attribute
        self.trajectory = trajectory

    def get_closest_point(self, prediction):
        closest_point = self.trajectory_points[0]
        best_dist = np.linalg.norm(closest_point - prediction)

        for point in self.trajectory_points:
            dist = np.linalg.norm(point - prediction)
            if best_dist > dist:
                closest_point = point
                best_dist = dist

        print closest_point
        return closest_point

    def draw(self, axis):
        for point in self.trajectory_points:
            axis.plot(point[0], point[1], color='red', marker=".",
                      markersize=3)

    def add_simulation_state(self, state):
        self.points.append((state["current_position"], state["theta1"],
                            state["theta2"], state["delta"]))

    def fix_large_phi(self, state, V):
        i = 0
        while (np.abs(state["phi"] - np.pi) > CONTROLLER_MAX_ANGLE / 2):
            # Get new delta
            if state["phi"] < np.pi:
                delta_phi = -1
            else:
                delta_phi = +1
            state["delta"] = \
                self.move_wheels(state["delta"],
                                 self.get_delta_from_phi(delta_phi, state, V),
                                 10)

            # Run simulation step
            self.simulation_step(state, V)

            # Add new state point
            self.add_simulation_state(state)

            i += 1
            if i > 100:
                break

    def turning_radius(self, delta):
        if tan(delta) == 0:
            return None
        else:
            return np.abs(self.car.L1 / tan(delta))

    def turning_center(self, radius, state):
        if state["delta"] > 0:
            sign = -1
        else:
            sign = +1

        dx = self.car.L2 * cos(state["theta1"]) + radius * \
            sin(state["theta1"]) * sign
        dy = self.car.L2 * sin(state["theta1"]) - radius * \
            cos(state["theta1"]) * sign

        return np.array((dx, dy))

    def turn_around(self, radius, center, state, V):
        # Determine sign according to delta
        if state["delta"] > 0:
            sign = -1
        else:
            sign = +1

        # Get angle variables
        alpha = arctan(self.car.L2 / radius)
        hitch_radius = np.hypot(radius, self.car.L2)
        angularV = sign * V / radius
        arc = angularV * self.dt

        # Get new angle theta1
        state["theta1"] -= arc
        state["theta1"] = abs_1st_period_angle(state["theta1"])

        # Upload current position according to theta1
        dx = hitch_radius * cos(state["theta1"] + (np.pi / 2 + alpha) * sign)
        dy = hitch_radius * sin(state["theta1"] + (np.pi / 2 + alpha) * sign)

        state["current_position"] = (np.array((dx, dy)) + center)

    def staight_ahead(self, state, V):
        state["current_position"] = state["current_position"] + \
            np.array((V * cos(state["theta1"]) * self.dt, V *
                      sin(state["theta1"]) * self.dt))

    def get_delta_from_phi(self, delta_phi, state, V):
        delta = np.arctan((-self.car.L1 * (delta_phi * self.trailer.L3 -
                          sin(state["phi"]) * self.dt * V)) /
                          (self.dt * V * (self.trailer.L3 + self.car.L2 *
                                          cos(state["phi"]))))

        if ((delta > np.pi / 2.) and (delta < 3 * np.pi / 2)):
            delta -= np.pi

        return delta

    def move_wheels(self, old_delta, delta, speed):
        gain = min(np.abs(delta), CONTROLLER_DELTA_STEP)

        if delta > old_delta:
            delta = old_delta + gain * speed
        elif delta < old_delta:
            delta = old_delta - gain * speed
        else:
            return old_delta

        if np.abs(delta) < 1e-5:
            return 0

        if delta > 0:
            return min(delta, CONTROLLER_MAX_DELTA)
        else:
            return max(delta, -CONTROLLER_MAX_DELTA)

    def simulation_step(self, state, V):
        radius = self.turning_radius(state["delta"])

        if radius is not None:
            center = state["current_position"] + self.turning_center(radius,
                                                                     state)
            self.turn_around(radius, center, state, V)

        else:
            self.staight_ahead(state, V)

        self.phi_predictor(state, V)
        # Do not touch this! Phi is predicted by the model in a way that needs
        # to be substracted from theta1 in order to get theta2
        state["theta2"] = state["theta1"] - state["phi"]

    def phi_predictor(self, state, V):
        new_phi = state["phi"] + (V / self.trailer.L3 * sin(state["phi"]) +
                                  V / self.car.L1 * tan(state["delta"]) *
                                  (1 + self.car.L2 * cos(state["phi"]) /
                                  self.trailer.L3)) * self.dt
        state["phi"] = new_phi

    def face_trailer_to_dir(self, state, V, goal_angle):
        delta_theta1 = abs_1st_period_angle(state["theta1"] - goal_angle +
                                            np.pi)

        # Face trailer
        while (np.abs(state["phi"] - np.pi) < CONTROLLER_MAX_ANGLE / 2):
            # Get new delta
            if delta_theta1 < np.pi:
                delta_phi = -1
            else:
                delta_phi = +1
            state["delta"] = \
                self.move_wheels(state["delta"],
                                 self.get_delta_from_phi(delta_phi, state,
                                                         V), 2)
            # Run simulation step
            self.simulation_step(state, V)
            # Add new state point
            self.add_simulation_state(state)

        # Fix phi
        while (np.abs(state["phi"] - np.pi) > CONTROLLER_MAX_ANGLE / 6):
            # Get new delta
            if state["phi"] < np.pi:
                delta_phi = -1
            else:
                delta_phi = +1
            state["delta"] = \
                self.move_wheels(state["delta"],
                                 self.get_delta_from_phi(delta_phi, state,
                                                         V), 2)
            # Run simulation step
            self.simulation_step(state, V)
            # Add new state point
            self.add_simulation_state(state)

    def go_straight(self, state, V):
        while (np.abs(state["phi"] - np.pi) < CONTROLLER_MAX_ANGLE / 4):
            # Get new delta
            if state["phi"] < np.pi:
                delta_phi = +1
            else:
                delta_phi = -1
            state["delta"] = \
                self.move_wheels(state["delta"],
                                 self.get_delta_from_phi(delta_phi, state,
                                                         V), 1)

            # Run simulation step
            self.simulation_step(state, V)

            # Add new state point
            self.add_simulation_state(state)

    def simulate(self, V=None, max_steps=None):
        if ((len(self.trajectory_points) == 0) or (self.trajectory is None)):
            sys.exit("Error: add trajectory with add_traj() before calling "
                     "simulate() function")

        # Set velocity V
        if V is None:
            V = self.def_V

        # Set maximum number of steps max_steps
        if max_steps is None:
            max_steps = self.def_steps

        # If initial position is not a numpy array, convert it
        elif type(self.position) is list or type(self.position) is tuple:
            self.position = np.array(self.position)

        # Define initial state of the system
        state = {}
        state["theta1"] = self.trajectory.in_theta1
        state["phi"] = self.trajectory.in_phi
        state["theta2"] = state["theta1"] + state["phi"]
        state["current_position"] = self.trajectory_points.pop(0)
        state["delta"] = 0

        # Add first state to list
        self.add_simulation_state(state)

        # Initialize variables for the loop
        ref_axis = np.array([1, 0])
        print self.trajectory_points
        checked_points = 0
        found = False
        goal_point = self.trajectory_points.pop(0)

        # In case of an initial large phi angle, fix it
        if np.abs(state["phi"] - np.pi) > CONTROLLER_MAX_ANGLE:
            self.fix_large_phi(state, V)

        # Start simulation
        for i in xrange(max_steps):
            if i % 100 == 0:
                print i
            # Calculate variables according to current position and goal point
            goal_dir = goal_point - state["current_position"]

            # Calculate goal angle and delta phi
            goal_angle = angle_between(ref_axis, goal_dir)

            delta_theta1 = state["theta1"] - goal_angle - np.pi
            print "goal_point", goal_point, "current_position", state["current_position"], "goal_dir", goal_dir, "goal_angle", goal_angle, "theta1", state["theta1"], "delta_theta1", delta_theta1

            self.face_trailer_to_dir(state, V, goal_angle)

            # Check if we have arrived to the current goal point
            for point in self.points[checked_points:]:
                dist = np.linalg.norm(goal_point - point[0])
                if dist < CONTROLLER_THRESHOLD:
                    # If this is the case and no points are left, finish loop
                    if len(self.trajectory_points) == 0:
                        print "Done!"
                        goal_point = None
                        found = True
                    else:
                        # If there are remaining points, go to the next one
                        goal_point = self.trajectory_points.pop(0)
                        print goal_point
                    break
                checked_points += 1

            if found:
                print len(self.points), checked_points
                for i in range(checked_points, len(self.points)):
                    del(self.points[-1])
                print len(self.points)

                if goal_point is None:
                    break
                else:
                    found = False

        # Update current position of the system
        self.position = state["current_position"]

        # End of while loop, remove trajectory from path object
        self.trajectory = None
        self.trajectory_points = []


def abs_1st_period_angle(angle):
    summand = np.pi * 2

    if angle > 0:
        while angle >= summand:
            angle -= summand
    else:
        while angle < 0:
            angle += summand

    return angle


def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


if __name__ == "__main__":
    # Model parameters
    L1 = 2.2
    L2 = 1.2
    L3 = 2
    V = -2
    delta = 0.2

    # Simulation parameters
    dt = 0.05
    steps = 1000

    fig, axis = plt.subplots(1)
    plt.gca().set_aspect('equal', adjustable='box')
    axis.set_xlim(0, 30)
    axis.set_ylim(0, 30)
    my_car = Car(L1, L2, axis, fig)
    my_trailer = Trailer(L3, axis, fig)

    """
    my_path = Path(my_car, my_trailer, in_position=(15, 15))

    for i in xrange(20):
        my_path.add_traj(V, 0.5, 20)
        my_path.add_traj(V, -0.5, 20)

    """
    # my_path = Park(my_car, my_trailer, in_position=(15, 15), fi_position=(5, 20))
    # my_path.controller(V, 1000)

    # Create trajectory points
    point1 = Trajectory(type='linear', num_points=100,
                               in_position=np.array((25, 5)),
                               fi_position=np.array((5, 10)))
    point1.draw(axis)
    point2 = Trajectory(type='linear', num_points=100,
                               in_position=np.array((25, 5)),
                               fi_position=np.array((5, 10)), in_theta1=np.pi)
    point2.draw(axis)

    point3 = Trajectory(type='linear', num_points=100,
                               in_position=np.array((5, 10)),
                               fi_position=np.array((25, 15)))
    point3.draw(axis)
    # Simulate path
    my_controller = Controller(my_car, my_trailer, dt=0.05, def_steps=100,
                               def_V=V)
    #my_controller.add_traj(point1, reduce_traj_points_to=0)
    my_controller.add_traj(point1, reduce_traj_points_to=0)
    my_controller.draw(axis)
    my_controller.simulate()

    # plt.show()
    """
    animation = Animation(my_car, my_trailer, my_path.traj)
    simulation = FuncAnimation(fig, animation.update, interval=10,
                               frames=len(my_path.traj),
                               repeat=True, blit=False)

    """
    animation = Animation(my_car, my_trailer, my_controller.points)
    simulation = FuncAnimation(fig, animation.update, interval=100,
                               frames=len(my_controller.points),
                               repeat=True, blit=True)

    animation.show()

