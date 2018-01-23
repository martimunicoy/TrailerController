import sys
from matplotlib import pyplot as plt
from numpy import sin, tan, cos, arctan
from matplotlib.animation import FuncAnimation
from animation import Car, Trailer, Animation

import numpy as np

# Constants
CONTROLLER_THRESHOLD = 0.5
CONTROLLER_MAX_ANGLE = 0.1

# Initial variables
initial_phi = 0


def phi_predictor(phi, V, L1, L2, L3, delta, dt):
    new_phi = phi + (V / L3 * sin(phi) + V / L1 * tan(delta) *
                     (1 + L2 * cos(phi) / L3)) * dt
    return new_phi


class oldPath():

    def __init__(self, car, trailer, in_position=(5, 5), in_theta1=0, phi=None,
                 dt=0.01):
        self.car = car
        self.trailer = trailer
        self.position = in_position
        self.theta1 = in_theta1
        if phi is None:
            self.phi = self.theta1 + np.pi
        else:
            self.phi = phi
        self.theta2 = self.phi + self.theta1
        self.trailer_position = (self.position[0] + self.trailer.L3 *
                                 cos(self.theta2),
                                 self.position[1] + self.trailer.L3 *
                                 sin(self.theta2))
        self.delta = 0
        self.dt = dt
        self.traj = []
        self.update_traj()

    def update_traj(self):
        self.traj.append((self.position, self.theta1, self.theta2,
                         self.delta))

    def add_traj(self, V, delta, steps):
        self.delta = delta
        for i in xrange(steps):
            angle = self.theta1 + delta
            shift = (V * cos(angle) * self.dt, V * sin(angle) * self.dt)
            self.position = tuple(sum(x) for x in zip(self.traj[-1][0], shift))
            self.theta1 += V / self.car.L1 * tan(delta) * self.dt
            self.phi = phi_predictor(self.phi, V, self.car.L1, self.car.L2,
                                     self.trailer.L3, delta, self.dt)
            self.theta2 = self.theta1 - self.phi
            """
            self.trailer_position = (self.position[0] + self.trailer.L3 *
                                     cos(self.theta2),
                                     self.position[1] + self.trailer.L3 *
                                     sin(self.theta2))
            """
            self.update_traj()


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
    dx = car.L2 * cos(theta1) + radius * sin(theta1) * sign
    dy = car.L2 * sin(theta1) - radius * cos(theta1) * sign
    return np.array((dx, dy))


def turn_around(car, radius, theta1, center, V, delta, dt):
    # Determine sign according to delta
    if delta > 0:
        sign = -1
    else:
        sign = +1

    # Get angle variables
    alpha = arctan(car.L2 / radius)
    hitch_radius = np.hypot(radius, car.L2)
    angularV = sign * V / radius
    arc = angularV * dt

    # Get new angle theta1
    theta1 -= arc

    # Get shift according to theta1
    dx = hitch_radius * cos(theta1 + (np.pi / 2 + alpha) * sign)
    dy = hitch_radius * sin(theta1 + (np.pi / 2 + alpha) * sign)
    shift = (np.array((dx, dy)) + center)

    return shift, theta1


class Path():

    def __init__(self, car, trailer, in_position=None, dt=0.01, def_steps=1000,
                 def_V=-10):
        self.car = car
        self.trailer = trailer
        self.position = in_position
        self.dt = dt
        self.def_steps = def_steps
        self.def_V = def_V
        self.sim_points = []

        # Trajectory attributes
        self.trajectory = None
        self.trajectory_points = []

    def update_traj(self):
        self.sim_points.append((self.position, self.theta1, self.theta2,
                               self.delta))

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

    def draw(self, axis):
        for point in self.trajectory_points:
            axis.plot(point[0], point[1], color='red', marker=".",
                      markersize=3)

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

        # If none initial position for the car-trailer system was given, set
        # the initial trajectory point as the initial one
        if self.position is None:
            self.position = self.trajectory_points[0]

        # If initial position is not a numpy array, convert it
        elif type(self.position) is list or type(self.position) is tuple:
            self.position = np.array(self.position)

        # Initial state
        current_position = self.position
        theta1 = self.trajectory.in_theta1
        phi = self.trajectory.in_phi
        theta2 = abs_1st_period_angle(theta1 - phi)
        self.sim_points.append((current_position, theta1, theta2, 0))

        # Initialize variables for the loop
        ref_axis = np.array([1, 0])
        step = 0
        goal_point = self.trajectory_points.pop(0)
        print goal_point

        while (step < max_steps):
            """
            delta = 0.2
            radius = turning_radius(self.car, delta)
            center = current_position + turning_center(self.car, radius, theta1, delta)
            axis.plot(center[0], center[1], color='red', marker=".", markersize=3)
            current_position, theta1 = turn_around(self.car, radius, theta1,
                                                   center, V, delta, 0.1)
            print "position", current_position
            phi = phi_predictor(phi, V, self.car.L1, self.car.L2,
                                self.trailer.L3, delta, self.dt)
            theta2 = abs_1st_period_angle(theta1 - phi)
            self.sim_points.append((current_position, theta1, theta2, delta))
            """

            # Calculate variables according to current position and goal point
            goal_dir = goal_point - current_position
            dist = np.linalg.norm(goal_dir)

            # Check if we have arrived to the current goal point
            if dist < CONTROLLER_THRESHOLD:
                # If this is the case and no points are left, finish loop
                if len(self.trajectory_points) == 0:
                    break
                # If there are remaining points, go to the next one
                goal_point = self.trajectory_points.pop(0)
                print goal_point
                continue

            # Calculate goal angle and delta phi
            goal_angle = angle_between(ref_axis, goal_dir)
            delta_phi = goal_angle - theta2
            """
            """
            #Improve!
            while (np.abs(phi - np.pi) > CONTROLLER_MAX_ANGLE):
                if phi < np.pi:
                    delta_phi = 0.05
                else:
                    delta_phi = -0.05

                delta = get_delta_from_phi(delta_phi, phi, V,
                                           self.car.L1, self.car.L2,
                                           self.trailer.L3, self.dt)
                # Update state
                radius = turning_radius(self.car, delta)
                center = current_position + turning_center(self.car, radius, theta1, delta)
                current_position, theta1 = turn_around(self.car, radius, theta1,
                                                       center, V, delta, 0.1)
                phi = phi_predictor(phi, V, self.car.L1, self.car.L2,
                                    self.trailer.L3, delta, self.dt)
                theta2 = abs_1st_period_angle(theta1 - phi)
                self.sim_points.append((current_position, theta1, theta2, delta))

            delta = get_delta_from_phi(delta_phi, phi, V,
                                       self.car.L1, self.car.L2,
                                       self.trailer.L3, self.dt)
            """



            delta = get_delta_from_phi2(0.01, phi, V,
                                       self.car.L1, self.car.L2,
                                       self.trailer.L3, self.dt)

            """
            # Update state
            radius = turning_radius(self.car, delta)
            center = current_position + turning_center(self.car, radius, theta1, delta)
            current_position, theta1 = turn_around(self.car, radius, theta1,
                                                   center, V, delta, 0.1)
            phi = phi_predictor(phi, V, self.car.L1, self.car.L2,
                                self.trailer.L3, delta, self.dt)
            theta2 = abs_1st_period_angle(theta1 - phi)
            self.sim_points.append((current_position, theta1, theta2, delta))


            # Sum up one step
            step += 1

        # Update current position of the system
        self.position = current_position

        """
        while (len(self.trajectory_points) > 1) and (step < max_steps):
            #print current_position, len(self.trajectory_points)
            # Calculate corrections using the goal trajectory
            dx = V * self.dt * cos(theta1)
            dy = V * self.dt * sin(theta1)
            prediction = current_position + np.array((dx, dy))
            point = self.get_closest_point(prediction)

            if np.array_equal(point, current_position):
                print "Warning: Path.add_traj may need a trajectory with " + \
                    "more points"
                index = self.trajectory_points.index(point)
                point = self.trajectory_points[index + 1]

            vec = point - current_position
            #print point, current_position, vec, vec[1] / vec[0]
            goal_theta2 = angle_between(ref_axis, vec)
            theta2_correction = goal_theta2 - theta2
            #print goal_theta2, theta2, theta2_correction
            phi_correction = theta2_correction

            # Predict delta
            delta = get_delta_from_phi(phi_correction, phi, V,
                                       self.car.L1, self.car.L2,
                                       self.trailer.L3, self.dt)
            #print phi_correction, delta

            # Update state
            angle = theta1 + delta
            shift = np.array((V * cos(angle) * self.dt,
                             V * sin(angle) * self.dt))
            current_position = current_position + shift
            theta1 += V / self.car.L1 * tan(delta) * self.dt
            phi = phi_predictor(phi, V, self.car.L1, self.car.L2,
                                self.trailer.L3, delta, self.dt)
            theta2 = abs_1st_period_angle(theta1 - phi)
            self.sim_points.append((current_position, theta1, theta2, delta))

            # Update trajectory
            #self.update_trajectory(current_position)

            # Sum up one step
            step += 1
        """

        # End of while loop, remove trajectory from path object
        self.trajectory = None
        self.trajectory_points = []

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


def get_delta_from_phi2(delta_phi, phi, V, L1, L2, L3, dt):
    delta = np.arctan((((delta_phi / dt) + V / L3 * sin(phi)) *
                      L1 / V) / (-1 - L2 * cos(phi) / L3))

    """
    if (delta > np.pi / 2.) and (delta < 3 * np.pi / 2):
        delta -= np.pi
    """

    while delta > 0.5:
        delta -= 0.05
    while delta < -1.2:
        delta += 0.5

    return delta


def get_delta_from_phi(delta_phi, phi, V, L1, L2, L3, dt):
    delta = np.arctan((L1 * (delta_phi * L3 - sin(phi) * dt * V)) / (
                      dt * V * (L3 + L2 * cos(phi))))

    """
    if (delta > np.pi / 2.) and (delta < 3 * np.pi / 2):
        delta -= np.pi
    """

    while delta > 0.5:
        delta -= 0.05
    while delta < -1.2:
        delta += 0.5

    return delta


if __name__ == "__main__":
    # Model parameters
    L1 = 2.2
    L2 = 1.2
    L3 = 2
    V = -2
    delta = 0.5

    # Simulation parameters
    dt = 0.1
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
    my_trajectory = Trajectory(type='linear', num_points=100,
                               in_position=np.array((20, 10)))
    my_trajectory.draw(axis)

    # Simulate path
    my_path = Path(my_car, my_trailer, dt=0.025, def_steps=100, def_V=V)
    my_path.add_traj(my_trajectory)
    my_path.draw(axis)
    my_path.simulate()

    # plt.show()
    """
    animation = Animation(my_car, my_trailer, my_path.traj)
    simulation = FuncAnimation(fig, animation.update, interval=10,
                               frames=len(my_path.traj),
                               repeat=True, blit=False)

    """
    animation = Animation(my_car, my_trailer, my_path.sim_points)
    simulation = FuncAnimation(fig, animation.update, interval=100,
                               frames=len(my_path.sim_points),
                               repeat=True, blit=True)

    animation.show()

