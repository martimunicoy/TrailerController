#!/usr/bin/env python
# -*- coding: utf-8 -*-


# General imports
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.transforms import Affine2D
from numpy import sin, cos


# Class definitions
class Animation:

    def __init__(self, car, trailer, positions):
        self.car = car
        self.trailer = trailer
        self.positions = positions

        self.car.initiate()
        self.trailer.initiate()

    def update(self, i):
        xy, theta1, theta2, delta = self.positions[i]
        self.car._update(xy, theta1, delta)
        self.trailer._update(xy, theta2)
        return []

    def show(self):
        plt.show()


class Car:

    def __init__(self, L1, L2, axis, fig, car_height=2.2, hitch_length=0.5,
                 front_length=0.8, color='red'):
        self.L1 = L1
        self.L2 = L2
        self.width = L1 + L2 - hitch_length + front_length
        self.height = car_height
        self.hitch_length = hitch_length
        self.front_length = front_length
        self.color = color
        self.axis = axis
        self.fig = fig

        # Display declarations
        self.extra_width = 0.6
        self.wheels_length = 0.7
        self.wheels_width = 0.22
        self.line_width = 0.2

    def initiate(self):
        # Car silouette
        self.silouette = Rectangle(xy=(0, 0), width=self.width,
                                   height=self.height, facecolor=self.color,
                                   edgecolor='k', linewidth=2)

        # Front wheels
        self.front_wheels_axis = Rectangle(xy=(0, 0), width=self.line_width,
                                           height=self.height +
                                           self.extra_width,
                                           facecolor='k')

        self.front_right_wheel = Rectangle(xy=(0, 0),
                                           width=self.wheels_length,
                                           height=self.wheels_width,
                                           facecolor='grey')

        self.front_left_wheel = Rectangle(xy=(0, 0),
                                          width=self.wheels_length,
                                          height=self.wheels_width,
                                          facecolor='grey')

        # Rear wheels
        self.rear_wheels_axis = Rectangle(xy=(0, 0), width=self.line_width,
                                          height=self.height +
                                          self.extra_width,
                                          facecolor='k')

        self.rear_right_wheel = Rectangle(xy=(0, 0),
                                          width=self.wheels_length,
                                          height=self.wheels_width,
                                          facecolor='grey')

        self.rear_left_wheel = Rectangle(xy=(0, 0),
                                         width=self.wheels_length,
                                         height=self.wheels_width,
                                         facecolor='grey')

        # Car axis
        self.car_axis = Rectangle(xy=(0, 0),
                                  width=self.L2 + self.L1 + self.front_length,
                                  height=self.line_width, facecolor='k')

        # Hitch
        self.hitch = Circle(xy=(0, 0), radius=0.2, facecolor='y')

        # Add all patches to axis
        self.axis.add_artist(self.silouette)
        self.axis.add_artist(self.front_wheels_axis)
        self.axis.add_artist(self.front_right_wheel)
        self.axis.add_artist(self.front_left_wheel)
        self.axis.add_artist(self.rear_wheels_axis)
        self.axis.add_artist(self.rear_right_wheel)
        self.axis.add_artist(self.rear_left_wheel)
        self.axis.add_artist(self.car_axis)
        self.axis.add_artist(self.hitch)

    def _update(self, xy, theta1, delta):
        # Coordinate decomposition
        x, y = xy
        sin_hitch = self.hitch_length * sin(theta1)
        cos_hitch = self.hitch_length * cos(theta1)
        sin_L2 = self.L2 * sin(theta1)
        cos_L2 = self.L2 * cos(theta1)
        sin_L1L2 = (self.L1 + self.L2) * sin(theta1)
        cos_L1L2 = (self.L1 + self.L2) * cos(theta1)
        sin_semi_height = self.height / 2. * sin(theta1)
        cos_semi_height = self.height / 2. * cos(theta1)
        sin_semi_extra_height = (self.height / 2. + self.extra_width / 2.) * \
            sin(theta1)
        cos_semi_extra_height = (self.height / 2. + self.extra_width / 2.) * \
            cos(theta1)
        sin_line = self.line_width / 2. * sin(theta1)
        cos_line = self.line_width / 2. * cos(theta1)
        sin_wheels_length = self.wheels_length / 2. * sin(theta1 + delta)
        cos_wheels_length = self.wheels_length / 2. * cos(theta1 + delta)
        sin_wheels_width = self.wheels_width / 2. * sin(theta1 + delta)
        cos_wheels_width = self.wheels_width / 2. * cos(theta1 + delta)
        sin_fixed_wheels_length = self.wheels_length / 2. * sin(theta1)
        cos_fixed_wheels_length = self.wheels_length / 2. * cos(theta1)
        sin_fixed_wheels_width = self.wheels_width / 2. * sin(theta1)
        cos_fixed_wheels_width = self.wheels_width / 2. * cos(theta1)

        # Car silouette
        xy = (x + cos_hitch + sin_semi_height,
              y + sin_hitch - cos_semi_height)
        self.silouette.set_xy(xy)
        self.silouette.set_transform(Affine2D().rotate_around(xy[0], xy[1],
                                                              theta1) +
                                     self.axis.transData)

        # Front wheels
        xy = (x + cos_L1L2 + sin_semi_extra_height - cos_line,
              y + sin_L1L2 - cos_semi_extra_height - sin_line)
        self.front_wheels_axis.set_xy(xy)
        self.front_wheels_axis.set_transform(Affine2D().rotate_around(xy[0],
                                                                      xy[1],
                                                                      theta1) +
                                             self.axis.transData)
        xy = (x + cos_L1L2 + sin_semi_extra_height - cos_wheels_length +
              sin_wheels_width,
              y + sin_L1L2 - cos_semi_extra_height - sin_wheels_length -
              cos_wheels_width)
        self.front_right_wheel.set_xy(xy)
        self.front_right_wheel.set_transform(Affine2D().rotate_around(xy[0],
                                                                      xy[1],
                                                                      theta1 +
                                                                      delta) +
                                             self.axis.transData)
        xy = (x + cos_L1L2 - sin_semi_extra_height - cos_wheels_length +
              sin_wheels_width,
              y + sin_L1L2 + cos_semi_extra_height - sin_wheels_length -
              cos_wheels_width)
        self.front_left_wheel.set_xy(xy)
        self.front_left_wheel.set_transform(Affine2D().rotate_around(xy[0],
                                                                     xy[1],
                                                                     theta1 +
                                                                     delta) +
                                            self.axis.transData)

        # Rear wheels
        xy = (x + cos_L2 + sin_semi_extra_height - cos_line,
              y + sin_L2 - cos_semi_extra_height - sin_line)
        self.rear_wheels_axis.set_xy(xy)
        self.rear_wheels_axis.set_transform(Affine2D().rotate_around(xy[0],
                                                                     xy[1],
                                                                     theta1) +
                                            self.axis.transData)
        xy = (x + cos_L2 + sin_semi_extra_height - cos_fixed_wheels_length +
              sin_fixed_wheels_width,
              y + sin_L2 - cos_semi_extra_height - sin_fixed_wheels_length -
              cos_fixed_wheels_width)
        self.rear_right_wheel.set_xy(xy)
        self.rear_right_wheel.set_transform(Affine2D().rotate_around(xy[0],
                                                                     xy[1],
                                                                     theta1) +
                                            self.axis.transData)

        xy = (x + cos_L2 - sin_semi_extra_height - cos_fixed_wheels_length +
              sin_fixed_wheels_width,
              y + sin_L2 + cos_semi_extra_height - sin_fixed_wheels_length -
              cos_fixed_wheels_width)
        self.rear_left_wheel.set_xy(xy)
        self.rear_left_wheel.set_transform(Affine2D().rotate_around(xy[0],
                                                                    xy[1],
                                                                    theta1) +
                                           self.axis.transData)

        # Car axis
        xy = (x + sin_line, y - cos_line)
        self.car_axis.set_xy(xy)
        self.car_axis.set_transform(Affine2D().rotate_around(xy[0], xy[1],
                                                             theta1) +
                                    self.axis.transData)

        # Hitch
        self.hitch.center = (x, y)

        # Return patches to animation
        return self.silouette, self.front_wheels_axis, \
            self.front_right_wheel, self.front_left_wheel, \
            self.rear_wheels_axis, self.rear_right_wheel, \
            self.rear_left_wheel, self.car_axis, self.hitch


class Trailer:

    def __init__(self, L3, axis, fig, trailer_height=2.2, hitch_length=0.5,
                 rear_length=0.5, color='orange'):
        self.L3 = L3
        self.width = L3 - hitch_length + rear_length
        self.height = trailer_height
        self.rear_length = rear_length
        self.hitch_length = hitch_length
        self.color = color
        self.axis = axis
        self.fig = fig

        # Display declarations
        self.extra_width = 0.6
        self.wheels_length = 0.7
        self.wheels_width = 0.22
        self.line_width = 0.2

    def initiate(self):
        # Trailer silouette
        self.silouette = Rectangle(xy=(0, 0), width=self.width,
                                   height=self.height,
                                   facecolor=self.color, edgecolor='k',
                                   linewidth=2)
        # Rear wheels
        self.wheels_axis = Rectangle(xy=(0, 0), width=self.line_width,
                                     height=self.height + self.extra_width,
                                     facecolor='k')

        self.right_wheel = Rectangle(xy=(0, 0), width=self.wheels_length,
                                     height=self.wheels_width,
                                     facecolor='grey')

        self.left_wheel = Rectangle(xy=(0, 0), width=self.wheels_length,
                                    height=self.wheels_width, facecolor='grey')

        # Trailer axis
        self.trailer_axis = Rectangle(xy=(0, 0),
                                      width=self.L3 + self.rear_length,
                                      height=self.line_width, facecolor='k')

        # Hitch
        self.hitch = Circle(xy=(0, 0), radius=0.2, facecolor='y')

        # Add all patches to axis
        self.axis.add_artist(self.silouette)
        self.axis.add_artist(self.wheels_axis)
        self.axis.add_artist(self.right_wheel)
        self.axis.add_artist(self.left_wheel)
        self.axis.add_artist(self.trailer_axis)
        self.axis.add_artist(self.hitch)

    def _update(self, xy, theta2):
        # Coordinate decomposition
        x, y = xy
        sin_hitch = self.hitch_length * sin(theta2)
        cos_hitch = self.hitch_length * cos(theta2)
        sin_L3 = self.L3 * sin(theta2)
        cos_L3 = self.L3 * cos(theta2)
        sin_semi_height = self.height / 2. * sin(theta2)
        cos_semi_height = self.height / 2. * cos(theta2)
        sin_semi_extra_height = (self.height / 2. + self.extra_width / 2.) * \
            sin(theta2)
        cos_semi_extra_height = (self.height / 2. + self.extra_width / 2.) * \
            cos(theta2)
        sin_line = self.line_width / 2. * sin(theta2)
        cos_line = self.line_width / 2. * cos(theta2)
        sin_fixed_wheels_length = self.wheels_length / 2. * sin(theta2)
        cos_fixed_wheels_length = self.wheels_length / 2. * cos(theta2)
        sin_fixed_wheels_width = self.wheels_width / 2. * sin(theta2)
        cos_fixed_wheels_width = self.wheels_width / 2. * cos(theta2)

        # Trailer silouette
        xy = (x + cos_hitch + sin_semi_height,
              y + sin_hitch - cos_semi_height)
        self.silouette.set_xy(xy)
        self.silouette.set_transform(Affine2D().rotate_around(xy[0], xy[1],
                                                              theta2) +
                                     self.axis.transData)

        # Trailer wheels
        xy = (x + cos_L3 + sin_semi_extra_height - cos_line,
              y + sin_L3 - cos_semi_extra_height - sin_line)
        self.wheels_axis.set_xy(xy)
        self.wheels_axis.set_transform(Affine2D().rotate_around(xy[0], xy[1],
                                                                theta2) +
                                       self.axis.transData)

        xy = (x + cos_L3 + sin_semi_extra_height - cos_fixed_wheels_length +
              sin_fixed_wheels_width,
              y + sin_L3 - cos_semi_extra_height - sin_fixed_wheels_length -
              cos_fixed_wheels_width)
        self.right_wheel.set_xy(xy)
        self.right_wheel.set_transform(Affine2D().rotate_around(xy[0], xy[1],
                                                                theta2) +
                                       self.axis.transData)

        xy = (x + cos_L3 - sin_semi_extra_height - cos_fixed_wheels_length +
              sin_fixed_wheels_width,
              y + sin_L3 + cos_semi_extra_height - sin_fixed_wheels_length -
              cos_fixed_wheels_width)
        self.left_wheel.set_xy(xy)
        self.left_wheel.set_transform(Affine2D().rotate_around(xy[0], xy[1],
                                                               theta2) +
                                      self.axis.transData)

        # Trailer axis
        xy = (x + sin_line, y - cos_line)
        self.trailer_axis.set_xy(xy)
        self.trailer_axis.set_transform(Affine2D().rotate_around(xy[0], xy[1],
                                                                 theta2) +
                                        self.axis.transData)

        # Hitch
        self.hitch.center = (x, y)

        # Return patches to animation
        return self.silouette, self.wheels_axis, self.right_wheel, \
            self.left_wheel, self.trailer_axis, self.hitch
