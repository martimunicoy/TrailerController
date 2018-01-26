#!/usr/bin/env python
# -*- coding: utf-8 -*-


# Controller constants
CONTROLLER_THRESHOLD = 1.5
CONTROLLER_MAX_ANGLE = 0.15
CONTROLLER_DELTA_STEP = 0.05
CONTROLLER_MAX_DELTA = 0.8

# Geometric constants
RAD_TO_DEG = 57.2957795131

# Other constants
PROGRAM_DESCRIPTION = "Run a simulation of a car-trailer system which follows"\
                      " a trajectory behind a controller"

# Model default parameters
L1 = 2.2
L2 = 1.2
L3 = 2
V = -2
DELTA = 0.2
THETA1 = 0
PHI = 3.1415926536

# Simulation default parameters
DT = 0.05
STEPS = 1000

# Animation default parameters
ANIMATION_REPEAT = False
ANIMATION_INTERVAL = 50
