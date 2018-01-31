#!/usr/bin/env python
# -*- coding: utf-8 -*-


# Controller constants
CONTROLLER_THRESHOLD = 2
CONTROLLER_MAX_ANGLE = 0.17
CONTROLLER_DELTA_STEP = 0.05
CONTROLLER_MAX_DELTA = 0.7
CONTROLLER_MAX_NOISE = 0.0025

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
ANIMATION_LABEL = "Simulation of a car-trailer system"
ANIMATION_DPI = 200
ANIMATION_FPS = 30
ANIMATION_BITRATE = 3600
ANIMATION_DEFNAME = "TCsimulation.mp4"
