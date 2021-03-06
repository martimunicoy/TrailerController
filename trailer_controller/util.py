#!/usr/bin/env python
# -*- coding: utf-8 -*-

# General imports
import numpy as np
import argparse

# Custom imports
import constants as co


# Functions
def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def first_period_angle(angle):
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


def cl_parser():
    parser = argparse.ArgumentParser(description=co.PROGRAM_DESCRIPTION)
    parser.add_argument('-f', '--file', metavar='FILE',
                        help='directory with a suitable input file')
    args = parser.parse_args()
    return args.file


def file_parser(input_file):
    if input_file is None:
        return {}
    file_args = {}
    with open(input_file, 'r') as f:
        for line in f:
            if line.startswith("#"):
                continue
            line.strip()
            if line.upper().startswith("POINT"):
                fields = line.split()
                file_args["POINTS"] = []
                for point in fields[1:]:
                    point = point.strip("(,),[,]")
                    point = np.array([float(i) for i in point.split(",")])
                    file_args["POINTS"].append(point)
            elif line.upper().startswith("SAVE"):
                fields = line.split()
                if len(fields) > 1:
                    file_args[fields[0].upper()] = fields[1]
                else:
                    file_args[fields[0].upper()] = None
            elif line.upper()[2:].startswith("AXIS"):
                fields = line.split()
                if len(fields) == 3:
                    file_args[fields[0].upper()] = (fields[1], fields[2])
            else:
                fields = line.split()
                if len(fields) == 2:
                    file_args[fields[0].upper()] = fields[1]
    return file_args


def print_license_header():
    print ("\n"
           "                        <Car-Trailer System Controller>\n  Copyrig"
           "ht (C) <2018> <Hernández, A., Massip, D., Municoy, M., Niemann, J."
           "H.>\n\n     Contact the authors at: aldomann.designs@gmail.com\n  "
           "                           david26694@gmail.com\n                 "
           "            mail@martimunicoy.com\n                             ja"
           "nhendrik.niemann@e-campus.uab.cat\n\n     This program is free sof"
           "tware: you can redistribute it and/or modify\n     it under the te"
           "rms of the GNU General Public License as published by\n     the Fr"
           "ee Software Foundation, either version 3 of the License, or\n     "
           "(at your option) any later version.\n\n     This program is distri"
           "buted in the hope that it will be useful,\n     but WITHOUT ANY WA"
           "RRANTY; without even the implied warranty of\n     MERCHANTABILITY"
           " or FITNESS FOR A PARTICULAR PURPOSE.  See the\n     GNU General P"
           "ublic License for more details.\n\n"
           )
