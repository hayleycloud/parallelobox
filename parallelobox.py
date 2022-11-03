#!/bin/bash/env python3

import sys
import pymesh
import numpy as np
from threading import Thread

target_file = None

float_max =  1000000.0
float_min = -1000000.0

def handle_program_args():
    global target_file
    index = 1
    num_args = len(sys.argv)
    while index < num_args:
        arg = sys.argv[index]
        index += 1
        if arg.startswith('--') or arg.startswith('-'):
            # do stuff
            continue

        target_file = arg

def define_box(l, r, u, d, f, b):
    vertices = np.array([
        (l, u, f),  # 0: top left front
        (l, d, f),  # 1: bottom left front
        (r, d, f),  # 2: bottom right front
        (r, u, f),  # 3: top right front
        (r, u, b),  # 4: top right back
        (r, d, b),  # 5: bottom right back
        (l, d, b),  # 6: bottom left back
        (l, u, b)   # 7: top left back
        ])

    indices = np.array([
        # Front
        (0, 1, 2),
        (2, 3, 0),
        # Right
        (3, 2, 5),
        (5, 4, 3),
        # Back
        (4, 7, 6),
        (6, 5, 4),
        # Left
        (0, 7, 6),
        (6, 1, 0),
        # Top
        (0, 3, 4),
        (4, 7, 0),
        # Bottom
        (1, 6, 5),
        (5, 2, 1)
        ])

    return pymesh.form_mesh(vertices, indices)

def test_pymesh():
    cube = define_box(-20.0, 20.0, 15.0, -15.0, 10.0, -15.0)
    print('Sample Mesh Vertices: {}'.format(len(cube.vertices)))

    pymesh.save_mesh('/models/outcube.stl', cube)

def split_mesh_task(mesh, intersector, out_mesh):
    out_mesh = pymesh.boolean(mesh, intersector, "intersection")

def split_by_boxes(mesh):
    left_cube = pymesh.generate_box_mesh(
            np.array([float_min, float_min, float_min]),
            np.array([0.0, float_max, float_max]))
    right_cube = pymesh.generate_box_mesh(
            np.array([0.0, float_min, float_min]),
            np.array([float_max, float_max, float_max]))

    left_mesh = None
    right_mesh = None
    thread_left = Thread(
            target=split_mesh_task, 
            args=(mesh, left_cube, left_mesh))
    thread_right = Thread(
            target=split_mesh_task, 
            args=(mesh, right_cube, right_mesh))

    thread_left.start();
    thread_right.start();

    thread_left.join();
    thread_right.join();

    return (left_mesh, right_mesh)

handle_program_args()

mesh = pymesh.load_mesh(target_file)

print('Target Mesh: ', target_file)
print('Vertices: {}'.format(len(mesh.vertices)))

(left_mesh, right_mesh) = split_by_boxes(mesh)

pymesh.save_mesh('/models/left.stl', left_mesh)
pymesh.save_mesh('/models/right.stl', right_mesh)

