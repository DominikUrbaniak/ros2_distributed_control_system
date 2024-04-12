#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os
import numpy as np

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

#set the coordinates for 9 places, cell #1 is ? (robot arm perspective)
def gen_3x3_grid_locations(first_cell_coords, x_step, y_step):
    grid_nx = 3
    grid_ny = 3
    grid_locations = np.zeros((grid_nx*grid_ny,3))
    counter = 0
    for i in range(0,grid_nx):
        for j in range(0,grid_ny):
            grid_locations[counter,:] = [first_cell_coords[0]+(i)*x_step,first_cell_coords[1]+(j)*y_step,0.025]
            counter = counter + 1
    return grid_locations

def gen_object_list(number_of_objects):
    objects = []
    colors = ['grey','violet','indigo','blue','green','yellow','orange','red']
    coords_first_cell = [-0.2,-0.7] #[-0.2,0.5]
    x_step = 0.1
    y_step = 0.15
    grid_positions = gen_3x3_grid_locations(coords_first_cell,x_step,y_step)

    for i in range(number_of_objects):
        object_name = 'cube_tag'+str(i)+'_'+colors[i]
        if (i==4):
            x_pos = grid_positions[i-1,0]
            y_pos = grid_positions[i-1,1]
            z_pos = grid_positions[i-1,2]+0.05
        else:
            x_pos = grid_positions[i,0]
            y_pos = grid_positions[i,1]
            z_pos = grid_positions[i,2]

        objects.append({'name': object_name, 'x_pose': x_pos, 'y_pose': y_pos, 'z_pose': z_pos, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0})

    #add base plate for the translation experiment
    objects.append({'name': 'translation_base', 'x_pose': -0.1+0.001, 'y_pose': 0.65-0.011, 'z_pose': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 1.0, 'qw': 0.0})

    return objects

def generate_launch_description():
    pkg_main = get_package_share_directory('main_pkg')

    # Names and poses of the objects
    objects = gen_object_list(8)

    # Create the list of spawn objects commands
    spawn_objects_cmds = []
    for object in objects:
        urdf = os.path.join(get_package_share_directory('main_pkg'), 'urdf/cubes/', object['name']+'.urdf')
        assert os.path.exists(urdf), "The " + object['name']+".urdf-file doesnt exist in "+str(urdf)
        spawn_objects_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_main, 'launch',
                                                           'spawn_cube_launch.py')),
                launch_arguments={
                    'object_urdf': urdf,
                    'x': TextSubstitution(text=str(object['x_pose'])),
                    'y': TextSubstitution(text=str(object['y_pose'])),
                    'z': TextSubstitution(text=str(object['z_pose'])),
                    'qx': TextSubstitution(text=str(object['qx'])),
                    'qy': TextSubstitution(text=str(object['qy'])),
                    'qz': TextSubstitution(text=str(object['qz'])),
                    'qw': TextSubstitution(text=str(object['qw'])),
                    'object_name': object['name'],
                    'object_namespace': object['name']
                }.items()))

    # Create the launch description and populate
    ld = LaunchDescription()

    for spawn_object_cmd in spawn_objects_cmds:
        ld.add_action(spawn_object_cmd)

    return ld
