#!/usr/bin/env python3
import math
import sys
import csv

from scipy.interpolate import CubicSpline
import numpy
from stl import mesh
import matplotlib.pyplot as plt
from operator import itemgetter
from typing import Tuple


def load_wall(file):
    points_with_ids = []
    with open(file, newline='') as csv_file:
        csv_reader = csv.DictReader(csv_file, delimiter=',')
        for row in csv_reader:
            id = row['Name']
            long = row['Longitude']
            lat = row['Latitude']
            points_with_ids.append([int(id), [float(long), float(lat), 0.0]])
    # https://stackoverflow.com/questions/17243620/operator-itemgetter-or-lambda
    points_with_ids.sort(key=itemgetter(0))
    points = numpy.empty((len(points_with_ids), 3), dtype=numpy.float64)
    for (i, point) in enumerate(points_with_ids):
        points[i] = point[1]
    return points


def trajectory_interpolate(points, int_size=20):
    _points = numpy.vstack((points, points[0, :]))
    distance = numpy.cumsum(numpy.sqrt(
        numpy.sum(numpy.diff(_points, axis=0) ** 2, axis=1)
    ))
    distance = numpy.insert(distance, 0, 0) / distance[-1]
    alpha = numpy.linspace(0, 1, int_size, endpoint=False)
    points_interpolated = CubicSpline(
        distance, _points, axis=0, bc_type='periodic'
    )(alpha)
    return points_interpolated


def gnss_to_cartesian_test(point, offset, radius_north, radius_east):
    point_centered = point - offset

    x = math.radians(point_centered[0]) * radius_east
    y = math.radians(point_centered[1]) * radius_north
    z = 0.0

    return [x, y, z]


def gnss_to_cartesian(points, offset, radius_north, radius_east):
    points_ = points - offset
    # degrees to radians
    points_ = numpy.radians(points_)
    # apply forward Equirectangular projection
    # https://en.wikipedia.org/wiki/Equirectangular_projection#Forward
    points_ = points_ * \
        numpy.repeat([[radius_east, radius_north, 1.0]], len(points), axis=0)
    return points_


def get_2d_in_3d_rotation_transform(angle):
    '''Apply with numpy.dot(left_wall, R)'''
    theta = numpy.radians(angle)
    c, s = numpy.cos(theta), numpy.sin(theta)
    R = numpy.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))
    return R


def convert_points(
    points_gps,
    base_point_gps,
    radius_north,
    radius_east,
    num_points=200,
    visualize=True,
):
    points_xyz = gnss_to_cartesian(
        points_gps,
        base_point_gps,
        radius_north,
        radius_east,
    )

    if visualize:
        plt.plot(
            points_xyz[:, 0],  # x
            points_xyz[:, 1],  # y
        )
        # add point numbers
        for (i, p) in enumerate(points_xyz):
            plt.text(p[0], p[1], str(i))

    # no interpolation
    if num_points is None:
        return points_xyz

    points_xyz_interpolated = trajectory_interpolate(points_xyz, num_points)
    if visualize:
        plt.plot(
            points_xyz_interpolated[:, 0],  # x
            points_xyz_interpolated[:, 1],  # y
        )
    return points_xyz_interpolated


def create_stl(vertices, faces):
    object = mesh.Mesh(numpy.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
    for i, f in enumerate(faces):
        for j in range(3):
            object.vectors[i][j] = vertices[f[j], :]
    return object


def create_racetrack_faces(left_border, right_border):
    if len(left_border) != len(right_border):
        print('create_racetrack_faces: len(left_border) != len(right_border)')
        sys.exit(1)
    faces = []
    triangle = [0, len(right_border), 1]
    faces.append(triangle)
    last_wall = 0
    for i in range(len(right_border) - 1):
        if last_wall:
            triangle = [triangle[0], triangle[2], triangle[0] + 1]
            faces.append(triangle)
            last_wall = 0
        if ~last_wall:
            triangle = [triangle[2], triangle[1], triangle[1] + 1]
            faces.append(triangle)
            last_wall = 1
    return faces


def create_stl_file(left_wall, right_wall, filename):
    vertices = numpy.concatenate((left_wall, right_wall), axis=0)
    faces = create_racetrack_faces(left_wall, right_wall)
    faces = numpy.array(faces)

    racetrack = create_stl(vertices=vertices, faces=faces)
    # write the mesh to a file
    racetrack.save(filename)


# TODO: replace hardcoded size
def svg_with_path(path: str) -> str:
    xml = f'''<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN" "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
<svg width="200000mm" height="200000mm" viewBox="0 0 200 200" version="1.1" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" xml:space="preserve" xmlns:serif="http://www.serif.com/" style="fill-rule:evenodd;clip-rule:evenodd;stroke-linejoin:round;stroke-miterlimit:2;">
    <path d="{path}" style="fill:rgb(255,82,31);"/>
</svg>
'''
    return xml


# TODO: replace hardcoded size
def points_to_svg(left_points, right_points):
    points_str_arr = []
    for point in left_points:
        x = point[0]
        y = 200.0 - point[1]
        points_str_arr.append(f'{x:.10f},{y:.10f}')
    points_str = 'L'.join(points_str_arr)
    left_path = f'M{points_str}Z'

    points_str_arr = []
    for point in right_points:
        x = point[0]
        y = 200.0 - point[1]
        points_str_arr.append(f'{x:.10f},{y:.10f}')
    points_str = 'L'.join(points_str_arr)
    right_path = f'M{points_str}Z'

    path = left_path + right_path

    return svg_with_path(path)


def get_earth_radius_at_latitude(latitude: float) -> Tuple[float, float]:
    # constants for from GPS Hector plugin
    EQUATORIAL_RADIUS = 6378137.0
    FLATTENING = 1.0 / 298.257223563
    ECCENTRICITY2 = 2.0 * FLATTENING - math.pow(FLATTENING, 2.0)

    # calculate earth radius from GPS Hector plugin
    base_point_latitude_ = math.radians(latitude)
    temp_ = 1.0 / (
        1.0 - ECCENTRICITY2 * math.pow(math.sin(base_point_latitude_), 2.0)
    )
    prime_vertical_radius_ = EQUATORIAL_RADIUS * math.sqrt(temp_)
    radius_north = prime_vertical_radius_ * (1.0 - ECCENTRICITY2) * temp_
    radius_east = prime_vertical_radius_ * math.cos(base_point_latitude_)

    return radius_north, radius_east


if __name__ == '__main__':
    # INPUT:
    left_wall_gps = load_wall('../models/purdue_racetrack/gps_data/purdue_left.csv')
    right_wall_gps = load_wall('../models/purdue_racetrack/gps_data/purdue_right.csv')
    # [longitude, latitude, elevation]
    # GNSS  after transform coordinates of map frame origin [0,0,0]
    base_point_gps_ = [-86.945105, 40.437265, 0.0]

    # constants for from GPS Hector plugin
    # EQUATORIAL_RADIUS = 6378137.0
    # FLATTENING = 1.0 / 298.257223563
    # ECCENTRICITY2 = 2.0 * FLATTENING - math.pow(FLATTENING, 2.0)

    # calculate earth radius from GPS Hector plugin
    # base_point_latitude_ = math.radians(base_point_gps_[1])
    # temp_ = 1.0 / (
    #     1.0 - ECCENTRICITY2 * math.pow(math.sin(base_point_latitude_), 2.0)
    # )
    # prime_vertical_radius_ = EQUATORIAL_RADIUS * math.sqrt(temp_)
    # radius_north_ = prime_vertical_radius_ * (1.0 - ECCENTRICITY2) * temp_
    # radius_east_ = prime_vertical_radius_ * math.cos(base_point_latitude_)

    radius_north_, radius_east_ = get_earth_radius_at_latitude(base_point_gps_[1])

    left_wall_xyz = convert_points(
        points_gps=left_wall_gps,
        base_point_gps=base_point_gps_,
        radius_north=radius_north_,
        radius_east=radius_east_,
        num_points=200,
        visualize=True,
    )

    right_wall_xyz = convert_points(
        points_gps=right_wall_gps,
        base_point_gps=base_point_gps_,
        radius_north=radius_north_,
        radius_east=radius_east_,
        num_points=200,
        visualize=True,
    )

    # plt.show()

    create_stl_file(
        left_wall=left_wall_xyz,
        right_wall=right_wall_xyz,
        filename='gps.local/racetrack.stl',
    )

    svg = points_to_svg(left_wall_xyz, right_wall_xyz)
    print(svg)

pass
