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
from ament_index_python import get_package_share_directory


def load_wall(file):
    points_with_ids = []
    with open(file, newline='') as csv_file:
        csv_reader = csv.DictReader(csv_file, delimiter=',')
        for row in csv_reader:
            # print(row)
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
    num_points=10,
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


def svg_with_path(width: int, height: int, path: str, fill_color: str = 'rgb(255,82,31)') -> str:
    """
    width [m]
    height [m]
    """
    # Note: only mm (millimeters) works as the absolute unit when importing to Blender
    width_in_mm = width * 1000
    height_in_mm = height * 1000
    xml = f'''<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN" "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
<svg width="{width_in_mm}mm" height="{height_in_mm}mm" viewBox="0 0 {width} {height}" version="1.1" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" xml:space="preserve" xmlns:serif="http://www.serif.com/" style="fill-rule:evenodd;clip-rule:evenodd;stroke-linejoin:round;stroke-miterlimit:2;">
    <path d="{path}" style="fill:{fill_color};"/>
</svg>
'''
    return xml


def create_track_svg_parts(bounding_rectangle_size, left_points, right_points):
    """
    All numbers should be floats. Unit is meters [m].
    But only whole numbers are supported for bounding_rectangle_size for now.
    Number of left_points and right_points can be different (N vs M).

    bounding_rectangle_size = [width, height]
    left_points = [[x_1, y_1], ..., [x_N, y_N]]
    right_points = [[x_1, y_1], ..., [x_M, y_M]]
    """

    [width, height] = bounding_rectangle_size

    boundary_points = [
        # the order matters!
        [0.0, 0.0],  # bottom left corner
        [width, 0.0],  # bottom right corner
        [width, height],  # top right corner
        [0.0, height],  # top left corner
    ]

    # Note: SVG coordinate origin [0,0] is top-left corner.
    #       Our coordinate origin [0,0] is bottom-left corner.
    #       So we need to flip vertically (that's the reason for y = height - point[1]).
    def point_to_svg_str(point):
        x = point[0]
        y = height - point[1]
        return f'{x:.10f},{y:.10f}'

    def points_to_path(points):
        points_str_arr = []
        for point in points:
            points_str_arr.append(point_to_svg_str(point))
        points_str = 'L'.join(points_str_arr)
        return f'M{points_str}Z'

    # note: Only integers are supported for SVG (and its viewBox) width/height.
    #       We could support floats as well but ints make things easier.
    w = int(width)
    h = int(height)

    boundary_path = points_to_path(boundary_points)
    left_path = points_to_path(left_points)
    right_path = points_to_path(right_points)

    svg_track_outside = svg_with_path(
        width=w,
        height=h,
        path=boundary_path + left_path,
        fill_color='rgb(255,82,31)',
    )
    svg_track_road = svg_with_path(
        width=w,
        height=h,
        path=left_path + right_path,
        fill_color='rgb(93,93,93)',
    )
    svg_track_inside = svg_with_path(
        width=w,
        height=h,
        path=right_path,
        fill_color='rgb(69,146,44)',
    )

    return svg_track_outside, svg_track_road, svg_track_inside


def save_txt_to_file(filename, text):
    with open(filename, 'w') as f:
        f.write(text)


def get_earth_radius_at_latitude(latitude: float) -> Tuple[float, float]:
    # constants from the GPS Hector plugin
    EQUATORIAL_RADIUS = 6378137.0
    FLATTENING = 1.0 / 298.257223563
    ECCENTRICITY2 = 2.0 * FLATTENING - math.pow(FLATTENING, 2.0)

    # calculate earth radius (same way as in the GPS Hector plugin)
    latitude_rad = math.radians(latitude)
    temp = 1.0 / (
        1.0 - ECCENTRICITY2 * math.pow(math.sin(latitude_rad), 2.0)
    )
    prime_vertical_radius_ = EQUATORIAL_RADIUS * math.sqrt(temp)
    radius_north = prime_vertical_radius_ * (1.0 - ECCENTRICITY2) * temp
    radius_east = prime_vertical_radius_ * math.cos(latitude_rad)

    return radius_north, radius_east


if __name__ == '__main__':
    # INPUTS:
    # GNSS racetrack boundary points
    left_wall_gps_ = load_wall(
        get_package_share_directory('simulator')
    + '/models/pennovation_track/gps_data/left.csv'
    )

    right_wall_gps_ = load_wall(
        get_package_share_directory('simulator')
        + '/models/pennovation_track/gps_data/right.csv'
    )


    # [longitude, latitude, elevation]
    # GNSS coordinates that corresponds to the XYZ coordinates origin point [0,0,0]
    # base_point_gps_ = [-86.945105, 40.437265, 0.0] # Purdue
    base_point_gps_ = [-75.19913, 39.94117, 0.0] # Pennovation

    # If program should create an STL file (using a primitive tesselation algorithm).
    # Does not work good enough. It is better to create an SVG, import it to Blender,
    # and use its tesselation algorithm for STL/DAE creation.
    create_stl_out_ = False
    # END OF INPUTS

    radius_north_, radius_east_ = get_earth_radius_at_latitude(
        latitude=base_point_gps_[1],
    )

    left_wall_xyz_ = convert_points(
        points_gps=left_wall_gps_,
        base_point_gps=base_point_gps_,
        radius_north=radius_north_,
        radius_east=radius_east_,
        num_points=25,
        visualize=True,
    )

    right_wall_xyz_ = convert_points(
        points_gps=right_wall_gps_,
        base_point_gps=base_point_gps_,
        radius_north=radius_north_,
        radius_east=radius_east_,
        num_points=25,
        visualize=True,
    )

    plt.show()

    if create_stl_out_:
        create_stl_file(
            left_wall=left_wall_xyz_,
            right_wall=right_wall_xyz_,
            filename='gps.local/racetrack.stl',
        )

    svg_track_outside, svg_track_road, svg_track_inside = create_track_svg_parts(
        bounding_rectangle_size=[100.0, 100.0],
        left_points=left_wall_xyz_,
        right_points=right_wall_xyz_,
    )
    save_txt_to_file('/home/rithwik/UPenn/GoKart/gokart-simulation/simulator/models/pennovation_track/svg_files/track_outside.local.svg',
        svg_track_outside)
    save_txt_to_file('/home/rithwik/UPenn/GoKart/gokart-simulation/simulator/models/pennovation_track/svg_files/track_road.local.svg',
        svg_track_road)
    save_txt_to_file('/home/rithwik/UPenn/GoKart/gokart-simulation/simulator/models/pennovation_track/svg_files/track_inside.local.svg',
        svg_track_inside)

    pass
