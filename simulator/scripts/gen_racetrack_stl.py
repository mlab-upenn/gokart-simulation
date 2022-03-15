import math
import sys
import csv

from scipy.interpolate import CubicSpline
import numpy
from stl import mesh
import matplotlib.pyplot as plt


def load_wall(file):
    with open(file, newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        rows = list(spamreader)
        wall = []
        for row, idx in zip(rows, range(len(rows))):
            if not idx:
                long_idx = row.index('Longitude')
                lat_idx = row.index('Latitude')
            else:
                wall.append([float(row[long_idx]), float(row[lat_idx]), 0.0])
    return wall


def trajectory_interpolate(points, int_size=20):
    _points = numpy.vstack((points, points[0, :]))

    distance = numpy.cumsum(numpy.sqrt(numpy.sum(numpy.diff(_points, axis=0) ** 2, axis=1)))
    distance = numpy.insert(distance, 0, 0) / distance[-1]
    alpha = numpy.linspace(0, 1, int_size, endpoint=False)
    ipol = CubicSpline(distance, _points, axis=0, bc_type="periodic")(alpha)
    return ipol


def gnss_to_cartesian_test(point, offset, radius_north, radius_east):
    point_centered = point - offset

    x = math.radians(point_centered[0]) * radius_east
    y = math.radians(point_centered[1]) * radius_north
    z = 0.0

    return [x, y, z]


def gnss_to_cartesian(points, offset, radius_north, radius_east):
    offset = [offset]
    offset_ = numpy.repeat(offset, len(points), axis=0)
    # remove offset
    points_ = points - offset_
    # degrees to radians
    points_ = numpy.radians(points_)
    # apply forward Equirectangular projection
    # https://en.wikipedia.org/wiki/Equirectangular_projection#Forward
    points_ = points_ * numpy.repeat([[radius_east, radius_north, 1.0]], len(points), axis=0)
    return points_


def create_stl(vertices, faces):
    object = mesh.Mesh(numpy.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
    for i, f in enumerate(faces):
        for j in range(3):
            object.vectors[i][j] = vertices[f[j], :]
    return object


def create_racetrack_faces(left_border, right_border):
    if len(left_border) != len(right_border):
        print("Error")
        sys.exit()

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


if __name__ == "__main__":
    left_wall = load_wall('purdue_left.csv')
    right_wall = load_wall('purdue_right.csv')
    # [longitude, latitude, elevation]
    base_point_gps = [-86.945105, 40.437265, 0.0]  # GNSS  after transform coordinates of map frame origin [0,0,0]

    # Constants from GPS Hector plugin
    equatorial_radius = 6378137.0
    flattening = 1.0 / 298.257223563
    eccentricity2 = 2.0 * flattening - math.pow(flattening, 2.0)

    # Calculate earth radius from GPS Hector plugin
    base_point_latitude = math.radians(base_point_gps[1])
    temp = 1.0 / (1.0 - eccentricity2 * math.pow(math.sin(base_point_latitude), 2.0))

    prime_vertical_radius = equatorial_radius * math.sqrt(temp)
    radius_north_ = prime_vertical_radius * (1.0 - eccentricity2) * temp
    radius_east_ = prime_vertical_radius * math.cos(base_point_latitude)

    # Pose.x: 55.043982949738 Pose.y: 45.211049994561
    #         55.04398295343817,      45.21104993984664
    # latitude: 40.437672148859 longitude: -86.944456253515

    #out = gnss_to_cartesian_test(numpy.array([-86.944456253515, 40.437672148859, 0.0]), base_point_gps, radius_north_, radius_east_)
    #out2 = gnss_to_cartesian(numpy.array([[-86.944456253515, 40.437672148859, 0.0]]), base_point_gps, radius_north_,
    #                             radius_east_)

    #print(f"OUT  {out}")
    #print(f"OUT2 {out2[0]}")

    #exit()

    print(left_wall[0])
    print(left_wall[20])
    print(right_wall[0])
    print(right_wall[20])

    left_wall = gnss_to_cartesian(
        numpy.array(left_wall),
        base_point_gps,
        radius_north_,
        radius_east_
    )

    right_wall = gnss_to_cartesian(
        numpy.array(right_wall),
        base_point_gps,
        radius_north_,
        radius_east_
    )

    # exit()

    theta = numpy.radians(0)
    c, s = numpy.cos(theta), numpy.sin(theta)
    R = numpy.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))

    plt.plot(left_wall[:, 0], left_wall[:, 1])
    left_wall = trajectory_interpolate(left_wall, 200)
    left_wall = numpy.vstack((left_wall, left_wall[0, :]))
    left_wall = numpy.dot(left_wall, R)
    plt.plot(left_wall[:, 0], left_wall[:, 1])

    plt.plot(right_wall[:, 0], right_wall[:, 1])
    right_wall = trajectory_interpolate(right_wall, 200)
    right_wall = numpy.vstack((right_wall, right_wall[0, :]))
    right_wall = numpy.dot(right_wall, R)
    plt.plot(right_wall[:, 0], right_wall[:, 1])

    print(left_wall[0])
    print(left_wall[20])
    print(right_wall[0])
    print(right_wall[20])
    plt.show()

    vertices = [y for x in [left_wall, right_wall] for y in x]
    faces = create_racetrack_faces(left_wall, right_wall)

    vertices = numpy.array(vertices)
    faces = numpy.array(faces)

    racetrack = create_stl(vertices=vertices, faces=faces)
    # Write the mesh to file "cube.stl"
    racetrack.save('racetrack.stl')
