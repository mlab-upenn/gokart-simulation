#!/usr/bin/env python3

import numpy
from gen_racetrack import gnss_to_cartesian, get_earth_radius_at_latitude


if __name__ == '__main__':

    points_gnss_ = numpy.array(
        [
            # [longitude, latitude, elevation]
            [-86.9444403, 40.4376945, 0.0]
        ],
        dtype=numpy.float64,
    )

    # [longitude, latitude, elevation]
    # GNSS  after transform coordinates of map frame origin [0,0,0]
    base_point_gnss_ = [-86.945105, 40.437265, 0.0]

    radius_north_, radius_east_ = get_earth_radius_at_latitude(
        latitude=base_point_gnss_[1],
    )

    points_xyz_ = gnss_to_cartesian(
        points_gnss_,
        base_point_gnss_,
        radius_north_,
        radius_east_,
    )

    print(points_xyz_)
