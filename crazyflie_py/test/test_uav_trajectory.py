"""Test trajectory CSV loading without ROS dependencies."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest


source = Path(__file__).parents[1] / 'crazyflie_py' / 'uav_trajectory.py'
spec = importlib.util.spec_from_file_location('uav_trajectory', source)
uav_trajectory = importlib.util.module_from_spec(spec)
spec.loader.exec_module(uav_trajectory)


@pytest.mark.parametrize('durations', [[1.5], [1.5, 2.0]])
def test_loadcsv(tmp_path, durations):
    """Load every piece and its coefficients from a trajectory CSV."""
    data = np.zeros((len(durations), 33))
    data[:, 0] = durations
    data[:, 1] = np.arange(len(durations)) + 1.0
    data[:, 9] = 2.0
    data[:, 17] = 3.0
    data[:, 25] = 0.25
    filename = tmp_path / 'trajectory.csv'
    np.savetxt(filename, data, delimiter=',', header='duration,x,y,z,yaw')

    trajectory = uav_trajectory.Trajectory()
    trajectory.loadcsv(filename)

    assert trajectory.n_pieces() == len(durations)
    assert trajectory.duration == sum(durations)
    for piece, row in zip(trajectory.polynomials, data):
        assert piece.duration == row[0]
        np.testing.assert_array_equal(piece.px.p, row[1:9])
        np.testing.assert_array_equal(piece.py.p, row[9:17])
        np.testing.assert_array_equal(piece.pz.p, row[17:25])
        np.testing.assert_array_equal(piece.pyaw.p, row[25:33])
