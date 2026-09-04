"""
Regression test for issue #884.

A high-level service callback that raises must not tear down the whole
crazyflie_server. The guard catches the exception, logs it at ERROR with the
service name and the vehicle namespace, and returns the response so the server
survives and keeps serving the rest of the swarm.

The tests drive the *real* service callbacks with a lightweight stand-in
``self`` whose simulated Crazyflie raises on every high-level command
(mirroring ``CrazyflieSIL.goTo`` raising ``ValueError`` from a low-level mode).
Removing a callback's try/except makes it raise again, so the tests flip
RED -- they are a genuine guard, not a tautology.
"""

import sys
import types

# The guard logic is pure Python; the compiled firmware bindings are irrelevant
# here. Stub the module so crazyflie_server imports without cffirmware present.
sys.modules.setdefault('cffirmware', types.ModuleType('cffirmware'))

from crazyflie_sim.crazyflie_server import CrazyflieServer  # noqa: E402
import pytest  # noqa: E402


class _RaisingCF:
    """A simulated Crazyflie that rejects every high-level command."""

    MESSAGE = 'goTo from low-level modes not yet supported.'

    def _boom(self, *args, **kwargs):
        raise ValueError(self.MESSAGE)

    takeoff = land = goTo = uploadTrajectory = startTrajectory = _boom


class _RecordingLogger:

    def __init__(self):
        self.errors = []
        self.infos = []

    def info(self, msg):
        self.infos.append(str(msg))

    def error(self, msg):
        self.errors.append(str(msg))


class _FakeServer:
    """Minimal stand-in exposing what the callbacks touch on ``self``."""

    def __init__(self, cf):
        self.cfs = {'cf1': cf}
        self._logger = _RecordingLogger()

    def get_logger(self):
        return self._logger


def _duration(sec=2, nanosec=0):
    return types.SimpleNamespace(sec=sec, nanosec=nanosec)


def _takeoff_request():
    return types.SimpleNamespace(height=1.0, duration=_duration(), group_mask=0)


def _land_request():
    return types.SimpleNamespace(height=0.0, duration=_duration(), group_mask=0)


def _go_to_request():
    return types.SimpleNamespace(
        goal=types.SimpleNamespace(x=0.0, y=0.0, z=1.0),
        yaw=0.0, duration=_duration(), relative=False, group_mask=0)


def _upload_trajectory_request():
    piece = types.SimpleNamespace(
        poly_x=[0.0] * 8, poly_y=[0.0] * 8, poly_z=[0.0] * 8,
        poly_yaw=[0.0] * 8, duration=_duration())
    return types.SimpleNamespace(
        trajectory_id=1, piece_offset=0, pieces=[piece])


def _start_trajectory_request():
    return types.SimpleNamespace(
        trajectory_id=1, timescale=1.0, reversed=False,
        relative=False, group_mask=0)


# (callback, request-factory, expected service label) for every high-level
# callback that dispatches a command into the simulated Crazyflie.
DISPATCHING = [
    (CrazyflieServer._takeoff_callback, _takeoff_request, 'takeoff'),
    (CrazyflieServer._land_callback, _land_request, 'land'),
    (CrazyflieServer._go_to_callback, _go_to_request, 'go_to'),
    (CrazyflieServer._upload_trajectory_callback,
     _upload_trajectory_request, 'upload_trajectory'),
    (CrazyflieServer._start_trajectory_callback,
     _start_trajectory_request, 'start_trajectory'),
]

# The five high-level callbacks that dispatch a command into CrazyflieSIL and
# therefore carry an inline try/except guard.


@pytest.mark.parametrize('callback, make_request, service', DISPATCHING)
def test_bad_high_level_call_does_not_kill_server(callback, make_request, service):
    server = _FakeServer(_RaisingCF())
    response = object()

    # The callback must NOT propagate the exception -- propagation is exactly
    # what tears the server down in issue #884.
    result = callback(server, make_request(), response, name='cf1')

    assert result is response, 'callback must return its response after a rejected call'
    assert server.get_logger().errors, 'a rejected call must be logged at ERROR'
    logged = server.get_logger().errors[-1]
    assert service in logged, f'error log must name the service ({service})'
    assert 'cf1' in logged, 'error log must name the vehicle namespace'


def test_server_keeps_serving_after_a_bad_call():
    """After a rejected call the same server still handles further requests."""
    server = _FakeServer(_RaisingCF())
    response = object()

    first = CrazyflieServer._go_to_callback(
        server, _go_to_request(), response, name='cf1')
    assert first is response

    # A second bad call is handled just the same -- the server is still alive.
    second = CrazyflieServer._takeoff_callback(
        server, _takeoff_request(), response, name='cf1')
    assert second is response
    assert len(server.get_logger().errors) == 2


def test_success_path_is_transparent():
    """On the happy path the guard is a no-op: response returned, no error."""
    class _GoodCF:
        def goTo(self, *args, **kwargs):
            return None

    server = _FakeServer(_GoodCF())
    response = object()
    result = CrazyflieServer._go_to_callback(
        server, _go_to_request(), response, name='cf1')
    assert result is response
    assert not server.get_logger().errors, 'no error must be logged on success'
