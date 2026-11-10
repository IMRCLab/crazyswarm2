#!/usr/bin/env python3

"""Run a minimal Crazyflie server with takeoff and land services, using cflib2."""

import asyncio
from functools import partial
import threading

from cflib2 import Crazyflie, LinkContext
from cflib2.toc_cache import FileTocCache
from crazyflie_interfaces.srv import Land, Takeoff
import rclpy
from rclpy.node import Node


class CrazyflieServer(Node):
    """ROS 2 node that bridges Crazyflie firmware and ROS services."""

    def __init__(self):
        """Initialize the Crazyflie ROS 2 server node."""
        super().__init__(
            'crazyflie_server',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # Turn ROS parameters into a dictionary
        self._ros_parameters = self._param_to_dict(self._parameters)

        self.uris = []
        # assign a all -> all mapping
        self.cf_dict = {'all': 'all'}
        self.uri_dict = {}
        self.type_dict = {}
        self.cfs = {}

        # Get the robot data from the ROS parameters
        robot_data = self._ros_parameters['robots']

        # Create easy lookup tables for uri, name and types
        for crazyflie in robot_data:
            if robot_data[crazyflie]['enabled']:
                type_cf = robot_data[crazyflie]['type']
                # do not include virtual objects
                connection = self._ros_parameters['robot_types'][type_cf].get(
                    'connection', 'crazyflie'
                )
                if connection == 'crazyflie':
                    uri = robot_data[crazyflie]['uri']
                    self.uris.append(uri)
                    self.cf_dict[uri] = crazyflie
                    self.uri_dict[crazyflie] = uri
                    self.type_dict[uri] = type_cf

        # cflib2 is async-first: run an asyncio loop in a background thread and
        # schedule all cflib2 calls onto it. The loop is a daemon thread, so it
        # is stopped when the node exits.
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._loop_thread.start()

        # Connect to all Crazyflies (blocks until all of them are connected)
        connect_future = asyncio.run_coroutine_threadsafe(self._connect(), self._loop)
        try:
            connect_future.result()
        except Exception as e:
            for line in str(e).splitlines():
                self.get_logger().error(line)
            exit()

        self._init_services()

        self.get_logger().info('All Crazyflies are connected!')

    async def _connect(self):
        """Connect to all Crazyflies through a shared link context."""
        self._link_context = LinkContext()
        toc_cache = FileTocCache('./cache')
        results = await asyncio.gather(
            *[
                Crazyflie.connect_from_uri(self._link_context, uri, toc_cache)
                for uri in self.uris
            ],
            return_exceptions=True,
        )
        self.cfs = {}
        errors = []
        for uri, result in zip(self.uris, results):
            if isinstance(result, BaseException):
                errors.append((uri, result))
            else:
                self.cfs[uri] = result
        if errors:
            # Disconnect the Crazyflies that did connect before failing
            await asyncio.gather(
                *[cf.disconnect() for cf in self.cfs.values()], return_exceptions=True
            )
            self.cfs = {}
            details = '\n'.join(
                f'[{self.cf_dict[uri]}] connecting to {uri} failed: {error}'
                for uri, error in errors
            )
            raise ConnectionError(
                'Error!: One or more Crazyflies can not be found:\n' + details
            )

    def _param_to_dict(self, param_ros):
        """Turn ROS 2 parameters from the node into a dict."""
        tree = {}
        for item in param_ros:
            t = tree
            for part in item.split('.'):
                if part == item.split('.')[-1]:
                    t = t.setdefault(part, param_ros[item].value)
                else:
                    t = t.setdefault(part, {})
        return tree

    def _init_services(self):
        """Create takeoff and land services for each crazyflie and all."""
        for uri in self.cf_dict:
            if uri == 'all':
                continue

            name = self.cf_dict[uri]

            self.create_service(
                Takeoff, name + '/takeoff', partial(self._takeoff_callback, uri=uri)
            )
            self.create_service(Land, name + '/land', partial(self._land_callback, uri=uri))

        self.create_service(Takeoff, 'all/takeoff', self._takeoff_callback)
        self.create_service(Land, 'all/land', self._land_callback)

    def _takeoff_callback(self, request, response, uri='all'):
        """Handle a high-level takeoff request."""
        duration = float(request.duration.sec) + float(request.duration.nanosec / 1e9)
        self.get_logger().info(
            f'[{self.cf_dict[uri]}] takeoff(height={request.height} m,'
            + f'duration={duration} s,'
            + f'group_mask={request.group_mask})'
        )
        self._schedule(self._takeoff(uri, request.height, duration, request.group_mask))

        return response

    def _land_callback(self, request, response, uri='all'):
        """Handle a high-level land request."""
        duration = float(request.duration.sec) + float(request.duration.nanosec / 1e9)
        self.get_logger().info(
            f'[{self.cf_dict[uri]}] land(height={request.height} m,'
            + f'duration={duration} s,'
            + f'group_mask={request.group_mask})'
        )
        self._schedule(self._land(uri, request.height, duration, request.group_mask))

        return response

    def _schedule(self, coroutine):
        """Schedule a cflib2 coroutine on the background asyncio loop."""
        future = asyncio.run_coroutine_threadsafe(coroutine, self._loop)
        future.add_done_callback(self._log_async_error)

    def _log_async_error(self, future):
        """Log an error if a scheduled cflib2 call failed."""
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f'cflib2 call failed: {e}')

    async def _takeoff(self, uri, height, duration, group_mask):
        """Send a high-level takeoff command to one or all Crazyflies."""
        if uri == 'all':
            await asyncio.gather(
                *[
                    cf.high_level_commander().take_off(height, None, duration, group_mask)
                    for cf in self.cfs.values()
                ]
            )
        else:
            await self.cfs[uri].high_level_commander().take_off(
                height, None, duration, group_mask
            )

    async def _land(self, uri, height, duration, group_mask):
        """Send a high-level land command to one or all Crazyflies."""
        if uri == 'all':
            await asyncio.gather(
                *[
                    cf.high_level_commander().land(height, None, duration, group_mask)
                    for cf in self.cfs.values()
                ]
            )
        else:
            await self.cfs[uri].high_level_commander().land(height, None, duration, group_mask)


def main(args=None):
    """Start the Crazyflie server node."""
    rclpy.init(args=args)
    crazyflie_server = CrazyflieServer()

    rclpy.spin(crazyflie_server)

    crazyflie_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
